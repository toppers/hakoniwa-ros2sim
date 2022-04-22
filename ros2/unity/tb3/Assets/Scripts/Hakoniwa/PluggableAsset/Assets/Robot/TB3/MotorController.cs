using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.TB3
{
    public class UpdateDeviceCycle
    {
        public int count;
        public int cycle;
        public UpdateDeviceCycle(int c)
        {
            this.count = 0;
            this.cycle = c;
        }
    }
    enum MotorType
    {
        MotorType_Right = 0,
        MotorType_Left,
        MotorType_Num
    }
    public class MotorController : MonoBehaviour, IRobotPartsController
    {
        private GameObject root;
        private string root_name;
        private IPduWriter pdu_joint_state;
        private PduIoConnector pdu_io;
        private IPduReader pdu_motor_control;
        private long current_timestamp;

        private IRobotPartsMotor[] motors = new IRobotPartsMotor[2];      // 0: R, 1: L
        private IRobotPartsMotorSensor[] motor_sensors = new IRobotPartsMotorSensor[2];      // 0: R, 1: L
        private float[] prev_angle = new float[2];  // 0: R, 1: L
        private float[] delta_angle = new float[2];  // 0: R, 1: L
        private float[] moving_distance = new float[2];  // 0: R, 1: L
        private int motor_power = 500;
        private float motor_interval_distance = 0.160f; // 16cm
        private ParamScale scale;
        private Dictionary<string, UpdateDeviceCycle> device_update_cycle = new Dictionary<string, UpdateDeviceCycle>();

        private float steering_sensitivity = 1.5f;                // 経験値

        private IRobotPartsMotorSensor GetRightMotor()
        {
            return motor_sensors[0];
        }
        private IRobotPartsMotorSensor GetLeftMotor()
        {
            return motor_sensors[1];
        }

        public void Initialize(GameObject root)
        {
            if (this.root == null)
            {
                Debug.Log("MotorController Init");
                this.root = root;
                this.root_name = string.Copy(this.root.transform.name);
                this.pdu_io = PduIoConnector.Get(this.root_name);
                this.scale = AssetConfigLoader.GetScale();

                this.pdu_motor_control = this.pdu_io.GetReader(this.root_name + "_cmd_velPdu");
                if (this.pdu_motor_control == null)
                {
                    throw new ArgumentException("can not found CmdVel pdu:" + this.root_name + "_cmd_velPdu");
                }


                this.pdu_joint_state = this.pdu_io.GetWriter(this.root_name + "_joint_statesPdu");
                if (this.pdu_joint_state == null)
                {
                    throw new ArgumentException("can not found joint_states pdu:" + this.root_name + "_joint_statesPdu");
                }
                this.pdu_joint_state.GetWriteOps().Ref("header").SetData("frame_id", "");
                string[] joint_names = new string[2];
                joint_names[0] = "wheel_left_joint";
                joint_names[1] = "wheel_right_joint";
                this.pdu_joint_state.GetWriteOps().SetData("name", joint_names);

                //Debug.Log("left_link=" + this.transform.Find("wheel_left_link"));

                this.motors[(int)MotorType.MotorType_Left] = this.transform.Find("wheel_left_link").GetComponentInChildren<IRobotPartsMotor>();
                this.motors[(int)MotorType.MotorType_Right] = this.transform.Find("wheel_right_link").GetComponentInChildren<IRobotPartsMotor>();
                this.motor_sensors[(int)MotorType.MotorType_Left] = this.transform.Find("wheel_left_link").GetComponentInChildren<IRobotPartsMotorSensor>();
                this.motor_sensors[(int)MotorType.MotorType_Right] = this.transform.Find("wheel_right_link").GetComponentInChildren<IRobotPartsMotorSensor>();
                int update_cycle = 1;
                for (int i = 0; i < this.motors.Length; i++)
                {
                    if (this.motors[i] != null)
                    {
                        this.motors[i].Initialize(root);
                        this.motors[i].SetForce(this.motor_power);
                        string devname_actuator = "motor_actuator" + i.ToString();
                        this.device_update_cycle[devname_actuator] = new UpdateDeviceCycle(update_cycle);
                    }
                    if (this.motor_sensors[i] != null)
                    {
                        this.motor_sensors[i].Initialize(root);
                        string devname_sensor = "motor_sensor" + i.ToString();
                        this.device_update_cycle[devname_sensor] = new UpdateDeviceCycle(update_cycle);
                    }
                }
            }
            else
            {

            }
        }

        private void CopySensingDataToPdu()
        {
            for (int i = 0; i < 2; i++)
            {
                string devname_sensor = "motor_sensor" + i.ToString();
                device_update_cycle[devname_sensor].count++;
                if (device_update_cycle[devname_sensor].count >= device_update_cycle[devname_sensor].cycle)
                {
                    this.motor_sensors[i].UpdateSensorValues();
                    var angle = motor_sensors[i].GetDegree();
                    this.delta_angle[i] = angle - this.prev_angle[i];
                    this.prev_angle[i] = angle;

                    this.moving_distance[i] = ((Mathf.Deg2Rad * this.delta_angle[i]) / Mathf.PI) * motor_sensors[i].GetRadius();
                    //Debug.Log("d[" + i + "]=" + this.moving_distance[i]);
                    //Debug.Log("delta_angle[" + i + "]=" + this.delta_angle[i]);
                    device_update_cycle[devname_sensor].count = 0;
                }
            }
        }
        private void DoActuation()
        {
            double target_velocity;
            double target_rotation_angle_rate;

            target_velocity = this.pdu_motor_control.GetReadOps().Ref("linear").GetDataFloat64("x") * this.scale.cmdvel;
            target_rotation_angle_rate = this.pdu_motor_control.GetReadOps().Ref("angular").GetDataFloat64("z");

            //Debug.Log("scale.cmdvel=" + this.scale.cmdvel);
            //Debug.Log("target_velocity=" + target_velocity);
            //Debug.Log("target_rotation_angle_rate=" + target_rotation_angle_rate);
            // V_R(右車輪の目標速度) = V(目標速度) + d × ω(目標角速度)
            // V_L(左車輪の目標速度) = V(目標速度) - d × ω(目標角速度)

            for (int i = 0; i < 2; i++)
            {
                string devname_actuator = "motor_actuator" + i.ToString();
                device_update_cycle[devname_actuator].count++;
                if (device_update_cycle[devname_actuator].count >= device_update_cycle[devname_actuator].cycle)
                {
                    if (i == 0)
                    {
                        motors[i].SetTargetVelicty((float)(target_velocity + (steering_sensitivity * target_rotation_angle_rate * motor_interval_distance / 2)));
                    }
                    else
                    {
                        motors[i].SetTargetVelicty((float)(target_velocity - (steering_sensitivity * target_rotation_angle_rate * motor_interval_distance / 2)));
                    }
                    device_update_cycle[devname_actuator].count = 0;
                }
            }

            this.PublishJointStates();
        }

        private float GetDeltaMovingDistance()
        {
            return ((this.moving_distance[0] + this.moving_distance[1]) / 2.0f);
        }

        public string[] topic_type = {
            "sensor_msgs/JointState",
            "geometry_msgs/Twist"
        };
        public string[] topic_name = {
            "joint_states",
             "cmd_vel"
        };
        public int[] update_cycle = {
            10,
            10
        };
        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[topic_type.Length];
            int i = 0;
            for (i = 0; i < topic_type.Length; i++)
            {
                cfg[i] = new RosTopicMessageConfig();
                cfg[i].topic_message_name = this.topic_name[i];
                cfg[i].topic_type_name = this.topic_type[i];
                if (cfg[i].topic_message_name == "cmd_vel")
                {
                    cfg[i].sub = true;
                }
                else
                {
                    cfg[i].sub = false;
                }
                cfg[i].pub_option = new RostopicPublisherOption();
                cfg[i].pub_option.cycle_scale = this.update_cycle[i];
                cfg[i].pub_option.latch = false;
                cfg[i].pub_option.queue_size = 1;
            }
            return cfg;
        }
        public void DoControl()
        {
            this.CopySensingDataToPdu();
            this.DoActuation();
        }
        private void PublishJointStates()
        {
            //ROS 0: left,  1: right
            TimeStamp.Set(this.current_timestamp, this.pdu_joint_state.GetWriteOps().Ref(null));
            //position
            double[] position = new double[2];
            position[0] = this.GetLeftMotor().GetCurrentAngle() * Mathf.Deg2Rad;
            position[1] = this.GetRightMotor().GetCurrentAngle() * Mathf.Deg2Rad;


            //velocity
            double[] velocity = new double[2];
            velocity[0] = this.GetLeftMotor().GetCurrentAngleVelocity() * Mathf.Deg2Rad;
            velocity[1] = this.GetRightMotor().GetCurrentAngleVelocity() * Mathf.Deg2Rad;

            //effort
            double[] effort = new double[2];
            effort[0] = 0.0f;
            effort[1] = 0.0f;

            //Set PDU
            this.pdu_joint_state.GetWriteOps().SetData("position", position);
            this.pdu_joint_state.GetWriteOps().SetData("velocity", velocity);
            this.pdu_joint_state.GetWriteOps().SetData("effort", effort);
        }
    }
}
