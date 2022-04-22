using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    enum MotorType {
        MotorType_Left = 0,
        MotorType_Right,
        MotorType_Num
    }
    public class DifferentialMotorController : MonoBehaviour, IRobotPartsController
    {
        private GameObject root;
        private string root_name;
        private IRobotPartsMotor [] motors = new IRobotPartsMotor[(int)MotorType.MotorType_Num];
        private PduIoConnector pdu_io;
        private IPduReader pdu_reader;
        private float steering_sensitivity = 1.5f;                // 経験値

        public string topic_type = "geometry_msgs/Twist";
        public string topic_name = "cmd_vel";
        public int update_cycle = 10;
        public float motor_interval_distance = 0.160f; // 16cm

        private int count = 0;

        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[1];
            cfg[0] = new RosTopicMessageConfig();
            cfg[0].topic_message_name = this.topic_name;
            cfg[0].topic_type_name = this.topic_type;
            cfg[0].sub = true;
            return cfg;
        }
        public void Initialize(GameObject root)
        {
            if (this.root == null)
            {
                this.root = root;
                this.root_name = string.Copy(this.root.transform.name);
                this.pdu_io = PduIoConnector.Get(root_name);
                if (this.pdu_io == null)
                {
                    throw new ArgumentException("can not found pdu_io:" + root_name);
                }
                var pdu_reader_name = root_name + "_" + this.topic_name + "Pdu";
                this.pdu_reader = this.pdu_io.GetReader(pdu_reader_name);
                if (this.pdu_reader == null)
                {
                    throw new ArgumentException("can not found pdu_reader:" + pdu_reader_name);
                }
                this.motors[(int)MotorType.MotorType_Left] = this.transform.Find("Interface-L").GetComponentInChildren<IRobotPartsMotor>();
                this.motors[(int)MotorType.MotorType_Right] = this.transform.Find("Interface-R").GetComponentInChildren<IRobotPartsMotor>();
                Debug.Log("motor left=" + this.motors[(int)MotorType.MotorType_Left]);
                Debug.Log("motor right=" + this.motors[(int)MotorType.MotorType_Right]);
            }
            for (int i = 0; i < this.motors.Length; i++)
            {
                if (this.motors[i] != null)
                {
                    this.motors[i].Initialize(root);
                }
            }
            this.count = 0;
        }

        public float motorFowardForceScale = 1.0f;
        public float motorRotateForceScale = 10.0f;
        public void DoControl()
        {
            this.count++;
            if (this.count < this.update_cycle)
            {
                return;
            }
            this.count = 0;
            double target_velocity;
            double target_rotation_angle_rate;

            target_velocity = this.pdu_reader.GetReadOps().Ref("linear").GetDataFloat64("x") * motorFowardForceScale;
            target_rotation_angle_rate = this.pdu_reader.GetReadOps().Ref("angular").GetDataFloat64("z") * motorRotateForceScale;

            //Debug.Log("target_velocity=" + this.pdu_reader.GetReadOps().Ref("linear").GetDataFloat64("x"));
            //Debug.Log("target_rotation_angle_rate=" + target_rotation_angle_rate);
            //Debug.Log("target_rotation_angle_rate=" + target_rotation_angle_rate);
            // V_R(右車輪の目標速度) = V(目標速度) + d × ω(目標角速度)
            // V_L(左車輪の目標速度) = V(目標速度) - d × ω(目標角速度)

            if (this.motors[(int)MotorType.MotorType_Right] != null)
            {
                motors[(int)MotorType.MotorType_Right].SetTargetVelicty((float)(target_velocity + (steering_sensitivity * target_rotation_angle_rate * motor_interval_distance / 2)));
            }
            if (this.motors[(int)MotorType.MotorType_Left] != null)
            {
                //Debug.Log("target_velocity=" + target_velocity);
                motors[(int)MotorType.MotorType_Left].SetTargetVelicty((float)(target_velocity - (steering_sensitivity * target_rotation_angle_rate * motor_interval_distance / 2)));
            }
        }

    }
}

