using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
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
    public class TrainMotorController : MonoBehaviour, IRobotPartsController
    {
        public int motorNo = 2;
        private GameObject root;
        private string root_name;
        private IPduReader pdu_reader;
        private IPduWriter pdu_writer;
        private PduIoConnector pdu_io;
        private long current_timestamp;

        private IRobotPartsMotor[] motors = new IRobotPartsMotor[2];      // 0: R, 1: L
        private IRobotPartsMotorSensor[] motor_sensors = new IRobotPartsMotorSensor[2];      // 0: R, 1: L
        private float[] prev_angle = new float[2];  // 0: R, 1: L
        private float[] delta_angle = new float[2];  // 0: R, 1: L
        private float[] moving_distance = new float[2];  // 0: R, 1: L
        private int motor_power = 100000;
        private ParamScale scale;
        private Dictionary<string, UpdateDeviceCycle> device_update_cycle = new Dictionary<string, UpdateDeviceCycle>();


        public void Initialize(GameObject root)
        {
            if (this.root != null)
            {
                return;
            }
            Debug.Log("TrainMotorController Init");
            this.root = root;
            this.root_name = string.Copy(this.root.transform.name);
            this.pdu_io = PduIoConnector.Get(this.root_name);
            this.pdu_reader = this.pdu_io.GetReader(this.root_name + "_ev3_actuatorPdu");
            if (this.pdu_reader == null)
            {
                throw new ArgumentException("can not found _ev3_actuator pdu:" + this.root_name + "_ev3_actuatorPdu");
            }
            this.pdu_writer = this.pdu_io.GetWriter(this.root_name + "_ev3_sensorPdu");
            if (this.pdu_writer == null)
            {
                throw new ArgumentException("can not found ev3_sensor pdu:" + this.root_name + "_ev3_sensorPdu");
            }

            this.scale = AssetConfigLoader.GetScale();

            string[] joint_names = new string[2];
            joint_names[0] = "wheel_left_joint";
            joint_names[1] = "wheel_right_joint";

            //Debug.Log("left_link=" + this.transform.Find("wheel_left_link"));

            this.motors[(int)MotorType.MotorType_Left] = this.transform.Find("Tire-L").GetComponentInChildren<IRobotPartsMotor>();
            this.motors[(int)MotorType.MotorType_Right] = this.transform.Find("Tire-R").GetComponentInChildren<IRobotPartsMotor>();
            this.motor_sensors[(int)MotorType.MotorType_Left] = this.transform.Find("Tire-L").GetComponentInChildren<IRobotPartsMotorSensor>();
            this.motor_sensors[(int)MotorType.MotorType_Right] = this.transform.Find("Tire-R").GetComponentInChildren<IRobotPartsMotorSensor>();
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

        public void DoControl()
        {
            this.CopySensingDataToPdu();
            this.DoActuation();
        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            return null;
        }
        private void CopySensingDataToPdu()
        {
            for (int i = 0; i < 2; i++)
            {
                motor_sensors[i].UpdateSensorValues();
            }

            uint[] motor_angles = this.pdu_writer.GetReadOps().GetDataUInt32Array("motor_angle");
            motor_angles[this.motorNo] = (uint)motor_sensors[0].GetDegree();
            this.pdu_writer.GetWriteOps().SetData("motor_angle", motor_angles);
        }
        private void DoActuation()
        {
            var power = this.pdu_reader.GetReadOps().Refs("motors")[this.motorNo].GetDataInt32("power");
            //int power = 40;
            for (int i = 0; i < 2; i++)
            {
                string devname_actuator = "motor_actuator" + i.ToString();
                device_update_cycle[devname_actuator].count++;
                if (device_update_cycle[devname_actuator].count >= device_update_cycle[devname_actuator].cycle)
                {
                    motors[i].SetTargetVelicty(power);
                    device_update_cycle[devname_actuator].count = 0;
                }
            }

        }
    }

}
