using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    enum PincherFingerMotorType {
        MotorType_A = 0,
        MotorType_B,
        MotorType_Num
    }
    public class PincherFingerController : MonoBehaviour, IRobotPartsController
    {
        private GameObject root;
        private string root_name;
        private IRobotPartsPincherFinger[] motors = new IRobotPartsPincherFinger[(int)PincherFingerMotorType.MotorType_Num];
        private PduIoConnector pdu_io;
        private IPduReader pinch_pdu_reader;

        public string pinch_topic_type = "geometry_msgs/Twist";
        public string pinch_topic_name = "pincher_cmd";

        public int update_cycle = 10;

        private int count = 0;

        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[1];
            cfg[0] = new RosTopicMessageConfig();
            cfg[0].topic_message_name = this.pinch_topic_name;
            cfg[0].topic_type_name = this.pinch_topic_type;
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
                var pdu_reader_name = root_name + "_" + this.pinch_topic_name + "Pdu";
                this.pinch_pdu_reader = this.pdu_io.GetReader(pdu_reader_name);
                if (this.pinch_pdu_reader == null)
                {
                    throw new ArgumentException("can not found pdu_reader:" + pdu_reader_name);
                }

                this.motors[(int)PincherFingerMotorType.MotorType_A] = this.transform.Find("Interface-A").GetComponentInChildren<IRobotPartsPincherFinger>();
                this.motors[(int)PincherFingerMotorType.MotorType_B] = this.transform.Find("Interface-B").GetComponentInChildren<IRobotPartsPincherFinger>();
                Debug.Log("pincher A=" + this.motors[(int)PincherFingerMotorType.MotorType_A]);
                Debug.Log("pincher B=" + this.motors[(int)PincherFingerMotorType.MotorType_B]);
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

        public void DoControl()
        {
            this.count++;
            if (this.count < this.update_cycle)
            {
                return;
            }
            this.count = 0;
            DoControlPinch();

        }

        private void DoControlPinch()
        {
            float target = (float)this.pinch_pdu_reader.GetReadOps().Ref("linear").GetDataFloat64("x");
            //Debug.Log("Pinch: grpi =" + grip);

            if (this.motors[(int)MotorType.MotorType_Right] != null)
            {
                motors[(int)PincherFingerMotorType.MotorType_A].UpdateGrip(target);
            }
            if (this.motors[(int)MotorType.MotorType_Left] != null)
            {
                motors[(int)PincherFingerMotorType.MotorType_B].UpdateGrip(target);
            }
        }

    }
}

