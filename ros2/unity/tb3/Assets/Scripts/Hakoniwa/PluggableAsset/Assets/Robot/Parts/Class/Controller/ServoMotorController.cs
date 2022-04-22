using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public class ServoMotorController : MonoBehaviour, IRobotPartsController
    {
        private GameObject root;
        private string root_name;
        private IRobotPartsMotor motor;
        private PduIoConnector pdu_io;
        private IPduReader pdu_reader;

        public string topic_type = "geometry_msgs/Twist";
        public string topic_name = "servo_angle";
        public int update_cycle = 1;
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
                this.motor = this.GetComponent<IRobotPartsMotor>();
                Debug.Log("servo motor=" + this.motor);
            }
            this.motor.Initialize(root);
            this.count = 0;
        }

        public float motorRotateForceScale = 1.0f;
        public void DoControl()
        {
            this.count++;
            if (this.count < this.update_cycle)
            {
                return;
            }
            this.count = 0;
            float target_rotation_angle_rate;

            target_rotation_angle_rate = (float)this.pdu_reader.GetReadOps().Ref("angular").GetDataFloat64("y") * motorRotateForceScale;

            if (this.motor != null)
            {
                //Debug.Log("topic_name=" + this.topic_name + " angular.y=" + target_rotation_angle_rate);
                motor.SetTargetVelicty(target_rotation_angle_rate);
            }
        }

    }
}

