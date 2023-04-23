using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using Assets.Scripts.Hakoniwa.PluggableAsset.Assets.Robot;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public class TouchSensor : MonoBehaviour, IRobotPartsSensor, IRobotPartsConfig
    {
        private GameObject root;
        private GameObject sensor;
        private PduIoConnector pdu_io;
        private IPduWriter pdu_writer;
        private string root_name;
        private string sensor_name;
        public bool isTouched = false;

        public string topic_type = "std_msgs/Bool";
        public int update_cycle = 100;
        public string topic_name = "touch_sensor";
        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[1];
            cfg[0] = new RosTopicMessageConfig();
            cfg[0].topic_type_name = this.topic_type;
            cfg[0].topic_message_name = this.topic_name;
            cfg[0].sub = false;
            cfg[0].pub_option = new RostopicPublisherOption();
            cfg[0].pub_option.cycle_scale = this.update_cycle;
            cfg[0].pub_option.latch = false;
            cfg[0].pub_option.queue_size = 1;
            return cfg;
        }

        public void Initialize(GameObject root)
        {
            if (this.root == null)
            {
                this.root = root;
                this.root_name = string.Copy(this.root.transform.name);
                this.sensor_name = string.Copy(this.transform.name);
                this.pdu_io = PduIoConnector.Get(root_name);
                if (this.pdu_io == null)
                {
                    throw new ArgumentException("can not found pdu_io:" + root_name);
                }
                var pdu_writer_name = root_name + "_" + this.topic_name + "Pdu";
                this.pdu_writer = this.pdu_io.GetWriter(pdu_writer_name);
                if (this.pdu_writer == null)
                {
                    throw new ArgumentException("can not found pdu_reader:" + pdu_writer_name);
                }
                this.sensor = this.gameObject;
            }
        }

        public bool isAttachedSpecificController()
        {
            return false;
        }

        public void UpdateSensorValues()
        {
            this.pdu_writer.GetWriteOps().SetData("data", isTouched);
        }
        void OnTriggerEnter(Collider t)
        {
            if (t.gameObject.name == "GrabVolumeCone")
            {
                return;
            }
            this.isTouched = true;
            Debug.Log("ENTER:" + t.gameObject.name);
        }
        void OnTriggerStay(Collider t)
        {
            if (t.gameObject.name == "GrabVolumeCone")
            {
                return;
            }
            this.isTouched = true;
            Debug.Log("STAY:" + t.gameObject.name);
        }

        private void OnTriggerExit(Collider t)
        {
            if (t.gameObject.name == "GrabVolumeCone")
            {
                return;
            }
            this.isTouched = false;
            Debug.Log("EXIT:" + t.gameObject.name);
        }
        public IoMethod io_method = IoMethod.RPC;
        public CommMethod comm_method = CommMethod.UDP;
        public RoboPartsConfigData[] GetRoboPartsConfig()
        {
            RoboPartsConfigData[] configs = new RoboPartsConfigData[1];
            configs[0] = new RoboPartsConfigData();
            configs[0].io_dir = IoDir.WRITE;
            configs[0].io_method = this.io_method;
            configs[0].value.org_name = this.topic_name;
            configs[0].value.type = this.topic_type;
            configs[0].value.class_name = ConstantValues.pdu_writer_class;
            configs[0].value.conv_class_name = ConstantValues.conv_pdu_writer_class;
            configs[0].value.pdu_size = ConstantValues.Bool_pdu_size;
            configs[0].value.write_cycle = this.update_cycle;
            configs[0].value.method_type = this.comm_method.ToString();
            return configs;
        }
    }
}