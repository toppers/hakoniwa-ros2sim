using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class ButtonSensor : MonoBehaviour, IRobotPartsSensor
    {
        private string root_name;
        private IPduWriter pdu_writer;
        private PduIoConnector pdu_io;

        private GameObject root;


        public GameObject left;
        public GameObject right;
        public GameObject up;
        public GameObject down;
        public GameObject enter;
        public GameObject back;

        private IRobotPartsTouchSensor button_left;
        private IRobotPartsTouchSensor button_right;
        private IRobotPartsTouchSensor button_up;
        private IRobotPartsTouchSensor button_down;
        private IRobotPartsTouchSensor button_enter;
        private IRobotPartsTouchSensor button_back;

        private IRobotPartsTouchSensor GetTouchSensor(GameObject parent)
        {
            var ret = parent.GetComponentInChildren<IRobotPartsTouchSensor>();
            if (ret == null)
            {
                throw new ArgumentException("can not found button_left:" + parent.name);
            }
            ret.Initialize(this.root);
            return ret;
        }

        public void Initialize(GameObject root)
        {
            if (this.root != null)
            {
                return;
            }
            this.root = root;

            this.root_name = string.Copy(this.root.transform.name);
            this.pdu_io = PduIoConnector.Get(this.root_name);
            this.pdu_writer = this.pdu_io.GetWriter(this.root_name + "_ev3_sensorPdu");
            if (this.pdu_writer == null)
            {
                throw new ArgumentException("can not found ev3_sensor pdu:" + this.root_name + "_ev3_sensorPdu");
            }
            this.button_left = GetTouchSensor(left);
            this.button_right = GetTouchSensor(right);
            this.button_up = GetTouchSensor(up);
            this.button_down = GetTouchSensor(down);
            this.button_enter = GetTouchSensor(enter);
            this.button_back = GetTouchSensor(back);

        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            return null;
        }


        public bool isAttachedSpecificController()
        {
            return false;
        }

        public void UpdateSensorValues()
        {
            this.UpdateSensorValuesLocal();
        }

        private void UpdateSensorValuesLocal()
        {
            byte[] button_value = new byte[1];
            button_value[0] = 0;

            button_left.UpdateSensorValues();
            if (this.button_left.IsPressed())
            {
                button_value[0] = (byte)(1U << 0);
            }
            button_right.UpdateSensorValues();
            if (this.button_right.IsPressed())
            {
                button_value[0] = (byte)(1U << 1);
            }
            button_up.UpdateSensorValues();
            if (this.button_up.IsPressed())
            {
                button_value[0] = (byte)(1U << 2);
            }
            button_down.UpdateSensorValues();
            if (this.button_down.IsPressed())
            {
                button_value[0] = (byte)(1U << 3);
            }
            button_enter.UpdateSensorValues();
            if (this.button_enter.IsPressed())
            {
                button_value[0] = (byte)(1U << 4);
            }
            button_back.UpdateSensorValues();
            if (this.button_back.IsPressed())
            {
                button_value[0] = (byte)(1U << 5);
            }
            pdu_writer.GetWriteOps().SetData("buttons", button_value);
        }
    }

}
