using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class ButtonSensor
    {
        private GameObject root;
        private string roboname;
        private IRobotTouchSensor button_left;
        private IRobotTouchSensor button_right;
        private IRobotTouchSensor button_up;
        private IRobotTouchSensor button_down;
        private IRobotTouchSensor button_enter;
        private IRobotTouchSensor button_back;

        public ButtonSensor(GameObject obj, string name)
        {
            this.root = obj;
            this.roboname = name;
        }

        internal void Initialize(IEV3Parts parts)
        {
            GameObject obj;
            string subParts = parts.getButtonSensor(ButtonSensorType.BUTTON_SENSOR_LEFT);
            if (subParts != null)
            {
                obj = root.transform.Find(this.roboname + "/" + subParts).gameObject;
                this.button_left = obj.GetComponentInChildren<IRobotTouchSensor>();
                this.button_left.Initialize(obj);
            }
            subParts = parts.getButtonSensor(ButtonSensorType.BUTTON_SENSOR_RIGHT);
            if (subParts != null)
            {
                obj = root.transform.Find(this.roboname + "/" + subParts).gameObject;
                this.button_right = obj.GetComponentInChildren<IRobotTouchSensor>();
                this.button_right.Initialize(obj);
            }
            subParts = parts.getButtonSensor(ButtonSensorType.BUTTON_SENSOR_UP);
            if (subParts != null)
            {
                obj = root.transform.Find(this.roboname + "/" + subParts).gameObject;
                this.button_up = obj.GetComponentInChildren<IRobotTouchSensor>();
                this.button_up.Initialize(obj);
            }
            subParts = parts.getButtonSensor(ButtonSensorType.BUTTON_SENSOR_DOWN);
            if (subParts != null)
            {
                obj = root.transform.Find(this.roboname + "/" + subParts).gameObject;
                this.button_down = obj.GetComponentInChildren<IRobotTouchSensor>();
                this.button_down.Initialize(obj);
            }
            subParts = parts.getButtonSensor(ButtonSensorType.BUTTON_SENSOR_ENTER);
            if (subParts != null)
            {
                obj = root.transform.Find(this.roboname + "/" + subParts).gameObject;
                this.button_enter = obj.GetComponentInChildren<IRobotTouchSensor>();
                this.button_enter.Initialize(obj);
            }
            subParts = parts.getButtonSensor(ButtonSensorType.BUTTON_SENSOR_BACK);
            if (subParts != null)
            {
                obj = root.transform.Find(this.roboname + "/" + subParts).gameObject;
                this.button_back = obj.GetComponentInChildren<IRobotTouchSensor>();
                this.button_back.Initialize(obj);
            }
        }

        internal void UpdateSensorValues(IPduWriter pdu_writer)
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
