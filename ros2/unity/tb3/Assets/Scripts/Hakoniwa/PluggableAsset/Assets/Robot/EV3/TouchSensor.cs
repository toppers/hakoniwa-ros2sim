using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class TouchSensor : MonoBehaviour, IRobotPartsTouchSensor, IPointerDownHandler, IPointerUpHandler
    {
        private string root_name;
        private IPduWriter pdu_writer;
        private PduIoConnector pdu_io;

        private GameObject root;
        public int touchSensorNo = 0;
        public bool hasParent = true;
        public bool isTouched;

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

            this.isTouched = false;
        }
        public bool IsPressed()
        {
            return this.isTouched;
        }

        private void OnTriggerStay(Collider other)
        {
            this.isTouched = true;
            //Debug.Log("Pressed");
        }
        private void OnTriggerExit(Collider other)
        {
            this.isTouched = false;
            //Debug.Log("NotPressed");
        }


        void IPointerDownHandler.OnPointerDown(PointerEventData eventData)
        {
            this.isTouched = true;
            //Debug.Log("Pressed");
        }


        void IPointerUpHandler.OnPointerUp(PointerEventData eventData)
        {
            this.isTouched = false;
            //Debug.Log("NotPressed");
        }

        public bool isAttachedSpecificController()
        {
            return hasParent;
        }

        public void UpdateSensorValues()
        {
            if (hasParent)
            {
                return;
            }
            if (this.IsPressed())
            {
                //Debug.Log("Touched1:");
                this.pdu_writer.GetWriteOps().Refs("touch_sensors")[this.touchSensorNo].SetData("value", (uint)4095);
            }
            else
            {
                this.pdu_writer.GetWriteOps().Refs("touch_sensors")[this.touchSensorNo].SetData("value", (uint)0);
            }
        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            return null;
        }
    }
}
