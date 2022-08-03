using Hakoniwa.PluggableAsset.Assets;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class UltrasonicSensor : MonoBehaviour, IRobotPartsSensor
    {
        private GameObject root;
        private string root_name;
        private IPduWriter pdu_writer;
        private PduIoConnector pdu_io;


        public static bool is_debug = true;
        private float contact_distance = 250f; /* cm */
        public float distanceValue; /* cm */
        private Quaternion init_angle;
        private ParamScale scale;

        public void Initialize(GameObject root)
        {
            if (this.root != null)
            {
                this.distanceValue = this.contact_distance;
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

            this.scale = AssetConfigLoader.GetScale();
            this.distanceValue = this.contact_distance;
            this.init_angle = this.transform.localRotation;
        }

        public float GetDistanceValue()
        {
            if ((this.distanceValue < 3) || (this.distanceValue > 249))
            {
                return 255.0f * this.scale.scan;
            }
            //Debug.Log("distance=" + this.distanceValue);
            return distanceValue * this.scale.scan; /* centimeters */
        }
        private float GetSensorValue(int inx, Color color)
        {
            Vector3 fwd = this.transform.TransformDirection(Vector3.forward);
            RaycastHit hit;
            if (Physics.Raycast(transform.position, fwd, out hit, contact_distance))
            {
                //Debug.Log("green");
                if (is_debug)
                {
                    //Debug.Log("deg=" + degree + " dist=" + hit.distance);
                    Debug.DrawRay(this.transform.position, fwd * hit.distance, color, 0.1f, false);
                }
                return hit.distance;

            }
            else
            {
                //Debug.Log("red");
                Debug.DrawRay(this.transform.position, fwd * contact_distance, Color.red, 0.1f, false);
                return contact_distance;
            }
        }
        private void UpdateSensorValuesLocal()
        {
            int i = 0;
            float value;
            float min = 10000.0f;
            this.transform.localRotation = this.init_angle;
            for (int deg = 0; deg <= 3; deg++)
            {
                value = GetSensorValue(i, Color.green);
                if (value < min)
                {
                    min = value;
                }
                this.transform.Rotate(0, -5, 0);
                i++;
            }
            this.transform.localRotation = this.init_angle;
            for (int deg = 0; deg < 3; deg++)
            {
                this.transform.Rotate(0, 5, 0);
                value = GetSensorValue(i, Color.green);
                if (value < min)
                {
                    min = value;
                }
                i++;
            }
            this.distanceValue = min;
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
            this.pdu_writer.GetWriteOps().SetData("sensor_ultrasonic", (uint)(this.GetDistanceValue() * 10));
        }
    }
}

