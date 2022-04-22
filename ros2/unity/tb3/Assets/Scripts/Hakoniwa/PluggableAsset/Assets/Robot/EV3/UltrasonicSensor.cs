using Hakoniwa.PluggableAsset.Assets;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class UltrasonicSensor : MonoBehaviour, IRobotUltraSonicSensor
    {
        public static bool is_debug = true;
        private float contact_distance = 250f; /* cm */
        public float distanceValue; /* cm */
        private GameObject frontSensor;
        private Quaternion init_angle;
        private ParamScale scale;

        public void Initialize(System.Object root)
        {
            if (this.frontSensor != null)
            {
                this.distanceValue = this.contact_distance;
                return;
            }
            this.scale = AssetConfigLoader.GetScale();
            frontSensor = (GameObject)root;
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
            Vector3 fwd = frontSensor.transform.TransformDirection(Vector3.forward);
            RaycastHit hit;
            if (Physics.Raycast(transform.position, fwd, out hit, contact_distance))
            {
                if (is_debug)
                {
                    //Debug.Log("deg=" + degree + " dist=" + hit.distance);
                    Debug.DrawRay(this.frontSensor.transform.position, fwd * hit.distance, color, 0.1f, false);
                }
                return hit.distance;

            }
            else
            {
                return contact_distance;
            }
        }
        public void UpdateSensorValues()
        {
            int i = 0;
            float value;
            float min = 10000.0f;
            this.frontSensor.transform.localRotation = this.init_angle;
            for (int deg = 0; deg <= 3; deg++)
            {
                value = GetSensorValue(i, Color.green);
                if (value < min)
                {
                    min = value;
                }
                this.frontSensor.transform.Rotate(0, -5, 0);
                i++;
            }
            this.frontSensor.transform.localRotation = this.init_angle;
            for (int deg = 0; deg < 3; deg++)
            {
                this.frontSensor.transform.Rotate(0, 5, 0);
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
            throw new System.NotImplementedException();
        }
    }
}

