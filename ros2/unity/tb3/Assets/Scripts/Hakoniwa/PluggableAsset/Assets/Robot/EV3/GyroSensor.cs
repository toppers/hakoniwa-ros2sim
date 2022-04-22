using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Globalization;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class GyroSensor : MonoBehaviour, IRobotGyroSensor
    {
        private GameObject obj;
        private Vector3 baseRotation;
        private Vector3 prevRotation; 
        private float deg_rate;
        private float deltaTime;
        private bool hasResetEvent;

        public void Initialize(System.Object root)
        {
            if (this.obj != null)
            {
                return;
            }
            this.obj = (GameObject)root;
            this.baseRotation = this.obj.transform.eulerAngles;
            this.prevRotation = this.baseRotation;
            this.deg_rate = 0.0f;
            this.deltaTime = Time.deltaTime;
            this.hasResetEvent = false;
        }

        public void ClearDegree()
        {
            this.deg_rate = 0.0f;
            this.hasResetEvent = true;
        }

        public float GetDegree()
        {

            //Debug.LogFormat("current : {0}, base :  {1}", this.obj.transform.eulerAngles.y, this.baseRotation.y);
            float diff = this.obj.transform.eulerAngles.y - this.baseRotation.y;

            if (diff > 180) {
                diff -= 360f;
            }

            if (diff < -180) {
                diff += 360f;
            }

            return diff;

            /*
            if (diff <= 180.0f)
            {
                return (diff);
            }
            else
            {
                return (diff - 360.0f);
            }
            */
        }

        public float GetDegreeRate()
        {
            return this.deg_rate;
        }

        public void UpdateSensorValues()
        {
            if (this.hasResetEvent)
            {
                this.baseRotation = this.obj.transform.eulerAngles;
                this.hasResetEvent = false;
            }
            float diff = this.obj.transform.eulerAngles.y - this.prevRotation.y;
            this.deg_rate = diff / this.deltaTime;
            this.prevRotation = this.obj.transform.eulerAngles;
            //Debug.Log("deg=" + (int)this.GetDegree() + ":deg_rate=" + (int)this.deg_rate);
        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            throw new System.NotImplementedException();
        }
    }
}