using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Globalization;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using System;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class GyroSensor : MonoBehaviour, IRobotPartsSensor
    {
        private string root_name;
        private IPduWriter pdu_writer;
        private PduIoConnector pdu_io;

        private GameObject root;
        private Vector3 baseRotation;
        private Vector3 prevRotation; 
        private float deg_rate;
        private float deltaTime;
        private bool hasResetEvent;

        public void Initialize(GameObject root)
        {
            if (this.root != null)
            {
                this.ClearDegree();
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

            this.baseRotation = this.transform.eulerAngles;
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
            float diff = this.transform.eulerAngles.y - this.baseRotation.y;

            if (diff > 180) {
                diff -= 360f;
            }

            if (diff < -180) {
                diff += 360f;
            }

            return diff;

        }

        public float GetDegreeRate()
        {
            return this.deg_rate;
        }

        private void UpdateSensorValuesLocal()
        {
            if (this.hasResetEvent)
            {
                this.baseRotation = this.transform.eulerAngles;
                this.hasResetEvent = false;
            }
            float diff = this.transform.eulerAngles.y - this.prevRotation.y;
            this.deg_rate = diff / this.deltaTime;
            this.prevRotation = this.transform.eulerAngles;
            //Debug.Log("deg=" + (int)this.GetDegree() + ":deg_rate=" + (int)this.deg_rate);
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
            this.pdu_writer.GetWriteOps().SetData("gyro_degree", (int)this.GetDegree());
            this.pdu_writer.GetWriteOps().SetData("gyro_degree_rate", (int)this.GetDegreeRate());
        }
    }
}