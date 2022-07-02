using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{

    public class GpsSensor : MonoBehaviour, IRobotPartsSensor
    {
        private string root_name;
        private IPduWriter pdu_writer;
        private PduIoConnector pdu_io;

        private GameObject root;
        private Vector3 pos;

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
            return;
        }
        public double GeLatitude()
        {
            return this.pos.x;
        }

        public double GetLongitude()
        {
            return this.pos.z;
        }

        public int GetStatus()
        {
            return 0;
        }


        public void UpdateSensorValuesLocal()
        {
            pos = new Vector3(this.transform.position.x, this.transform.position.y, this.transform.position.z);
            //Debug.Log("x=" + pos.x + " y=" + pos.y);
            return;
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
            this.pdu_writer.GetWriteOps().SetData("gps_lon", this.GetLongitude());
            this.pdu_writer.GetWriteOps().SetData("gps_lat", this.GeLatitude());

        }
    }
}