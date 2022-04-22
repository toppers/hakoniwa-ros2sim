using Hakoniwa.PluggableAsset.Assets.Robot;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public class LaserScanner : MonoBehaviour, IRobotPartsSensor
    {
        private GameObject root;
        private GameObject sensor;
        private string root_name;
        private PduIoConnector pdu_io;
        private IPduWriter pdu_writer;
        public float scale  = 1.0f;

        private Quaternion init_angle;
        public static bool is_debug = true;
        private float contact_distance = 350f; /* cm */
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
                var pdu_writer_name = root_name + "_" + this.topic_name + "Pdu";
                this.pdu_writer = this.pdu_io.GetWriter(pdu_writer_name);
                if (this.pdu_writer == null)
                {
                    throw new ArgumentException("can not found pdu_reader:" + pdu_writer_name);
                }
                this.sensor = this.gameObject;
                this.init_angle = this.sensor.transform.localRotation;
                this.distances = new float[max_count];

            }
        }

        public void UpdateSensorValues()
        {
            this.count++;
            if (this.count < this.update_cycle)
            {
                return;
            }
            this.count = 0;
            this.Scan();
            this.UpdatePdu(this.pdu_writer.GetWriteOps().Ref(null));
            return;
        }
        public int max_count = 360;
        private float[] distances;
        private float angle_min = 0.0f;
        private float angle_max = 6.26573181152f; //© 6.265 rad = 359‹
        private float range_min = 0.119999997318f; //© 12cm
        private float range_max = 3.5f; //© 3.5m
        private float angle_increment = 0.0174532923847f; //© 0.01745 rad = 1‹
        private float time_increment = 2.98800005112e-05f;
        private float scan_time = 0.0f;
        private float[] intensities = new float[0];
        public void UpdatePdu(Pdu pdu)
        {
            TimeStamp.Set(pdu);
            pdu.Ref("header").SetData("frame_id", "base_scan");

            pdu.SetData("angle_min", angle_min);
            pdu.SetData("angle_max", angle_max);
            pdu.SetData("range_min", range_min);
            pdu.SetData("range_max", range_max);
            pdu.SetData("ranges", distances);
            pdu.SetData("angle_increment", angle_increment);
            pdu.SetData("time_increment", time_increment);
            pdu.SetData("scan_time", scan_time);
            pdu.SetData("intensities", intensities);
        }


        private void Scan()
        {
            this.sensor.transform.localRotation = this.init_angle;
            for (int i = 0; i < max_count; i++)
            {
                distances[max_count - i - 1] = (GetSensorValue(i) * this.scale) / 100.0f;
                this.sensor.transform.Rotate(0, 1, 0);
                //Debug.Log("angle=" + this.sensor.transform.localEulerAngles.y);
            }
        }
        public int view_interval = 5;
        private float GetSensorValue(int degree)
        {
            Vector3 fwd = this.sensor.transform.forward;
            RaycastHit hit;
            if (Physics.Raycast(sensor.transform.position, fwd, out hit, contact_distance))
            {
                if (is_debug && (degree % view_interval) == 0)
                {
                    Debug.DrawRay(this.sensor.transform.position, fwd * hit.distance, Color.red, 0.05f, false);
                }
                return hit.distance;
            }
            else
            {
                if (is_debug && (degree % view_interval) == 0) {
                    Debug.DrawRay(this.sensor.transform.position, fwd * contact_distance, Color.green, 0.05f, false);
                }
                return contact_distance;
            }
        }
        public string topic_type = "sensor_msgs/LaserScan";
        public string topic_name = "scan";
        public int update_cycle = 10;
        private int count = 0;
        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[1];
            cfg[0] = new RosTopicMessageConfig();
            cfg[0].topic_message_name = this.topic_name;
            cfg[0].topic_type_name = this.topic_type;
            cfg[0].sub = false;
            cfg[0].pub_option = new RostopicPublisherOption();
            cfg[0].pub_option.cycle_scale = this.update_cycle;
            cfg[0].pub_option.latch = false;
            cfg[0].pub_option.queue_size = 1;
            return cfg;
        }

        public bool isAttachedSpecificController()
        {
            return false;
        }
    }

}
