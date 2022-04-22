using Hakoniwa.Core.Simulation;
using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets;
using Hakoniwa.PluggableAsset.Assets.Environment;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts.Hakoniwa.PluggableAsset.Assets.Environment
{
    class HakoEnv : MonoBehaviour, IInsideAssetController, IHakoEnv
    {
        private GameObject root;
        private IHakoEnvObstacle[] obstacles;
        private IHakoEnvCamera[] cameras;
        private IPduWriter pdu_hakoenv;
        private IPduWriter[] pdu_obstacles;
        private IPduWriter[] pdu_cameras;
        private PduIoConnector pdu_io;
        private string my_name = "HakoEnv";
        private bool[] is_touch;

        private void InitializeObstacleMonitors()
        {
            var tmp = this.root.GetComponentsInChildren<IHakoEnvObstacle>();
            obstacles = new IHakoEnvObstacle[tmp.Length];
            pdu_obstacles = new IPduWriter[tmp.Length];
            int i = 0;
            foreach (var e in tmp)
            {
                this.obstacles[i] = e;
                var pdu_name = my_name + "_" + e.GetAssetName() + "Pdu";
                this.pdu_obstacles[i] = this.pdu_io.GetWriter(pdu_name);
                if (this.pdu_obstacles[i] == null)
                {
                    throw new ArgumentException("can not found HakoEnv pdu:" + pdu_name);
                }
                Debug.Log("Obstacle:" + e);
                i++;
            }
            this.is_touch = new bool[tmp.Length];
            this.UpdateObstacleSensorValues();
        }
        private void InitializeCameras()
        {
            var tmp = this.root.GetComponentsInChildren<IHakoEnvCamera>();
            if (tmp == null)
            {
                return;
            }
            Debug.Log("camera num=" + tmp.Length);
            cameras = new IHakoEnvCamera[tmp.Length];
            pdu_cameras = new IPduWriter[tmp.Length];

            int i = 0;
            foreach (var child in tmp)
            {
                cameras[i] = child;
                Debug.Log("child=" + child.ToString());
                cameras[i].Initialize(child);
                string pdu_name = my_name + "_" + cameras[i].GetAssetName() + "Pdu";
                this.pdu_cameras[i] = this.pdu_io.GetWriter(pdu_name);
                if (this.pdu_cameras[i] == null)
                {
                    throw new ArgumentException("can not found HakoEnvCamera pdu:" + pdu_name);
                }
                Debug.Log("Camera:pdu=" + pdu_name);
                i++;
            }
        }
        private void UpdateObstacleSensorValues()
        {
            int i = 0;
            foreach (var e in obstacles)
            {
                this.is_touch[i] = e.IsTouched();
                bool[] output_topic = new bool[1];
                output_topic[0] = this.is_touch[i];
                this.pdu_obstacles[i].GetWriteOps().SetData("is_touch", output_topic);
                i++;
                //Debug.Log("is_touch:" + this.is_touch[i]);
            }
        }
        private void UpdateCameraSensorValues()
        {
            int i = 0;
            foreach (var e in cameras)
            {
                e.UpdateSensorValues(this.pdu_cameras[i].GetReadOps().Ref(null));
                i++;
            }

        }

        public void Initialize()
        {
            this.root = GameObject.Find("HakoEnv");
            Debug.Log("HakoEnv Enter");
            this.pdu_io = PduIoConnector.Get(my_name);
            this.pdu_hakoenv = this.pdu_io.GetWriter(my_name + "_HakoEnvPdu");
            this.InitializeObstacleMonitors();
            this.InitializeCameras();

        }
        public void CopySensingDataToPdu()
        {
            this.UpdateObstacleSensorValues();
            this.UpdateCameraSensorValues();
            if (this.pdu_hakoenv != null)
            {
                this.pdu_hakoenv.GetWriteOps().SetData("simtime", (UInt64)SimulationController.Get().GetWorldTime());
            }
        }

        public void DoActuation()
        {
            /* nothing to do */
        }

        public string GetName()
        {
            return "HakoEnv";
        }

        public void Initialize(object root)
        {
            throw new NotImplementedException();
        }

        public string GetAssetName()
        {
            return this.gameObject.name;
        }

        public void UpdateSensorValues(Pdu pdu)
        {
            throw new NotImplementedException();
        }

        public string topic_type = "rule_msgs/HakoEnv";
        public int update_cycle = 10;
        public RosTopicMessageConfig [] getRosConfig()
        {
            RosTopicMessageConfig [] cfg = new RosTopicMessageConfig[1];
            cfg[0] = new RosTopicMessageConfig();
            cfg[0].topic_type_name = this.topic_type;
            cfg[0].sub = false;
            cfg[0].pub_option = new RostopicPublisherOption();
            cfg[0].pub_option.cycle_scale = this.update_cycle;
            cfg[0].pub_option.latch = false;
            cfg[0].pub_option.queue_size = 1;
            return cfg;
        }
    }
}
