using Hakoniwa.PluggableAsset.Assets.Environment;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Environment
{
    public class HakoEnvObstacle : MonoBehaviour, IHakoEnvObstacle
    {
        private GameObject obstacle;
        public bool isTouched = false;

        public void Initialize(object root)
        {
            obstacle = (GameObject)root;
            this.isTouched = false;
        }

        public bool IsTouched()
        {
            return isTouched;
        }

        void OnTriggerEnter(Collider t)
        {
            this.isTouched = true;
            //Debug.Log("ENTER:" + t.gameObject.name);
        }
        void OnTriggerStay(Collider t)
        {
            this.isTouched = true;
            //Debug.Log("STAY:" + t.gameObject.name);
        }

        private void OnTriggerExit(Collider t)
        {
            this.isTouched = false;
            //Debug.Log("EXIT:" + t.gameObject.name);
        }

        public void UpdateSensorValues(Pdu pdu)
        {
            throw new NotImplementedException();
        }

        public string GetAssetName()
        {
            return this.gameObject.name;
        }

        public string topic_type = "rule_msgs/HakoRuleObstacle";
        public int update_cycle = 10;
        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[1];
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
