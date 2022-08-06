using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

using Hakoniwa.Core;
using Hakoniwa.PluggableAsset.Assets;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class RosTopicInfo : MonoBehaviour, IRobotParts
    {
        public string[] topic_type = {
            "ev3_msgs/Ev3PduSensor",
            "ev3_msgs/Ev3PduActuator"
        };
        public string[] topic_name = {
            "ev3_sensor",
            "ev3_actuator"
        };

        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[topic_type.Length];
            int i = 0;
            for (i = 0; i < topic_type.Length; i++)
            {
                cfg[i] = new RosTopicMessageConfig();
                cfg[i].topic_message_name = this.topic_name[i];
                cfg[i].topic_type_name = this.topic_type[i];
                if (cfg[i].topic_message_name == "ev3_actuator")
                {
                    cfg[i].sub = true;
                }
                else
                {
                    cfg[i].sub = false;
                }
                cfg[i].pub_option = new RostopicPublisherOption();
                cfg[i].pub_option.cycle_scale = 1;
                cfg[i].pub_option.latch = false;
                cfg[i].pub_option.queue_size = 1;
            }
            Debug.Log("cfg=" + cfg.Length);
            return cfg;
        }

        public void Initialize(GameObject root)
        {
            //nothing to do
            return;
        }
    }

}