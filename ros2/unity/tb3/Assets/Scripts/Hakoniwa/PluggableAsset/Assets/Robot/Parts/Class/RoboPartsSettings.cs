using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    class RoboPartsSettings : MonoBehaviour, IMiconSettings
    {
        public bool on = false;
        public string GetSettings(string name)
        {
            var root = GameObject.Find("Robot");
            var myObject = GameObject.Find("Robot/" + this.transform.name);
            Debug.Log("RoboPartsSettings: transform.name=" + this.transform.name);
            var robo_parts = myObject.GetComponentsInChildren<IRobotPartsConfig>();
            if (robo_parts == null)
            {
                return null;
            }
            RobotPartsConfigContainer container = new RobotPartsConfigContainer();
            container.name = name;
            List<RobotPartsConfig> rpc_readers = new List<RobotPartsConfig>();
            List<RobotPartsConfig> rpc_writers = new List<RobotPartsConfig>();
            List<RobotPartsConfig> shm_readers = new List<RobotPartsConfig>();
            List<RobotPartsConfig> shm_writers = new List<RobotPartsConfig>();
            int channel_id = 0;
            foreach (var single_parts in robo_parts)
            {
                var configs = single_parts.GetRoboPartsConfig();
                if (configs == null)
                {
                    continue;
                }
                foreach (var config in configs)
                {
                    config.value.name = name + "_" + config.value.org_name;
                    if (config.io_method == IoMethod.RPC)
                    {
                        if (config.io_dir == IoDir.READ)
                        {
                            rpc_readers.Add(config.value);
                        }
                        else
                        {
                            rpc_writers.Add(config.value);
                        }
                    }
                    else
                    {
                        if (config.io_dir == IoDir.READ)
                        {
                            shm_readers.Add(config.value);
                        }
                        else
                        {
                            shm_writers.Add(config.value);
                        }
                    }
                    config.value.channel_id = channel_id;
                    channel_id++;
                }
            }
            container.rpc_pdu_readers = this.ConvListToArray(rpc_readers);
            container.rpc_pdu_writers = this.ConvListToArray(rpc_writers);
            container.shm_pdu_readers = this.ConvListToArray(shm_readers);
            container.shm_pdu_writers = this.ConvListToArray(shm_writers);
            return JsonConvert.SerializeObject(container, Formatting.Indented); ;
        }
        private RobotPartsConfig[] ConvListToArray(List<RobotPartsConfig> list)
        {
            RobotPartsConfig[] ret_array = new RobotPartsConfig[list.Count];
            int i = 0;
            foreach (var e in list)
            {
                ret_array[i] = e;
                i++;
            }
            return ret_array;
        }

        public bool isEnabled()
        {
            return this.on;
        }
    }
}
