using System.Collections;
using System.Collections.Generic;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Newtonsoft.Json;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    [System.Serializable]
    public class Ev3MiconConfigShmProxy
    {
        public string name = "Athrill";
        public string class_name = "Hakoniwa.PluggableAsset.Assets.Micon.EV3.Ev3MiconAssetController";
    }
    [System.Serializable]
    public class Ev3MiconConfigShmPduReader
    {
        public string type = "ev3_msgs/Ev3PduActuator";
        public string org_name = "ev3_actuator";
        public string name = null;
        public string class_name = "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduReader";
        public string class_path = null;
        public string conv_class_name = "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduReaderConverter";
        public string conv_class_path = null;
        public int channel_id = 0;
        public int pdu_size = 196;
    }
    [System.Serializable]
    public class Ev3MiconConfigShmPduWriter
    {
        public string type = "ev3_msgs/Ev3PduSensor";
        public string org_name = "ev3_sensor";
        public string name = null;
        public string class_name = "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter";
        public string class_path = null;
        public string conv_class_name = "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter";
        public string conv_class_path = null;
        public int channel_id = 1;
        public int pdu_size = 248;
    }
    [System.Serializable]
    public class Ev3MiconShmConfigSettingsContainer
    {
        public string name = "micon_setting";
        public Ev3MiconConfigShmProxy shm_proxy;
        public Ev3MiconConfigShmPduReader[] shm_pdu_readers =
        {
            new Ev3MiconConfigShmPduReader()
        };
        public Ev3MiconConfigShmPduWriter[] shm_pdu_writers =
        {
            new Ev3MiconConfigShmPduWriter()
        };
    }
    public class Ev3ShmMiconConfig : MonoBehaviour, IMiconSettings
    {
        public bool on = false;
        public Ev3MiconShmConfigSettingsContainer settings;

        public string GetSettings(string name)
        {
            this.settings.name = name;
            foreach (var e in this.settings.shm_pdu_readers)
            {
                e.name = name + "_" + e.org_name;
            }
            foreach (var e in this.settings.shm_pdu_writers)
            {
                e.name = name + "_" + e.org_name;
            }
            return JsonConvert.SerializeObject(this.settings, Formatting.Indented);
        }

        public bool isEnabled()
        {
            return this.on;
        }
    }

}
