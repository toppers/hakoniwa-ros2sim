using System.Collections;
using System.Collections.Generic;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Newtonsoft.Json;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    [System.Serializable]
    public class Ev3MiconConfigUdpProxy
    {
        public string name = "Athrill";
        public string class_name = "Hakoniwa.PluggableAsset.Assets.Micon.EV3.Ev3MiconAssetController";
        public string ipaddr = "127.0.0.1";
        public int tx_port = 54001;
        public int rx_port = 54002;
    }
    [System.Serializable]
    public class Ev3MiconConfigUdpPduReader
    {
        public string type = "ev3_msgs/Ev3PduActuator";
        public string org_name = "ev3_actuator";
        public string name = null;
        public string class_name = "Hakoniwa.PluggableAsset.Communication.Pdu.Ev3.Ev3PduReader";
        public string class_path = null;
        public string conv_class_name = "Hakoniwa.PluggableAsset.Communication.Pdu.Ev3.Ev3PduReaderProtobufConverter";
        public string conv_class_path = null;
    }
    [System.Serializable]
    public class Ev3MiconConfigUdpPduWriter
    {
        public string type = "ev3_msgs/Ev3PduSensor";
        public string org_name = "ev3_sensor";
        public string name = null;
        public string class_name = "Hakoniwa.PluggableAsset.Communication.Pdu.Ev3.Ev3PduWriter";
        public string class_path = null;
        public string conv_class_name = "Hakoniwa.PluggableAsset.Communication.Pdu.Ev3.Ev3PduWriterProtobufConverter";
        public string conv_class_path = null;
    }
    [System.Serializable]
    public class Ev3MiconConfigSettingsContainer
    {
        public string name;
        public Ev3MiconConfigUdpProxy udp_proxy;
        public Ev3MiconConfigUdpPduReader [] udp_pdu_readers;
        public Ev3MiconConfigUdpPduWriter [] udp_pdu_writers;
    }
    public class Ev3MiconConfig : MonoBehaviour, IMiconSettings
    {
        public bool on = false;
        public Ev3MiconConfigSettingsContainer settings;

        public string GetSettings(string name)
        {
            this.settings.name = name;
            foreach (var e in this.settings.udp_pdu_readers)
            {
                e.name = name + "_" + e.org_name;
            }
            foreach (var e in this.settings.udp_pdu_writers)
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
