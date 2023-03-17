using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.TB3
{
    [System.Serializable]
    public class TB3MiconConfigPduReader
    {
        public string type = null;
        public string org_name = null;
        public string name = null;
        public string class_name = null;
        public string class_path = null;
        public string conv_class_name = null;
        public string conv_class_path = null;
        public int channel_id = 0;
        public int pdu_size = 1024;
        public string method_type = "UDP";
        public TB3MiconConfigPduReader(string type, string org_name, string class_name, string conv_class_name, int channel_id, int pdu_size)
        {
            this.type = type;
            this.org_name = org_name;
            this.class_name = class_name;
            this.conv_class_name = conv_class_name;
            this.channel_id = channel_id;
            this.pdu_size = pdu_size;
        }
    }
    [System.Serializable]
    public class TB3MiconConfigPduWriter
    {
        public string type = null;
        public string org_name = null;
        public string name = null;
        public string class_name = null;
        public string class_path = null;
        public string conv_class_name = null;
        public string conv_class_path = null;
        public int channel_id = 0;
        public int pdu_size = 1024;
        public int write_cycle = 1;
        public string method_type = "UDP";
        public TB3MiconConfigPduWriter(string type, string org_name, string class_name, string conv_class_name, int channel_id, int pdu_size)
        {
            this.type = type;
            this.org_name = org_name;
            this.class_name = class_name;
            this.conv_class_name = conv_class_name;
            this.channel_id = channel_id;
            this.pdu_size = pdu_size;
        }
    }

    [System.Serializable]
    public class TB3MiconConfigSettingsContainer
    {
        public string name = "micon_setting";
        public TB3MiconConfigPduReader[] rpc_pdu_readers =
        {
            new TB3MiconConfigPduReader(
                "geometry_msgs/Twist",
                "cmd_vel",
                 "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduReader",
                  "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduReaderConverter",
                  0,
                  48
                ),
        };
        public TB3MiconConfigPduWriter[] rpc_pdu_writers =
        {
            new TB3MiconConfigPduWriter(
                "sensor_msgs/JointState",
                "joint_states",
                 "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter",
                  "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter",
                  1,
                  440
                ),
            new TB3MiconConfigPduWriter(
                "sensor_msgs/Imu",
                "imu",
                 "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter",
                  "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter",
                  2,
                  432
                ),
            new TB3MiconConfigPduWriter(
                "nav_msgs/Odometry",
                "odom",
                 "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter",
                  "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter",
                  3,
                  944
                ),
            new TB3MiconConfigPduWriter(
                "tf2_msgs/TFMessage",
                "tf",
                 "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter",
                  "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter",
                  4,
                  320
                ),
            new TB3MiconConfigPduWriter(
                "sensor_msgs/Image",
                "image",
                 "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter",
                  "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter",
                  5,
                  1229080
                ),
            new TB3MiconConfigPduWriter(
                "sensor_msgs/CompressedImage",
                "image/compressed",
                 "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter",
                  "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter",
                  6,
                  1229064
                ),
            new TB3MiconConfigPduWriter(
                "sensor_msgs/CameraInfo",
                "camera_info",
                 "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter",
                  "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter",
                  7,
                  580
                ),
            new TB3MiconConfigPduWriter(
                "sensor_msgs/LaserScan",
                "scan",
                 "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter",
                  "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter",
                  8,
                  1800
                ),
        };
    }

    public class TB3MiconConfig : MonoBehaviour, IMiconSettings
    {
        public bool on = false;
        public TB3MiconConfigSettingsContainer settings;

        public string GetSettings(string name)
        {
            this.settings.name = name;
            foreach (var e in this.settings.rpc_pdu_readers)
            {
                e.name = name + "_" + e.org_name;
            }
            foreach (var e in this.settings.rpc_pdu_writers)
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
