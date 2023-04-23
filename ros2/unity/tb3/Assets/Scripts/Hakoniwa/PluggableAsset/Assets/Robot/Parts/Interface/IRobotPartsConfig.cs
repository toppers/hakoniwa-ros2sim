using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public interface IRobotPartsConfig
    {
        RoboPartsConfigData[] GetRoboPartsConfig();
    }
    public enum IoMethod
    {
        SHM,
        RPC
    }
    public enum IoDir
    {
        READ,
        WRITE
    }
    public enum CommMethod
    {
        UDP,
        MQTT,
        DIRECT
    }
    public class RoboPartsConfigData
    {
        public IoMethod io_method = IoMethod.RPC;
        public IoDir io_dir = IoDir.WRITE;
        public RobotPartsConfig value = new RobotPartsConfig();
    }
    [System.Serializable]
    public class RobotPartsConfig
    {
        public string type = null;
        public string org_name = null;
        public string name = null;
        public string class_name = null;
        public string conv_class_name = null;
        public int channel_id = 0;
        public int pdu_size = 0;
        public int write_cycle = 1;
        public string method_type = "UDP";
    }
    [System.Serializable]
    public class RobotPartsConfigContainer
    {
        public string name = "micon_setting";
        public RobotPartsConfig[] rpc_pdu_readers = null;
        public RobotPartsConfig[] rpc_pdu_writers = null;
        public RobotPartsConfig[] shm_pdu_readers = null;
        public RobotPartsConfig[] shm_pdu_writers = null;
    }
    static class ConstantValues
    {
        public static readonly string pdu_reader_class = "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduReader";
        public static readonly string pdu_writer_class = "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriter";
        public static readonly string conv_pdu_reader_class = "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduReaderConverter";
        public static readonly string conv_pdu_writer_class = "Hakoniwa.PluggableAsset.Communication.Pdu.Raw.RawPduWriterConverter";
        public static readonly int Twist_pdu_size = 48;
        public static readonly int JointState_pdu_size = 440;
        public static readonly int Imu_pdu_size = 432;
        public static readonly int Odometry_pdu_size = 944;
        public static readonly int TFMessage_pdu_size = 320;
        public static readonly int Image_pdu_size = 1229080;
        public static readonly int CompressedImage_pdu_size = 1229064;
        public static readonly int CameraInfo_pdu_size = 580;
        public static readonly int LaserScan_pdu_size = 3044;
        public static readonly int Bool_pdu_size = 4;
    }
}
