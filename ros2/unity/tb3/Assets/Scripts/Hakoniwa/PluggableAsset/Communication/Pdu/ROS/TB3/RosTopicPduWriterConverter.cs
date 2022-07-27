using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS.TB3;

using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.ROS.TB3
{
    public class RosTopicPduWriterConverter : IPduWriterConverter
    {
        public IPduCommData ConvertToIoData(IPduWriter src)
        {
            RosTopicPduWriter pdu_writer = src as RosTopicPduWriter;
            return new RosTopicPduCommTypedData(pdu_writer);
        }
        

        static private void ConvertToMessage(IPduReadOperation src, CameraInfoMsg dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
			dst.height = src.GetDataUInt32("height");
			dst.width = src.GetDataUInt32("width");
			dst.distortion_model = src.GetDataString("distortion_model");
			dst.d = src.GetDataFloat64Array("d");
			dst.k = src.GetDataFloat64Array("k");
			dst.r = src.GetDataFloat64Array("r");
			dst.p = src.GetDataFloat64Array("p");
			dst.binning_x = src.GetDataUInt32("binning_x");
			dst.binning_y = src.GetDataUInt32("binning_y");
            ConvertToMessage(src.Ref("roi").GetPduReadOps(), dst.roi);
        }
        static private void ConvertToMessage(IPduReadOperation src, CompressedImageMsg dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
			dst.format = src.GetDataString("format");
			dst.data = src.GetDataUInt8Array("data");
        }
        static private void ConvertToMessage(IPduReadOperation src, HeaderMsg dst)
        {
            ConvertToMessage(src.Ref("stamp").GetPduReadOps(), dst.stamp);
			dst.frame_id = src.GetDataString("frame_id");
        }
        static private void ConvertToMessage(IPduReadOperation src, ImageMsg dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
			dst.height = src.GetDataUInt32("height");
			dst.width = src.GetDataUInt32("width");
			dst.encoding = src.GetDataString("encoding");
			dst.is_bigendian = src.GetDataUInt8("is_bigendian");
			dst.step = src.GetDataUInt32("step");
			dst.data = src.GetDataUInt8Array("data");
        }
        static private void ConvertToMessage(IPduReadOperation src, ImuMsg dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
            ConvertToMessage(src.Ref("orientation").GetPduReadOps(), dst.orientation);
			dst.orientation_covariance = src.GetDataFloat64Array("orientation_covariance");
            ConvertToMessage(src.Ref("angular_velocity").GetPduReadOps(), dst.angular_velocity);
			dst.angular_velocity_covariance = src.GetDataFloat64Array("angular_velocity_covariance");
            ConvertToMessage(src.Ref("linear_acceleration").GetPduReadOps(), dst.linear_acceleration);
			dst.linear_acceleration_covariance = src.GetDataFloat64Array("linear_acceleration_covariance");
        }
        static private void ConvertToMessage(IPduReadOperation src, JointStateMsg dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
			dst.name = src.GetDataStringArray("name");
			dst.position = src.GetDataFloat64Array("position");
			dst.velocity = src.GetDataFloat64Array("velocity");
			dst.effort = src.GetDataFloat64Array("effort");
        }
        static private void ConvertToMessage(IPduReadOperation src, LaserScanMsg dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
			dst.angle_min = src.GetDataFloat32("angle_min");
			dst.angle_max = src.GetDataFloat32("angle_max");
			dst.angle_increment = src.GetDataFloat32("angle_increment");
			dst.time_increment = src.GetDataFloat32("time_increment");
			dst.scan_time = src.GetDataFloat32("scan_time");
			dst.range_min = src.GetDataFloat32("range_min");
			dst.range_max = src.GetDataFloat32("range_max");
			dst.ranges = src.GetDataFloat32Array("ranges");
			dst.intensities = src.GetDataFloat32Array("intensities");
        }
        static private void ConvertToMessage(IPduReadOperation src, OdometryMsg dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
			dst.child_frame_id = src.GetDataString("child_frame_id");
            ConvertToMessage(src.Ref("pose").GetPduReadOps(), dst.pose);
            ConvertToMessage(src.Ref("twist").GetPduReadOps(), dst.twist);
        }
        static private void ConvertToMessage(IPduReadOperation src, PointMsg dst)
        {
			dst.x = src.GetDataFloat64("x");
			dst.y = src.GetDataFloat64("y");
			dst.z = src.GetDataFloat64("z");
        }
        static private void ConvertToMessage(IPduReadOperation src, PoseMsg dst)
        {
            ConvertToMessage(src.Ref("position").GetPduReadOps(), dst.position);
            ConvertToMessage(src.Ref("orientation").GetPduReadOps(), dst.orientation);
        }
        static private void ConvertToMessage(IPduReadOperation src, PoseWithCovarianceMsg dst)
        {
            ConvertToMessage(src.Ref("pose").GetPduReadOps(), dst.pose);
			dst.covariance = src.GetDataFloat64Array("covariance");
        }
        static private void ConvertToMessage(IPduReadOperation src, QuaternionMsg dst)
        {
			dst.x = src.GetDataFloat64("x");
			dst.y = src.GetDataFloat64("y");
			dst.z = src.GetDataFloat64("z");
			dst.w = src.GetDataFloat64("w");
        }
        static private void ConvertToMessage(IPduReadOperation src, RegionOfInterestMsg dst)
        {
			dst.x_offset = src.GetDataUInt32("x_offset");
			dst.y_offset = src.GetDataUInt32("y_offset");
			dst.height = src.GetDataUInt32("height");
			dst.width = src.GetDataUInt32("width");
			dst.do_rectify = src.GetDataBool("do_rectify");
        }
        static private void ConvertToMessage(IPduReadOperation src, TFMessageMsg dst)
        {
            if (dst.transforms.Length < src.Refs("transforms").Length)
            {
                dst.transforms = new TransformStampedMsg[src.Refs("transforms").Length];
            }
            foreach (var e in src.Refs("transforms"))
            {
                int index = Array.IndexOf(src.Refs("transforms"), e);
                if (dst.transforms[index] == null) {
                    dst.transforms[index] = new TransformStampedMsg();
                }
                ConvertToMessage(e.GetPduReadOps(), dst.transforms[index]);
            }
        }
        static private void ConvertToMessage(IPduReadOperation src, TimeMsg dst)
        {
			dst.sec = src.GetDataInt32("sec");
			dst.nanosec = src.GetDataUInt32("nanosec");
        }
        static private void ConvertToMessage(IPduReadOperation src, TransformMsg dst)
        {
            ConvertToMessage(src.Ref("translation").GetPduReadOps(), dst.translation);
            ConvertToMessage(src.Ref("rotation").GetPduReadOps(), dst.rotation);
        }
        static private void ConvertToMessage(IPduReadOperation src, TransformStampedMsg dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
			dst.child_frame_id = src.GetDataString("child_frame_id");
            ConvertToMessage(src.Ref("transform").GetPduReadOps(), dst.transform);
        }
        static private void ConvertToMessage(IPduReadOperation src, TwistMsg dst)
        {
            ConvertToMessage(src.Ref("linear").GetPduReadOps(), dst.linear);
            ConvertToMessage(src.Ref("angular").GetPduReadOps(), dst.angular);
        }
        static private void ConvertToMessage(IPduReadOperation src, TwistWithCovarianceMsg dst)
        {
            ConvertToMessage(src.Ref("twist").GetPduReadOps(), dst.twist);
			dst.covariance = src.GetDataFloat64Array("covariance");
        }
        static private void ConvertToMessage(IPduReadOperation src, Vector3Msg dst)
        {
			dst.x = src.GetDataFloat64("x");
			dst.y = src.GetDataFloat64("y");
			dst.z = src.GetDataFloat64("z");
        }
        
        
        static public Message ConvertToMessage(IPduReadOperation src, string type)
        {

            if (type.Equals("sensor_msgs/Imu"))
            {
            	ImuMsg ros_topic = new ImuMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("nav_msgs/Odometry"))
            {
            	OdometryMsg ros_topic = new OdometryMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("tf2_msgs/TFMessage"))
            {
            	TFMessageMsg ros_topic = new TFMessageMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("sensor_msgs/JointState"))
            {
            	JointStateMsg ros_topic = new JointStateMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("geometry_msgs/Twist"))
            {
            	TwistMsg ros_topic = new TwistMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("sensor_msgs/Image"))
            {
            	ImageMsg ros_topic = new ImageMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("sensor_msgs/CompressedImage"))
            {
            	CompressedImageMsg ros_topic = new CompressedImageMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("sensor_msgs/CameraInfo"))
            {
            	CameraInfoMsg ros_topic = new CameraInfoMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("sensor_msgs/LaserScan"))
            {
            	LaserScanMsg ros_topic = new LaserScanMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            throw new InvalidCastException("Can not find ros message type:" + type);
        }
        
        static public Message ConvertToMessage(RosTopicPduWriter pdu_writer)
        {
            return ConvertToMessage(pdu_writer.GetReadOps(), pdu_writer.GetTypeName());
        }
    }

}
