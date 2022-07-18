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
    public class RosTopicPduReaderConverter : IPduReaderConverter
    {
        public IPduCommData ConvertToIoData(IPduReader src)
        {
            RosTopicPduReader pdu_reader = src as RosTopicPduReader;
            return new RosTopicPduCommTypedData(pdu_reader);
        }
        

        private void ConvertToPdu(CameraInfoMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
            dst.SetData("height", src.height);
            dst.SetData("width", src.width);
            dst.SetData("distortion_model", src.distortion_model);
            dst.SetData("d", src.d);
            dst.SetData("k", src.k);
            dst.SetData("r", src.r);
            dst.SetData("p", src.p);
            dst.SetData("binning_x", src.binning_x);
            dst.SetData("binning_y", src.binning_y);
			ConvertToPdu(src.roi, dst.Ref("roi").GetPduWriteOps());
        }
        private void ConvertToPdu(CompressedImageMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
            dst.SetData("format", src.format);
            dst.SetData("data", src.data);
        }
        private void ConvertToPdu(HeaderMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.stamp, dst.Ref("stamp").GetPduWriteOps());
            dst.SetData("frame_id", src.frame_id);
        }
        private void ConvertToPdu(ImageMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
            dst.SetData("height", src.height);
            dst.SetData("width", src.width);
            dst.SetData("encoding", src.encoding);
            dst.SetData("is_bigendian", src.is_bigendian);
            dst.SetData("step", src.step);
            dst.SetData("data", src.data);
        }
        private void ConvertToPdu(ImuMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
			ConvertToPdu(src.orientation, dst.Ref("orientation").GetPduWriteOps());
            dst.SetData("orientation_covariance", src.orientation_covariance);
			ConvertToPdu(src.angular_velocity, dst.Ref("angular_velocity").GetPduWriteOps());
            dst.SetData("angular_velocity_covariance", src.angular_velocity_covariance);
			ConvertToPdu(src.linear_acceleration, dst.Ref("linear_acceleration").GetPduWriteOps());
            dst.SetData("linear_acceleration_covariance", src.linear_acceleration_covariance);
        }
        private void ConvertToPdu(JointStateMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
            dst.SetData("name", src.name);
            dst.SetData("position", src.position);
            dst.SetData("velocity", src.velocity);
            dst.SetData("effort", src.effort);
        }
        private void ConvertToPdu(LaserScanMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
            dst.SetData("angle_min", src.angle_min);
            dst.SetData("angle_max", src.angle_max);
            dst.SetData("angle_increment", src.angle_increment);
            dst.SetData("time_increment", src.time_increment);
            dst.SetData("scan_time", src.scan_time);
            dst.SetData("range_min", src.range_min);
            dst.SetData("range_max", src.range_max);
            dst.SetData("ranges", src.ranges);
            dst.SetData("intensities", src.intensities);
        }
        private void ConvertToPdu(OdometryMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
            dst.SetData("child_frame_id", src.child_frame_id);
			ConvertToPdu(src.pose, dst.Ref("pose").GetPduWriteOps());
			ConvertToPdu(src.twist, dst.Ref("twist").GetPduWriteOps());
        }
        private void ConvertToPdu(PointMsg src, IPduWriteOperation dst)
        {
            dst.SetData("x", src.x);
            dst.SetData("y", src.y);
            dst.SetData("z", src.z);
        }
        private void ConvertToPdu(PoseMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.position, dst.Ref("position").GetPduWriteOps());
			ConvertToPdu(src.orientation, dst.Ref("orientation").GetPduWriteOps());
        }
        private void ConvertToPdu(PoseWithCovarianceMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.pose, dst.Ref("pose").GetPduWriteOps());
            dst.SetData("covariance", src.covariance);
        }
        private void ConvertToPdu(QuaternionMsg src, IPduWriteOperation dst)
        {
            dst.SetData("x", src.x);
            dst.SetData("y", src.y);
            dst.SetData("z", src.z);
            dst.SetData("w", src.w);
        }
        private void ConvertToPdu(RegionOfInterestMsg src, IPduWriteOperation dst)
        {
            dst.SetData("x_offset", src.x_offset);
            dst.SetData("y_offset", src.y_offset);
            dst.SetData("height", src.height);
            dst.SetData("width", src.width);
            dst.SetData("do_rectify", src.do_rectify);
        }
        private void ConvertToPdu(TFMessageMsg src, IPduWriteOperation dst)
        {
            foreach (var e in dst.Refs("transforms"))
            {
                int index = Array.IndexOf(dst.Refs("transforms"), e);
                if (src.transforms[index] == null) {
                    src.transforms[index] = new TransformStampedMsg();
                }
				ConvertToPdu(src.transforms[index], e.GetPduWriteOps());
            }
        }
        private void ConvertToPdu(TimeMsg src, IPduWriteOperation dst)
        {
            dst.SetData("sec", src.sec);
            dst.SetData("nanosec", src.nanosec);
        }
        private void ConvertToPdu(TransformMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.translation, dst.Ref("translation").GetPduWriteOps());
			ConvertToPdu(src.rotation, dst.Ref("rotation").GetPduWriteOps());
        }
        private void ConvertToPdu(TransformStampedMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
            dst.SetData("child_frame_id", src.child_frame_id);
			ConvertToPdu(src.transform, dst.Ref("transform").GetPduWriteOps());
        }
        private void ConvertToPdu(TwistMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.linear, dst.Ref("linear").GetPduWriteOps());
			ConvertToPdu(src.angular, dst.Ref("angular").GetPduWriteOps());
        }
        private void ConvertToPdu(TwistWithCovarianceMsg src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.twist, dst.Ref("twist").GetPduWriteOps());
            dst.SetData("covariance", src.covariance);
        }
        private void ConvertToPdu(Vector3Msg src, IPduWriteOperation dst)
        {
            dst.SetData("x", src.x);
            dst.SetData("y", src.y);
            dst.SetData("z", src.z);
        }

        public void ConvertToPduData(IPduCommData src, IPduReader dst)
        {
            RosTopicPduCommTypedData ros_topic = src as RosTopicPduCommTypedData;

            RosTopicPduReader ros_pdu_reader = dst as RosTopicPduReader;

            if (ros_pdu_reader.GetTypeName().Equals("sensor_msgs/Imu"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as ImuMsg;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("nav_msgs/Odometry"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as OdometryMsg;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("tf2_msgs/TFMessage"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as TFMessageMsg;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("sensor_msgs/JointState"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as JointStateMsg;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("geometry_msgs/Twist"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as TwistMsg;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("sensor_msgs/Image"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as ImageMsg;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("sensor_msgs/CompressedImage"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as CompressedImageMsg;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("sensor_msgs/CameraInfo"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as CameraInfoMsg;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("sensor_msgs/LaserScan"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as LaserScanMsg;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            throw new InvalidCastException("Can not find ros message type:" + ros_pdu_reader.GetTypeName());

        }
        static public Message ConvertToMessage(RosTopicPduReader pdu_reader)
        {
            return RosTopicPduWriterConverter.ConvertToMessage(pdu_reader.GetReadOps(), pdu_reader.GetTypeName());
        }
    }
}
