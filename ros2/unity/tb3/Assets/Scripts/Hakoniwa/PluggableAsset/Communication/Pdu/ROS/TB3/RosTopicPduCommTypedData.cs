using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS.TB3;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.ROS.TB3
{
    public class RosTopicPduCommTypedData : IPduCommTypedData
    {
        private Message topic_data = null;
        private string topic_name;
        private string topic_type_name;

        public Message GetTopicData()
        {
            return this.topic_data;
        }

        public RosTopicPduCommTypedData(RosTopicPduWriter pdu_writer)
        {
            this.topic_name = pdu_writer.GetTopicName();
            this.topic_type_name = pdu_writer.GetTypeName();
            this.topic_data = RosTopicPduWriterConverter.ConvertToMessage(pdu_writer);
        }
        public RosTopicPduCommTypedData(RosTopicPduReader pdu_reader)
        {
            this.topic_name = pdu_reader.GetTopicName();
            this.topic_type_name = pdu_reader.GetTypeName();
            this.topic_data = RosTopicPduReaderConverter.ConvertToMessage(pdu_reader);
        }
        public RosTopicPduCommTypedData(string name, string type, Message data)
        {
            this.topic_name = name;
            this.topic_type_name = type;
            this.topic_data = data;
        }

        public string GetDataName()
        {
            return this.topic_name;
        }

        public string GetDataTypeName()
        {
            return this.topic_type_name;
        }
    }
}
