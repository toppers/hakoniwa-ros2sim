//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ev3
{
    [Serializable]
    public class Ev3PduSensorHeaderMsg : Message
    {
        public const string k_RosMessageName = "ev3_msgs/Ev3PduSensorHeader";
        public override string RosMessageName => k_RosMessageName;

        public string name;
        public uint version;
        public long hakoniwa_time;
        public uint ext_off;
        public uint ext_size;

        public Ev3PduSensorHeaderMsg()
        {
            this.name = "";
            this.version = 0;
            this.hakoniwa_time = 0;
            this.ext_off = 0;
            this.ext_size = 0;
        }

        public Ev3PduSensorHeaderMsg(string name, uint version, long hakoniwa_time, uint ext_off, uint ext_size)
        {
            this.name = name;
            this.version = version;
            this.hakoniwa_time = hakoniwa_time;
            this.ext_off = ext_off;
            this.ext_size = ext_size;
        }

        public static Ev3PduSensorHeaderMsg Deserialize(MessageDeserializer deserializer) => new Ev3PduSensorHeaderMsg(deserializer);

        private Ev3PduSensorHeaderMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.name);
            deserializer.Read(out this.version);
            deserializer.Read(out this.hakoniwa_time);
            deserializer.Read(out this.ext_off);
            deserializer.Read(out this.ext_size);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.name);
            serializer.Write(this.version);
            serializer.Write(this.hakoniwa_time);
            serializer.Write(this.ext_off);
            serializer.Write(this.ext_size);
        }

        public override string ToString()
        {
            return "Ev3PduSensorHeaderMsg: " +
            "\nname: " + name.ToString() +
            "\nversion: " + version.ToString() +
            "\nhakoniwa_time: " + hakoniwa_time.ToString() +
            "\next_off: " + ext_off.ToString() +
            "\next_size: " + ext_size.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
