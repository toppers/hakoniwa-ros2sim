using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using Assets.Scripts.Hakoniwa.PluggableAsset.Assets.Robot;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public class CameraSensor : MonoBehaviour, IRobotPartsSensor
    {
        private GameObject root;
        private GameObject sensor;
        private PduIoConnector pdu_io;
        private IPduWriter pdu_writer;
        private RenderTexture RenderTextureRef;
        private Texture2D tex;
        private int width = 640;
        private int height = 480;
        private byte[] raw_bytes;
        private byte[] jpg_bytes;
        private string root_name;
        private string sensor_name;

        public void Initialize(GameObject root)
        {
            if (this.root == null)
            {
                this.root = root;
                this.root_name = string.Copy(this.root.transform.name);
                this.sensor_name = string.Copy(this.transform.name);
                this.pdu_io = PduIoConnector.Get(root_name);
                if (this.pdu_io == null)
                {
                    throw new ArgumentException("can not found pdu_io:" + root_name);
                }
                var pdu_writer_name = root_name + "_" + this.topic_name + "Pdu";
                this.pdu_writer = this.pdu_io.GetWriter(pdu_writer_name);
                if (this.pdu_writer == null)
                {
                    throw new ArgumentException("can not found pdu_reader:" + pdu_writer_name);
                }
                this.my_camera = this.GetComponentInChildren<Camera>();
                var texture = new Texture2D(this.width, this.height, TextureFormat.RGB24, false);
                this.RenderTextureRef = new RenderTexture(texture.width, texture.height, 32);
                this.my_camera.targetTexture = this.RenderTextureRef;
                this.sensor = this.gameObject;
            }
        }

        private void Scan()
        {
            tex = new Texture2D(RenderTextureRef.width, RenderTextureRef.height, TextureFormat.RGB24, false);
            RenderTexture.active = RenderTextureRef;
            int width = RenderTextureRef.width;
            int height = RenderTextureRef.height;
            int step = width * 3;
            tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            tex.Apply();
            // raw_bytes = tex.GetRawTextureData();

            // Raw Image RGB24=(ROS)rgb8
            byte[] _byte = tex.GetRawTextureData();
            raw_bytes = new byte[_byte.Length];
            for (int i = 0; i < height; i++)
            {
                System.Array.Copy(_byte, i * step, raw_bytes, (height - i - 1) * step, step);
            }

            // Encode texture into JPG
            jpg_bytes = tex.EncodeToJPG();
            UnityEngine.Object.Destroy(tex);
        }

        private int count = 0;
        public void UpdateSensorValues()
        {
            this.count++;
            if (this.count < this.update_cycle)
            {
                return;
            }
            this.count = 0;
            this.Scan();
            this.UpdatePdu(this.pdu_writer.GetWriteOps().Ref(null));
            return;
        }

        private void UpdatePdu(Pdu pdu)
        {
            TimeStamp.Set(pdu);
            pdu.Ref("header").SetData("frame_id", this.sensor_name);
            pdu.SetData("format", "jpeg");
            pdu.SetData("data", jpg_bytes);
        }


        public bool isAttachedSpecificController()
        {
            return false;
        }

        public string topic_type = "sensor_msgs/CompressedImage";
        public int update_cycle = 100;
        private string topic_name = "camera_image_jpg";
        private Camera my_camera;

        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[1];
            cfg[0] = new RosTopicMessageConfig();
            cfg[0].topic_type_name = this.topic_type;
            cfg[0].topic_message_name = this.topic_name;
            cfg[0].sub = false;
            cfg[0].pub_option = new RostopicPublisherOption();
            cfg[0].pub_option.cycle_scale = this.update_cycle;
            cfg[0].pub_option.latch = false;
            cfg[0].pub_option.queue_size = 1;
            return cfg;
        }
    }

}
