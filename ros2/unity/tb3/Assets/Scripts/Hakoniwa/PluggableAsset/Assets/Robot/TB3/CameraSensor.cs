using Hakoniwa.PluggableAsset.Communication.Pdu;
using UnityEngine;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using System;

namespace Hakoniwa.PluggableAsset.Assets.Robot.TB3
{
    public class CameraSensor : MonoBehaviour, IRobotPartsSensor
    {
        private GameObject root;
        private GameObject sensor;
        public RenderTexture RenderTextureRef;
        //public string saveFilePath = "./SavedScreen.jpeg";
        private Texture2D tex;
        private byte[] raw_bytes;
        private byte[] jpg_bytes;
        private string frame_id = "camera_link";

        private PduIoConnector pdu_io;
        private int width = 640;
        private int height = 480;
        private string root_name;
        private string sensor_name;
        private Camera my_camera;
        private IPduWriter [] pdu = new IPduWriter[3];


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
                this.my_camera = this.GetComponentInChildren<Camera>();
                var texture = new Texture2D(this.width, this.height, TextureFormat.RGB24, false);
                this.RenderTextureRef = new RenderTexture(texture.width, texture.height, 32);
                this.my_camera.targetTexture = this.RenderTextureRef;
                this.sensor = this.gameObject;


                this.pdu[0] = this.pdu_io.GetWriter(this.root_name + "_camera_infoPdu");
                if (this.pdu[0] == null)
                {
                    throw new ArgumentException("can not found camera_info pdu:" + this.root_name + "_camera_infoPdu");
                }
                this.pdu[1] = this.pdu_io.GetWriter(this.root_name + "_imagePdu");
                if (this.pdu[1] == null)
                {
                    throw new ArgumentException("can not found image pdu:" + this.root_name + "_imagePdu");
                }
                this.pdu[2] = this.pdu_io.GetWriter(this.root_name + "_image" + "/" + "compressedPdu");
                if (this.pdu[2] == null)
                {
                    throw new ArgumentException("can not found image pdu:" + this.root_name + "_image" + "/" + "compressedPdu");
                }

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
              System.Array.Copy(_byte, i*step, raw_bytes, (height-i-1)*step, step);
            }

            // Encode texture into JPG
            jpg_bytes = tex.EncodeToJPG();
            UnityEngine.Object.Destroy(tex);
            //File.WriteAllBytes(saveFilePath, bytes);
        }
        private void UpdateSensorData(Pdu pdu)
        {
            if (pdu.GetName() == "sensor_msgs/Image") {
              TimeStamp.Set(pdu);
              pdu.Ref("header").SetData("frame_id", frame_id);
              pdu.SetData("height", (System.UInt32)RenderTextureRef.height);
              pdu.SetData("width", (System.UInt32)RenderTextureRef.width);
              pdu.SetData("encoding", "rgb8");
              pdu.SetData("step", (System.UInt32)RenderTextureRef.width*3);
              pdu.SetData("data", raw_bytes);
            } else if (pdu.GetName() == "sensor_msgs/CompressedImage") {
              TimeStamp.Set(pdu);
              pdu.Ref("header").SetData("frame_id", frame_id);
              pdu.SetData("format", "jpeg");
              pdu.SetData("data", jpg_bytes);
            } else {
                PublishCameraInfo();
            }
        }
        private void PublishCameraInfo()
        {
            double[] _D = new double[5] { 0.1639958233797625, -0.271840030972792, 0.001055841660100477, -0.00166555973740089, 0 };
            double[] _K = new double[9] { 322.0704122808738, 0, 199.2680620421962, 0, 320.8673986158544, 155.2533082600705, 0, 0, 1 };
            double[] _R = new double[9] { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
            double[] _P = new double[12] { 329.2483825683594, 0, 198.4101510452074, 0, 0, 329.1044006347656, 155.5057121208347, 0, 0, 0, 1, 0 };
            //PDU
            //header
            TimeStamp.Set(this.pdu[0].GetWriteOps().Ref(null));
            this.pdu[0].GetWriteOps().Ref("header").SetData("frame_id", frame_id);
            this.pdu[0].GetWriteOps().SetData("height", (System.UInt32)480);
            this.pdu[0].GetWriteOps().SetData("width", (System.UInt32)640);
            this.pdu[0].GetWriteOps().SetData("distortion_model", "plumb_bob");
            this.pdu[0].GetWriteOps().SetData("d", _D);
            this.pdu[0].GetWriteOps().SetData("k", _K);
            this.pdu[0].GetWriteOps().SetData("r", _R);
            this.pdu[0].GetWriteOps().SetData("p", _P);
            this.pdu[0].GetWriteOps().SetData("binning_x", (System.UInt32)0);
            this.pdu[0].GetWriteOps().SetData("binning_y", (System.UInt32)0);
        }


        public string [] topic_type = {
            "sensor_msgs/Image",
            "sensor_msgs/CompressedImage",
            "sensor_msgs/CameraInfo"
        };
        public string [] topic_name = {
            "image",
            "image/compressed",
            "camera_info"
        };
        public int [] update_cycle = {
            100,
            100,
            10
        };
        private int [] count = {
            0,
            0,
            0
        };
        public void UpdateSensorValues()
        {
            for (int i = 0; i < count.Length; i++)
            {
                this.count[i]++;
                if (this.count[i] < this.update_cycle[i])
                {
                    continue;
                }
                this.count[i] = 0;
                if (i == 0)
                {
                    this.Scan();
                }
                //Debug.Log("camera update[" + i + "]:" + this.pdu[i].GetWriteOps().Ref(null).GetName());
                this.UpdateSensorData(this.pdu[i].GetWriteOps().Ref(null));
            }
            return;
        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[topic_type.Length];
            int i = 0;
            for (i = 0; i < topic_type.Length; i++)
            {
                cfg[i] = new RosTopicMessageConfig();
                cfg[i].topic_message_name = this.topic_name[i];
                cfg[i].topic_type_name = this.topic_type[i];
                cfg[i].sub = false;
                cfg[i].pub_option = new RostopicPublisherOption();
                cfg[i].pub_option.cycle_scale = this.update_cycle[i];
                cfg[i].pub_option.latch = false;
                cfg[i].pub_option.queue_size = 1;
            }

            return cfg;
        }

        public bool isAttachedSpecificController()
        {
            return false;
        }

    }
}

