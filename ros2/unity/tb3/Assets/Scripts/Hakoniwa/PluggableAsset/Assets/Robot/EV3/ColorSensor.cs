using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class ColorSensor : MonoBehaviour, IRobotPartsSensor
    {
        private string root_name;
        private IPduWriter pdu_writer;
        private PduIoConnector pdu_io;

        public int colorSensorNo;
        public Camera dispCamera;
        private float lightValue;
        private GameObject obj = null;
        public Color rgb;
        private float rgb_r;
        private float rgb_g;
        private float rgb_b;
        private float sensor_rgb_max = 255;
        private int sensor_rgb_r;
        private int sensor_rgb_g;
        private int sensor_rgb_b;
        private Texture2D targetTexture;
        private int r_col, g_col, b_col;
        private ColorNumber[,,] color_array = new ColorNumber[3, 3, 3];
        private ColorNumber color_id = ColorNumber.COLOR_NONE;

        public void Initialize(GameObject root)
        {
            if (this.obj != null)
            {
                return;
            }
            this.obj = root;
            this.root_name = string.Copy(this.obj.transform.name);
            this.pdu_io = PduIoConnector.Get(this.root_name);
            this.pdu_writer = this.pdu_io.GetWriter(this.root_name + "_ev3_sensorPdu");
            if (this.pdu_writer == null)
            {
                throw new ArgumentException("can not found ev3_sensor pdu:" + this.root_name + "_ev3_sensorPdu");
            }

            this.dispCamera = this.GetComponentInChildren<Camera>();
            this.dispCamera.targetTexture = new RenderTexture(32, 32, 24, RenderTextureFormat.BGRA32);
            var tex = dispCamera.targetTexture;
            targetTexture = new Texture2D(tex.width, tex.height, TextureFormat.ARGB32, false);
            color_array[0, 0, 0] = ColorNumber.COLOR_BLACK;
            color_array[0, 0, 1] = ColorNumber.COLOR_BLACK;
            color_array[0, 0, 2] = ColorNumber.COLOR_BLUE;

            color_array[0, 1, 0] = ColorNumber.COLOR_GREEN;
            color_array[0, 1, 1] = ColorNumber.COLOR_GREEN;
            color_array[0, 1, 2] = ColorNumber.COLOR_BLUE;

            color_array[0, 2, 0] = ColorNumber.COLOR_GREEN;
            color_array[0, 2, 1] = ColorNumber.COLOR_GREEN;
            color_array[0, 2, 2] = ColorNumber.COLOR_BLUE;

            color_array[1, 0, 0] = ColorNumber.COLOR_BROWN;
            color_array[1, 0, 1] = ColorNumber.COLOR_RED;
            color_array[1, 0, 2] = ColorNumber.COLOR_RED;

            color_array[1, 1, 0] = ColorNumber.COLOR_YELLOW;
            color_array[1, 1, 1] = ColorNumber.COLOR_BLACK;
            color_array[1, 1, 2] = ColorNumber.COLOR_BLUE;

            color_array[1, 2, 0] = ColorNumber.COLOR_GREEN;
            color_array[1, 2, 1] = ColorNumber.COLOR_GREEN;
            color_array[1, 2, 2] = ColorNumber.COLOR_BLUE;

            color_array[2, 0, 0] = ColorNumber.COLOR_RED;
            color_array[2, 0, 1] = ColorNumber.COLOR_RED;
            color_array[2, 0, 2] = ColorNumber.COLOR_RED;

            color_array[2, 1, 0] = ColorNumber.COLOR_BROWN;
            color_array[2, 1, 1] = ColorNumber.COLOR_RED;
            color_array[2, 1, 2] = ColorNumber.COLOR_RED;

            color_array[2, 2, 0] = ColorNumber.COLOR_YELLOW;
            color_array[2, 2, 1] = ColorNumber.COLOR_YELLOW;
            color_array[2, 2, 2] = ColorNumber.COLOR_WHITE;

        }

        private int GetCol(float x)
        {
            int y = (int)(x * 255);
            if (y < 75)
            {
                return 0;
            }
            else if (y < 128)
            {
                return 1;
            }
            return 2;
        }

        private void UpdateSensorValuesLocal()
        {
            var tex = dispCamera.targetTexture;
            // RenderTextureキャプチャ
            RenderTexture.active = dispCamera.targetTexture;

            targetTexture.ReadPixels(new Rect(0, 0, tex.width, tex.height), 0, 0);
            targetTexture.Apply();

            // 照度を取得する
            this.lightValue = GetLightValue(targetTexture);

            //color id を取得する．
            r_col = this.GetCol(this.rgb_r);
            g_col = this.GetCol(this.rgb_g);
            b_col = this.GetCol(this.rgb_b);
            this.color_id = this.color_array[r_col, g_col, b_col];
        }

        // 画像全体の照度計算
        private float GetLightValue(Texture2D tex)
        {
            var cols = tex.GetPixels();

            // 平均色計算
            Color avg = new Color(0, 0, 0);
            foreach (var col in cols)
            {
                avg += col;
            }
            avg /= cols.Length;
            this.rgb = avg;
            this.rgb_r = avg.r;
            this.rgb_g = avg.g;
            this.rgb_b = avg.b;

            this.sensor_rgb_r = (int)(avg.r * this.sensor_rgb_max);
            this.sensor_rgb_g = (int)(avg.g * this.sensor_rgb_max);
            this.sensor_rgb_b = (int)(avg.b * this.sensor_rgb_max);

            // 照度計算
            return 0.299f * avg.r + 0.587f * avg.g + 0.114f * avg.b;
        }
        public ColorNumber GetColorId()
        {
            return this.color_id;
        }
        public float GetLightValue()
        {
            return this.lightValue;
        }
        public void GetRgb(out ColorRGB value)
        {
            value.r = this.sensor_rgb_r;
            value.g = this.sensor_rgb_g;
            value.b = this.sensor_rgb_b;
        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            return null;
        }

        public bool isAttachedSpecificController()
        {
            return false;
        }

        public void UpdateSensorValues()
        {
            this.UpdateSensorValuesLocal();

            this.pdu_writer.GetWriteOps().Refs("color_sensors")[this.colorSensorNo].SetData("reflect", (uint)(this.GetLightValue() * 100f));
            ColorRGB color_sensor_rgb;
            this.GetRgb(out color_sensor_rgb);
            this.pdu_writer.GetWriteOps().Refs("color_sensors")[this.colorSensorNo].SetData("rgb_r", (uint)color_sensor_rgb.r);
            this.pdu_writer.GetWriteOps().Refs("color_sensors")[this.colorSensorNo].SetData("rgb_g", (uint)color_sensor_rgb.g);
            this.pdu_writer.GetWriteOps().Refs("color_sensors")[this.colorSensorNo].SetData("rgb_b", (uint)color_sensor_rgb.b);
            this.pdu_writer.GetWriteOps().Refs("color_sensors")[this.colorSensorNo].SetData("color", (uint)this.GetColorId());
        }
    }
}
