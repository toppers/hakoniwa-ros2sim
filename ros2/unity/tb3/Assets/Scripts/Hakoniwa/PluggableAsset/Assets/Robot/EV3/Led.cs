using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class Led : MonoBehaviour, IRobotPartsController
    {
        private GameObject root;
        private IPduReader pdu_reader;
        private string root_name;
        private PduIoConnector pdu_io;

        public Material[] materials = new Material[4];
        private Renderer rend;

        public void Initialize(GameObject root)
        {
            if (this.root != null)
            {
                rend = GetComponent<Renderer>();
                rend.material.color = materials[0].color;
                return;
            }
            this.root = root;

            this.root_name = string.Copy(this.root.transform.name);
            this.pdu_io = PduIoConnector.Get(this.root_name);
            this.pdu_reader = this.pdu_io.GetReader(this.root_name + "_ev3_actuatorPdu");
            if (this.pdu_reader == null)
            {
                throw new ArgumentException("can not found _ev3_actuator pdu:" + this.root_name + "_ev3_actuatorPdu");
            }
            rend = GetComponent<Renderer>();
            rend.material.color = materials[0].color;

        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            return null;
        }

        public void SetLedColor(LedColor color)
        {
            //Debug.Log("color=" +color);
            rend.material.color = materials[(int)color].color;
        }
        //public int debug_led_color = 0;

        public void DoControl()
        {
            int led_color = this.pdu_reader.GetReadOps().GetDataUInt8Array("leds")[0];
            //int led_color = debug_led_color;
            this.SetLedColor((LedColor)(((led_color) & 0x3)));

        }
    }
}

