using Hakoniwa.PluggableAsset.Assets.Robot.EV3;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3Crossing
{
    public class EV3CrossingParts : MonoBehaviour, IEV3Parts
    {
        private string motor_arm = "EV3GuardHolder";

        public void Load()
        {
            return;
        }
        public string getGyroSensor()
        {
            return null;
        }

        public string GetMotorA()
        {
            return null;
        }

        public string GetMotorB()
        {
            return null;
        }

        public string GetMotorC()
        {
            return motor_arm;
            //return null;
        }

        public string getTouchSensor0()
        {
            return null;
        }

        public string getTouchSensor1()
        {
            return null;
        }

        public string getUltraSonicSensor()
        {
            return null;
        }
        public string GetLed()
        {
            return null;
        }
        public string getGpsSensor()
        {
            return null;
        }

        public string GetColorSensor0()
        {
            return null;
        }

        public string GetColorSensor1()
        {
            return null;
        }

        public string getButtonSensor(ButtonSensorType type)
        {
            return null;
        }
    }

}


