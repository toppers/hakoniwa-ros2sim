using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Hakoniwa.PluggableAsset.Assets.Robot.EV3;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Jeep

{
    public class JeepParts : MonoBehaviour, IEV3Parts
    {
        private string motor_a = "Jeep_Axle/Wheel_Front_Left";
        private string motor_b = "Jeep_Axle/Wheel_Front_Right";
        private string color_sensor0 = "Jeep_Axle/LineTraceSensorHolder/ColorSensor";
        private string ultra_sonic_sensor = "Jeep_Axle/UltrasonicSensor";

        public void Load()
        {
            return;
        }
        public string GetColorSensor0()
        {
            return color_sensor0;
        }
        public string GetColorSensor1()
        {
            return null;
        }

        public string GetMotorA()
        {
            return motor_a;
        }

        public string GetMotorB()
        {
            return motor_b;
        }

        public string getUltraSonicSensor()
        {
            return ultra_sonic_sensor;
        }
        public string getTouchSensor0()
        {
            return null;
        }
        public string getTouchSensor1()
        {
            return null;
        }

        public string getGyroSensor()
        {
            return null;
        }

        public string GetMotorC()
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

        public string getButtonSensor(ButtonSensorType type)
        {
            return null;
        }
    }
}
