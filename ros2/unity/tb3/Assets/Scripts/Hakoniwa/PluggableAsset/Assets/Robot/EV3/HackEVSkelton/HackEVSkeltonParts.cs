using Hakoniwa.PluggableAsset.Assets.Robot.EV3;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.HackEV
{
    public class HackEVSkeltonParts : MonoBehaviour, IEV3Parts
    {
        private string motor_a = "LeftTire";
        private string motor_b = "RightTire";
        private string color_sensor0 = "ArmMotor/ArmAxis/ColorSensor/Camera";
        private string ultra_sonic_sensor = "Axis/Body/UltraSonicSensor";
        private string gyro_sensor = null;
        private string motor_arm = "ArmMotor";
        private string touch_sensor0 = null;
        private string touch_sensor1 = "ArmMotor/ArmAxis/ColorSensor/FrontTouchSensor";
        private string led = "Axis/Body/LED";

        public string GetColorSensor0()
        {
            return color_sensor0;
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
            return touch_sensor0;
        }

        public string getGyroSensor()
        {
            return gyro_sensor;
        }

        public string GetMotorC()
        {
            return motor_arm;
        }

        string IEV3Parts.getTouchSensor1()
        {
            return touch_sensor1;
        }
        public string GetLed()
        {
            return led;
        }

        public string getGpsSensor()
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

        public void Load()
        {
            return;
        }
    }
}
