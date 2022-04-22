using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Hakoniwa.PluggableAsset.Assets.Robot.EV3;


namespace Hakoniwa.PluggableAsset.Assets.Robot.SimpleRobot

{
    public class SimpleRobotParts : MonoBehaviour, IEV3Parts
    {
        private string motor_a = "Tire1";
        private string motor_b = "Tire2";
        private string color_sensor0 = "Axle/SensorHolder/SensorBox/ColorSensor";
        private string ultra_sonic_sensor = "Axle/SensorHolder/SensorBox/UltrasonicSensor";
        private string touch_sensor0 = "Axle/TouchSensor";
        private string gyro_sensor = "Axle/GyroSensor";
        private string motor_arm = "Axle/Arm/ArmMotor";

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
            return touch_sensor0;
        }
        public string getTouchSensor1()
        {
            return null;
        }
        public string getGyroSensor()
        {
            return this.gyro_sensor;
        }

        public string GetMotorC()
        {
            return this.motor_arm;
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
