using Hakoniwa.PluggableAsset.Assets.Robot.EV3;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.HackEV
{
    public class HackEVParts : MonoBehaviour, IEV3Parts
    {
        private string motor_a = "HackEV_L8_LeftMotor/HackEV_L8_Wheel2/L8_Tire_Bk";
        private string motor_b = "HackEV_L8_RightMotor/HackEV_L8_Wheel/L8_Tire_Bk 1";
        private string color_sensor0 = "EV3_InteractiveServoMotor_L_MotorRoot/EV3_ArmMort_Axis/HackEV_L6_RC1_LightSensor/EV3_ColorSensor/EV3_ColorSensor_Wh06/ColorSensor";
        private string color_sensor1 = null;
        private string ultra_sonic_sensor = "RoboModel_Axis/Body/HackEV_L8_Sidewinder_Head/EV3_UltrasonicSensor/EV3_UltrasonicSensor_Bk10";
        private string gyro_sensor = "RoboModel_Axis/Body/HackEV_L8_Sidewinder_Head/EV3_GyroSensor/EV3_GyroSensor_Ash01";
        private string motor_arm = "EV3_InteractiveServoMotor_L_MotorRoot";
        private string touch_sensor0 = "RoboModel_Axis/Body/HackEV_L9_Sidewinder_ShoulderSensor/HackEV_L8_TouchSensor2/L8_DoubleBevelGear20_Bk 1";
        private string touch_sensor1 = "EV3_InteractiveServoMotor_L_MotorRoot/EV3_ArmMort_Axis/HackEV_L6_RC1_LightSensor/EV3_ColorSensor/ColorSensorCollider";
        private string led = "RoboModel_Axis/Body/EV3_IntelligentBlock/EV3_IntelligentBlock_Root/EV3_IntelligentBlock_Ash01";
        private string gps = "RoboModel_Axis";

        public void Load()
        {
            return;
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
            return gps;
        }

        public string GetColorSensor0()
        {
            return color_sensor0;
        }

        public string GetColorSensor1()
        {
            return color_sensor1;
        }

        public string getButtonSensor(ButtonSensorType type)
        {
            switch (type)
            {
                case ButtonSensorType.BUTTON_SENSOR_LEFT:
                    return "RoboModel_Axis/Body/EV3_IntelligentBlock/EV3_IntelligentBlock_Root/EV3_IntelligentBlock_Ash05/TouchSensor";
                case ButtonSensorType.BUTTON_SENSOR_RIGHT:
                    return "RoboModel_Axis/Body/EV3_IntelligentBlock/EV3_IntelligentBlock_Root/EV3_IntelligentBlock_Ash06/TouchSensor";
                case ButtonSensorType.BUTTON_SENSOR_ENTER:
                    return "RoboModel_Axis/Body/EV3_IntelligentBlock/EV3_IntelligentBlock_Root/EV3_IntelligentBlock_DAsh07/TouchSensor";
                case ButtonSensorType.BUTTON_SENSOR_DOWN:
                    return "RoboModel_Axis/Body/EV3_IntelligentBlock/EV3_IntelligentBlock_Root/EV3_IntelligentBlock_Ash08/TouchSensor";
                case ButtonSensorType.BUTTON_SENSOR_UP:
                    return "RoboModel_Axis/Body/EV3_IntelligentBlock/EV3_IntelligentBlock_Root/EV3_IntelligentBlock_Ash09/TouchSensor";
                case ButtonSensorType.BUTTON_SENSOR_BACK:
                    return "RoboModel_Axis/Body/EV3_IntelligentBlock/EV3_IntelligentBlock_Root/EV3_IntelligentBlock_Ash04/TouchSensor";
            }
            throw new System.NotImplementedException();
        }
    }
}
