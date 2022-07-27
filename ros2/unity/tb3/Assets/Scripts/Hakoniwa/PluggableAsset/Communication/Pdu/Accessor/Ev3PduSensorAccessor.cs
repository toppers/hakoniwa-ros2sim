using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class Ev3PduSensorAccessor
    {
        private Pdu pdu;
		private Ev3PduSensorHeaderAccessor pdu_head_accessor;
		private Ev3PduColorSensorAccessor[] pdu_color_sensors_array_accessor;
		private Ev3PduTouchSensorAccessor[] pdu_touch_sensors_array_accessor;
        
        public Ev3PduSensorAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_head_accessor = new Ev3PduSensorHeaderAccessor(pdu.Ref("head"));
            var pdu_color_sensors_array = pdu.Refs("color_sensors");
            for (int i = 0; i < pdu_color_sensors_array.Length; i++)
            {
                this.pdu_color_sensors_array_accessor[i] = new Ev3PduColorSensorAccessor(pdu_color_sensors_array[i]);
            }
            var pdu_touch_sensors_array = pdu.Refs("touch_sensors");
            for (int i = 0; i < pdu_touch_sensors_array.Length; i++)
            {
                this.pdu_touch_sensors_array_accessor[i] = new Ev3PduTouchSensorAccessor(pdu_touch_sensors_array[i]);
            }
        }
        public Ev3PduSensorHeaderAccessor head
        {
            set
            {
                pdu_head_accessor = value;
            }
            get
            {
                return pdu_head_accessor;
            }
        }
        public Byte[] buttons
        {
            get
            {
                return pdu.GetDataUInt8Array("buttons");
            }
        }
        public Ev3PduColorSensorAccessor[] color_sensors
        {
            get
            {
                return pdu_color_sensors_array_accessor;
            }
        }
        public Ev3PduTouchSensorAccessor[] touch_sensors
        {
            get
            {
                return pdu_touch_sensors_array_accessor;
            }
        }
        public UInt32[] motor_angle
        {
            get
            {
                return pdu.GetDataUInt32Array("motor_angle");
            }
        }
        public Int32 gyro_degree
        {
            set
            {
                pdu.SetData("gyro_degree", value);
            }
            get
            {
                return pdu.GetDataInt32("gyro_degree");
            }
        }
        public Int32 gyro_degree_rate
        {
            set
            {
                pdu.SetData("gyro_degree_rate", value);
            }
            get
            {
                return pdu.GetDataInt32("gyro_degree_rate");
            }
        }
        public UInt32 sensor_ultrasonic
        {
            set
            {
                pdu.SetData("sensor_ultrasonic", value);
            }
            get
            {
                return pdu.GetDataUInt32("sensor_ultrasonic");
            }
        }
        public double gps_lat
        {
            set
            {
                pdu.SetData("gps_lat", value);
            }
            get
            {
                return pdu.GetDataFloat64("gps_lat");
            }
        }
        public double gps_lon
        {
            set
            {
                pdu.SetData("gps_lon", value);
            }
            get
            {
                return pdu.GetDataFloat64("gps_lon");
            }
        }
    }
}
