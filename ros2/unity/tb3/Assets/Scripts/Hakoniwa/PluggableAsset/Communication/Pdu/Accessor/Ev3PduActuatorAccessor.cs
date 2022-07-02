using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class Ev3PduActuatorAccessor
    {
        private Pdu pdu;
		private Ev3PduActuatorHeaderAccessor pdu_head_accessor;
		private Ev3PduMotorAccessor[] pdu_motors_array_accessor;
        
        public Ev3PduActuatorAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_head_accessor = new Ev3PduActuatorHeaderAccessor(pdu.Ref("head"));
            var pdu_motors_array = pdu.Refs("motors");
            for (int i = 0; i < pdu_motors_array.Length; i++)
            {
                this.pdu_motors_array_accessor[i] = new Ev3PduMotorAccessor(pdu_motors_array[i]);
            }
        }
        public Ev3PduActuatorHeaderAccessor head
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
        public Byte[] leds
        {
            get
            {
                return pdu.GetDataUInt8Array("leds");
            }
        }
        public Ev3PduMotorAccessor[] motors
        {
            get
            {
                return pdu_motors_array_accessor;
            }
        }
        public UInt32 gyro_reset
        {
            set
            {
                pdu.SetData("gyro_reset", value);
            }
            get
            {
                return pdu.GetDataUInt32("gyro_reset");
            }
        }
    }
}
