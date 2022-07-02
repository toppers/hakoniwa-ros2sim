using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class Ev3PduMotorAccessor
    {
        private Pdu pdu;
        
        public Ev3PduMotorAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public Int32 power
        {
            set
            {
                pdu.SetData("power", value);
            }
            get
            {
                return pdu.GetDataInt32("power");
            }
        }
        public UInt32 stop
        {
            set
            {
                pdu.SetData("stop", value);
            }
            get
            {
                return pdu.GetDataUInt32("stop");
            }
        }
        public UInt32 reset_angle
        {
            set
            {
                pdu.SetData("reset_angle", value);
            }
            get
            {
                return pdu.GetDataUInt32("reset_angle");
            }
        }
    }
}
