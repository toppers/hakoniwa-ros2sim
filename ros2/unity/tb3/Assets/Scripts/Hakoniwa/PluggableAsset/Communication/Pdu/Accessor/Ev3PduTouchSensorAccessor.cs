using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class Ev3PduTouchSensorAccessor
    {
        private Pdu pdu;
        
        public Ev3PduTouchSensorAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public UInt32 value
        {
            set
            {
                pdu.SetData("value", value);
            }
            get
            {
                return pdu.GetDataUInt32("value");
            }
        }
    }
}
