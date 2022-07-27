using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class Ev3PduColorSensorAccessor
    {
        private Pdu pdu;
        
        public Ev3PduColorSensorAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public UInt32 color
        {
            set
            {
                pdu.SetData("color", value);
            }
            get
            {
                return pdu.GetDataUInt32("color");
            }
        }
        public UInt32 reflect
        {
            set
            {
                pdu.SetData("reflect", value);
            }
            get
            {
                return pdu.GetDataUInt32("reflect");
            }
        }
        public UInt32 rgb_r
        {
            set
            {
                pdu.SetData("rgb_r", value);
            }
            get
            {
                return pdu.GetDataUInt32("rgb_r");
            }
        }
        public UInt32 rgb_g
        {
            set
            {
                pdu.SetData("rgb_g", value);
            }
            get
            {
                return pdu.GetDataUInt32("rgb_g");
            }
        }
        public UInt32 rgb_b
        {
            set
            {
                pdu.SetData("rgb_b", value);
            }
            get
            {
                return pdu.GetDataUInt32("rgb_b");
            }
        }
    }
}
