using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class TimeAccessor
    {
        private Pdu pdu;
        
        public TimeAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public Int32 sec
        {
            set
            {
                pdu.SetData("sec", value);
            }
            get
            {
                return pdu.GetDataInt32("sec");
            }
        }
        public UInt32 nanosec
        {
            set
            {
                pdu.SetData("nanosec", value);
            }
            get
            {
                return pdu.GetDataUInt32("nanosec");
            }
        }
    }
}
