using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    class HakoEnvAccessor
    {
        private Pdu pdu;
        
        public HakoEnvAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public UInt64 simtime
        {
            set
            {
                pdu.SetData("simtime", value);
            }
            get
            {
                return pdu.GetDataUInt64("simtime");
            }
        }
    }
}
