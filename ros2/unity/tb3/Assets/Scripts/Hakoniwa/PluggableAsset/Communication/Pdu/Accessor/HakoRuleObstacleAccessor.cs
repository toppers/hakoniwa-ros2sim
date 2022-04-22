using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    class HakoRuleObstacleAccessor
    {
        private Pdu pdu;
        
        public HakoRuleObstacleAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public bool[] is_touch
        {
            get
            {
                return pdu.GetDataBoolArray("is_touch");
            }
        }
    }
}
