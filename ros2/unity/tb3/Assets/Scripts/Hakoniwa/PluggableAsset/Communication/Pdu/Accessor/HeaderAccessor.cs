using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class HeaderAccessor
    {
        private Pdu pdu;
		private TimeAccessor pdu_stamp_accessor;
        
        public HeaderAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_stamp_accessor = new TimeAccessor(pdu.Ref("stamp"));
        }
        public TimeAccessor stamp
        {
            set
            {
                pdu_stamp_accessor = value;
            }
            get
            {
                return pdu_stamp_accessor;
            }
        }
        public string frame_id
        {
            set
            {
                pdu.SetData("frame_id", value);
            }
            get
            {
                return pdu.GetDataString("frame_id");
            }
        }
    }
}
