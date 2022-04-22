using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class CompressedImageAccessor
    {
        private Pdu pdu;
		private HeaderAccessor pdu_header_accessor;
        
        public CompressedImageAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_header_accessor = new HeaderAccessor(pdu.Ref("header"));
        }
        public HeaderAccessor header
        {
            set
            {
                pdu_header_accessor = value;
            }
            get
            {
                return pdu_header_accessor;
            }
        }
        public string format
        {
            set
            {
                pdu.SetData("format", value);
            }
            get
            {
                return pdu.GetDataString("format");
            }
        }
        public Byte[] data
        {
            get
            {
                return pdu.GetDataUInt8Array("data");
            }
        }
    }
}
