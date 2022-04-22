using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class ImageAccessor
    {
        private Pdu pdu;
		private HeaderAccessor pdu_header_accessor;
        
        public ImageAccessor(Pdu pdu)
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
        public UInt32 height
        {
            set
            {
                pdu.SetData("height", value);
            }
            get
            {
                return pdu.GetDataUInt32("height");
            }
        }
        public UInt32 width
        {
            set
            {
                pdu.SetData("width", value);
            }
            get
            {
                return pdu.GetDataUInt32("width");
            }
        }
        public string encoding
        {
            set
            {
                pdu.SetData("encoding", value);
            }
            get
            {
                return pdu.GetDataString("encoding");
            }
        }
        public Byte is_bigendian
        {
            set
            {
                pdu.SetData("is_bigendian", value);
            }
            get
            {
                return pdu.GetDataUInt8("is_bigendian");
            }
        }
        public UInt32 step
        {
            set
            {
                pdu.SetData("step", value);
            }
            get
            {
                return pdu.GetDataUInt32("step");
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
