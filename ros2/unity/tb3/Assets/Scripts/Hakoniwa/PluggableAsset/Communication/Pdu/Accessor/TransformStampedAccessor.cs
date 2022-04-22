using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class TransformStampedAccessor
    {
        private Pdu pdu;
		private HeaderAccessor pdu_header_accessor;
		private TransformAccessor pdu_transform_accessor;
        
        public TransformStampedAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_header_accessor = new HeaderAccessor(pdu.Ref("header"));
			this.pdu_transform_accessor = new TransformAccessor(pdu.Ref("transform"));
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
        public string child_frame_id
        {
            set
            {
                pdu.SetData("child_frame_id", value);
            }
            get
            {
                return pdu.GetDataString("child_frame_id");
            }
        }
        public TransformAccessor transform
        {
            set
            {
                pdu_transform_accessor = value;
            }
            get
            {
                return pdu_transform_accessor;
            }
        }
    }
}
