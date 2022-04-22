using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class JointStateAccessor
    {
        private Pdu pdu;
		private HeaderAccessor pdu_header_accessor;
        
        public JointStateAccessor(Pdu pdu)
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
        public string[] name
        {
            get
            {
                return pdu.GetDataStringArray("name");
            }
        }
        public double[] position
        {
            get
            {
                return pdu.GetDataFloat64Array("position");
            }
        }
        public double[] velocity
        {
            get
            {
                return pdu.GetDataFloat64Array("velocity");
            }
        }
        public double[] effort
        {
            get
            {
                return pdu.GetDataFloat64Array("effort");
            }
        }
    }
}
