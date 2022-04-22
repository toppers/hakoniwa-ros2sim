using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class TwistAccessor
    {
        private Pdu pdu;
		private Vector3Accessor pdu_linear_accessor;
		private Vector3Accessor pdu_angular_accessor;
        
        public TwistAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_linear_accessor = new Vector3Accessor(pdu.Ref("linear"));
			this.pdu_angular_accessor = new Vector3Accessor(pdu.Ref("angular"));
        }
        public Vector3Accessor linear
        {
            set
            {
                pdu_linear_accessor = value;
            }
            get
            {
                return pdu_linear_accessor;
            }
        }
        public Vector3Accessor angular
        {
            set
            {
                pdu_angular_accessor = value;
            }
            get
            {
                return pdu_angular_accessor;
            }
        }
    }
}
