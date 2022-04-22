using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class TwistWithCovarianceAccessor
    {
        private Pdu pdu;
		private TwistAccessor pdu_twist_accessor;
        
        public TwistWithCovarianceAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_twist_accessor = new TwistAccessor(pdu.Ref("twist"));
        }
        public TwistAccessor twist
        {
            set
            {
                pdu_twist_accessor = value;
            }
            get
            {
                return pdu_twist_accessor;
            }
        }
        public double[] covariance
        {
            get
            {
                return pdu.GetDataFloat64Array("covariance");
            }
        }
    }
}
