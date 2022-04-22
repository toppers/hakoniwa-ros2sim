using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class PoseWithCovarianceAccessor
    {
        private Pdu pdu;
		private PoseAccessor pdu_pose_accessor;
        
        public PoseWithCovarianceAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_pose_accessor = new PoseAccessor(pdu.Ref("pose"));
        }
        public PoseAccessor pose
        {
            set
            {
                pdu_pose_accessor = value;
            }
            get
            {
                return pdu_pose_accessor;
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
