using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class OdometryAccessor
    {
        private Pdu pdu;
		private HeaderAccessor pdu_header_accessor;
		private PoseWithCovarianceAccessor pdu_pose_accessor;
		private TwistWithCovarianceAccessor pdu_twist_accessor;
        
        public OdometryAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_header_accessor = new HeaderAccessor(pdu.Ref("header"));
			this.pdu_pose_accessor = new PoseWithCovarianceAccessor(pdu.Ref("pose"));
			this.pdu_twist_accessor = new TwistWithCovarianceAccessor(pdu.Ref("twist"));
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
        public PoseWithCovarianceAccessor pose
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
        public TwistWithCovarianceAccessor twist
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
    }
}
