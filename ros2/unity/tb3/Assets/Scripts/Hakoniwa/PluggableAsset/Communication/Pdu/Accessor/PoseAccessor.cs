using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class PoseAccessor
    {
        private Pdu pdu;
		private PointAccessor pdu_position_accessor;
		private QuaternionAccessor pdu_orientation_accessor;
        
        public PoseAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_position_accessor = new PointAccessor(pdu.Ref("position"));
			this.pdu_orientation_accessor = new QuaternionAccessor(pdu.Ref("orientation"));
        }
        public PointAccessor position
        {
            set
            {
                pdu_position_accessor = value;
            }
            get
            {
                return pdu_position_accessor;
            }
        }
        public QuaternionAccessor orientation
        {
            set
            {
                pdu_orientation_accessor = value;
            }
            get
            {
                return pdu_orientation_accessor;
            }
        }
    }
}
