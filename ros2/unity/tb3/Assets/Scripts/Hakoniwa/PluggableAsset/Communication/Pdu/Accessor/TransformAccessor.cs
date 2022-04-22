using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class TransformAccessor
    {
        private Pdu pdu;
		private Vector3Accessor pdu_translation_accessor;
		private QuaternionAccessor pdu_rotation_accessor;
        
        public TransformAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_translation_accessor = new Vector3Accessor(pdu.Ref("translation"));
			this.pdu_rotation_accessor = new QuaternionAccessor(pdu.Ref("rotation"));
        }
        public Vector3Accessor translation
        {
            set
            {
                pdu_translation_accessor = value;
            }
            get
            {
                return pdu_translation_accessor;
            }
        }
        public QuaternionAccessor rotation
        {
            set
            {
                pdu_rotation_accessor = value;
            }
            get
            {
                return pdu_rotation_accessor;
            }
        }
    }
}
