using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class TFMessageAccessor
    {
        private Pdu pdu;
		private TransformStampedAccessor[] pdu_transforms_array_accessor;
        
        public TFMessageAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
            var pdu_transforms_array = pdu.Refs("transforms");
            for (int i = 0; i < pdu_transforms_array.Length; i++)
            {
                this.pdu_transforms_array_accessor[i] = new TransformStampedAccessor(pdu_transforms_array[i]);
            }
        }
        public TransformStampedAccessor[] transforms
        {
            get
            {
                return pdu_transforms_array_accessor;
            }
        }
    }
}
