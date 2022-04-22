using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class Vector3Accessor
    {
        private Pdu pdu;
        
        public Vector3Accessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public double x
        {
            set
            {
                pdu.SetData("x", value);
            }
            get
            {
                return pdu.GetDataFloat64("x");
            }
        }
        public double y
        {
            set
            {
                pdu.SetData("y", value);
            }
            get
            {
                return pdu.GetDataFloat64("y");
            }
        }
        public double z
        {
            set
            {
                pdu.SetData("z", value);
            }
            get
            {
                return pdu.GetDataFloat64("z");
            }
        }
    }
}
