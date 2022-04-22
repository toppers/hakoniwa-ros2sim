using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class RegionOfInterestAccessor
    {
        private Pdu pdu;
        
        public RegionOfInterestAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public UInt32 x_offset
        {
            set
            {
                pdu.SetData("x_offset", value);
            }
            get
            {
                return pdu.GetDataUInt32("x_offset");
            }
        }
        public UInt32 y_offset
        {
            set
            {
                pdu.SetData("y_offset", value);
            }
            get
            {
                return pdu.GetDataUInt32("y_offset");
            }
        }
        public UInt32 height
        {
            set
            {
                pdu.SetData("height", value);
            }
            get
            {
                return pdu.GetDataUInt32("height");
            }
        }
        public UInt32 width
        {
            set
            {
                pdu.SetData("width", value);
            }
            get
            {
                return pdu.GetDataUInt32("width");
            }
        }
        public bool do_rectify
        {
            set
            {
                pdu.SetData("do_rectify", value);
            }
            get
            {
                return pdu.GetDataBool("do_rectify");
            }
        }
    }
}
