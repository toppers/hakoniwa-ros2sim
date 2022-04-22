using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class CameraInfoAccessor
    {
        private Pdu pdu;
		private HeaderAccessor pdu_header_accessor;
		private RegionOfInterestAccessor pdu_roi_accessor;
        
        public CameraInfoAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_header_accessor = new HeaderAccessor(pdu.Ref("header"));
			this.pdu_roi_accessor = new RegionOfInterestAccessor(pdu.Ref("roi"));
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
        public string distortion_model
        {
            set
            {
                pdu.SetData("distortion_model", value);
            }
            get
            {
                return pdu.GetDataString("distortion_model");
            }
        }
        public double[] d
        {
            get
            {
                return pdu.GetDataFloat64Array("d");
            }
        }
        public double[] k
        {
            get
            {
                return pdu.GetDataFloat64Array("k");
            }
        }
        public double[] r
        {
            get
            {
                return pdu.GetDataFloat64Array("r");
            }
        }
        public double[] p
        {
            get
            {
                return pdu.GetDataFloat64Array("p");
            }
        }
        public UInt32 binning_x
        {
            set
            {
                pdu.SetData("binning_x", value);
            }
            get
            {
                return pdu.GetDataUInt32("binning_x");
            }
        }
        public UInt32 binning_y
        {
            set
            {
                pdu.SetData("binning_y", value);
            }
            get
            {
                return pdu.GetDataUInt32("binning_y");
            }
        }
        public RegionOfInterestAccessor roi
        {
            set
            {
                pdu_roi_accessor = value;
            }
            get
            {
                return pdu_roi_accessor;
            }
        }
    }
}
