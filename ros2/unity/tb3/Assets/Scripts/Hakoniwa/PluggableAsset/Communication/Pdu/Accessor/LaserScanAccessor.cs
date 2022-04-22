using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class LaserScanAccessor
    {
        private Pdu pdu;
		private HeaderAccessor pdu_header_accessor;
        
        public LaserScanAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_header_accessor = new HeaderAccessor(pdu.Ref("header"));
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
        public float angle_min
        {
            set
            {
                pdu.SetData("angle_min", value);
            }
            get
            {
                return pdu.GetDataFloat32("angle_min");
            }
        }
        public float angle_max
        {
            set
            {
                pdu.SetData("angle_max", value);
            }
            get
            {
                return pdu.GetDataFloat32("angle_max");
            }
        }
        public float angle_increment
        {
            set
            {
                pdu.SetData("angle_increment", value);
            }
            get
            {
                return pdu.GetDataFloat32("angle_increment");
            }
        }
        public float time_increment
        {
            set
            {
                pdu.SetData("time_increment", value);
            }
            get
            {
                return pdu.GetDataFloat32("time_increment");
            }
        }
        public float scan_time
        {
            set
            {
                pdu.SetData("scan_time", value);
            }
            get
            {
                return pdu.GetDataFloat32("scan_time");
            }
        }
        public float range_min
        {
            set
            {
                pdu.SetData("range_min", value);
            }
            get
            {
                return pdu.GetDataFloat32("range_min");
            }
        }
        public float range_max
        {
            set
            {
                pdu.SetData("range_max", value);
            }
            get
            {
                return pdu.GetDataFloat32("range_max");
            }
        }
        public float[] ranges
        {
            get
            {
                return pdu.GetDataFloat32Array("ranges");
            }
        }
        public float[] intensities
        {
            get
            {
                return pdu.GetDataFloat32Array("intensities");
            }
        }
    }
}
