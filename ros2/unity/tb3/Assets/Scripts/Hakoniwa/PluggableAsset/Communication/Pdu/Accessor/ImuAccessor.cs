using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class ImuAccessor
    {
        private Pdu pdu;
		private HeaderAccessor pdu_header_accessor;
		private QuaternionAccessor pdu_orientation_accessor;
		private Vector3Accessor pdu_angular_velocity_accessor;
		private Vector3Accessor pdu_linear_acceleration_accessor;
        
        public ImuAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
			this.pdu_header_accessor = new HeaderAccessor(pdu.Ref("header"));
			this.pdu_orientation_accessor = new QuaternionAccessor(pdu.Ref("orientation"));
			this.pdu_angular_velocity_accessor = new Vector3Accessor(pdu.Ref("angular_velocity"));
			this.pdu_linear_acceleration_accessor = new Vector3Accessor(pdu.Ref("linear_acceleration"));
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
        public double[] orientation_covariance
        {
            get
            {
                return pdu.GetDataFloat64Array("orientation_covariance");
            }
        }
        public Vector3Accessor angular_velocity
        {
            set
            {
                pdu_angular_velocity_accessor = value;
            }
            get
            {
                return pdu_angular_velocity_accessor;
            }
        }
        public double[] angular_velocity_covariance
        {
            get
            {
                return pdu.GetDataFloat64Array("angular_velocity_covariance");
            }
        }
        public Vector3Accessor linear_acceleration
        {
            set
            {
                pdu_linear_acceleration_accessor = value;
            }
            get
            {
                return pdu_linear_acceleration_accessor;
            }
        }
        public double[] linear_acceleration_covariance
        {
            get
            {
                return pdu.GetDataFloat64Array("linear_acceleration_covariance");
            }
        }
    }
}
