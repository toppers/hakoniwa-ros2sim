using Hakoniwa.Core.Utils;
using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets.Robot;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.Accessor;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Assets.Scripts.Hakoniwa.PluggableAsset.Assets.Robot.TB3
{
    class IMUSensorArticulationBody : MonoBehaviour, IRobotPartsSensor
    {
        private GameObject root;
        private GameObject sensor;
        private string root_name;
        private PduIoConnector pdu_io;
        private IPduWriter pdu_imu;
        private IPduWriter pdu_odometry;
        private IPduWriter pdu_tf;
        private OdometryAccessor pdu_odometry_accessor;
        private ParamScale scale;
        private const int tf_num = 1;
        private long current_timestamp;

        private float deltaTime;
        private Vector3 prev_velocity = Vector3.zero;
        private ArticulationBody mybody;
        private Vector3 prev_angle = Vector3.zero;
        private Vector3 delta_angle = Vector3.zero;

        public void Initialize(GameObject root)
        {
            if (this.root == null)
            {
                this.scale = AssetConfigLoader.GetScale();
                IRobotVector3 pos = this.GetPosition();
                this.init_pos_unity = new Vector3(pos.x, pos.y, pos.z);
                this.root = root;
                this.root_name = string.Copy(this.root.transform.name);
                this.pdu_io = PduIoConnector.Get(root_name);
                if (this.pdu_io == null)
                {
                    throw new ArgumentException("can not found pdu_io:" + root_name);
                }
                this.sensor = this.gameObject;
                this.mybody = this.sensor.GetComponentInChildren<ArticulationBody>();
                this.deltaTime = Time.fixedDeltaTime;
                this.pdu_imu = this.pdu_io.GetWriter(this.root_name + "_imuPdu");
                if (this.pdu_imu == null)
                {
                    throw new ArgumentException("can not found Imu pdu:" + this.root_name + "_imuPdu");
                }
                this.pdu_odometry = this.pdu_io.GetWriter(this.root_name + "_odomPdu");
                if (this.pdu_odometry == null)
                {
                    throw new ArgumentException("can not found Imu pdu:" + this.root_name + "_odomPdu");
                }
                this.pdu_tf = this.pdu_io.GetWriter(this.root_name + "_tfPdu");
                if (this.pdu_tf == null)
                {
                    throw new ArgumentException("can not found Tf pdu:" + this.root_name + "_tfPdu");
                }
                this.pdu_tf.GetWriteOps().InitializePduArray("transforms", tf_num);


            }
        }
        private void UpdateOrientation(Pdu pdu)
        {
            pdu.Ref("orientation").SetData("x", (double)this.sensor.transform.rotation.z);
            pdu.Ref("orientation").SetData("y", (double)-this.sensor.transform.rotation.x);
            pdu.Ref("orientation").SetData("z", (double)this.sensor.transform.rotation.y);
            pdu.Ref("orientation").SetData("w", (double)-this.sensor.transform.rotation.w);
        }
        private void UpdateAngularVelocity(Pdu pdu)
        {
            pdu.Ref("angular_velocity").SetData("x", (double)mybody.angularVelocity.z);
            pdu.Ref("angular_velocity").SetData("y", (double)-mybody.angularVelocity.x);
            pdu.Ref("angular_velocity").SetData("z", (double)mybody.angularVelocity.y);
        }

        private Vector3 GetCurrentEulerAngle()
        {
            return this.sensor.transform.rotation.eulerAngles;
        }
        private Quaternion GetCurrentAngle()
        {
            return this.sensor.transform.rotation;
        }
        private Quaternion GetCurrentLocalAngle()
        {
            return this.sensor.transform.localRotation;
        }

        private Vector3 GetDeltaEulerAngle()
        {
            return this.delta_angle;
        }

        private void UpdateLinearAcceleration(Pdu pdu)
        {
            Vector3 current_velocity = this.sensor.transform.InverseTransformDirection(mybody.velocity);
            Vector3 acceleration = (current_velocity - prev_velocity) / deltaTime;
            this.prev_velocity = current_velocity;
            this.delta_angle = this.GetCurrentEulerAngle() - prev_angle;
            this.prev_angle = this.GetCurrentEulerAngle();
            //gravity element
            acceleration += transform.InverseTransformDirection(Physics.gravity);

            pdu.Ref("linear_acceleration").SetData("x", (double)acceleration.z);
            pdu.Ref("linear_acceleration").SetData("y", (double)-acceleration.x);
            pdu.Ref("linear_acceleration").SetData("z", (double)acceleration.y);
        }

        private void UpdateSensorData(Pdu pdu)
        {
            TimeStamp.Set(pdu);
            pdu.Ref("header").SetData("frame_id", "imu_link");

            //orientation
            UpdateOrientation(pdu);

            //angular_velocity
            UpdateAngularVelocity(pdu);

            //linear_acceleration
            UpdateLinearAcceleration(pdu);
        }


        public void UpdateSensorValues()
        {
            this.current_timestamp = UtilTime.GetUnixTime();
            //Debug.Log("count_len=" + count.Length);
            //Debug.Log("update_cycle_len=" + update_cycle.Length);
            for (int i = 0; i < count.Length; i++)
            {
                this.count[i]++;
                if (this.count[i] < this.update_cycle[i])
                {
                    continue;
                }
                this.count[i] = 0;
                if (i == 0)
                {
                    //IMUSensor
                    this.UpdateSensorData(pdu_imu.GetWriteOps().Ref(null));
                }
                else if (i == 1)
                {
                    //Odometry
                    this.CalcOdometry();
                }
                else
                {
                    //Tf
                    this.PublishTf();
                }
            }
        }


        private IRobotVector3 GetPosition()
        {
            IRobotVector3 pos = new IRobotVector3();
            pos.x = this.transform.position.x;
            pos.y = this.transform.position.y;
            pos.x = this.transform.position.z;
            return pos;
        }


        public string[] topic_type = {
            "sensor_msgs/Imu",
            "nav_msgs/Odometry",
            "tf2_msgs/TFMessage"
        };
        public string[] topic_name = {
            "imu",
            "odom",
            "tf"
        };
        public int[] update_cycle = {
            10,
            10,
            10
        };
        private int[] count = {
            0,
            0,
            0
        };

        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[topic_type.Length];
            int i = 0;
            for (i = 0; i < topic_type.Length; i++)
            {
                cfg[i] = new RosTopicMessageConfig();
                cfg[i].topic_message_name = this.topic_name[i];
                cfg[i].topic_type_name = this.topic_type[i];
                cfg[i].sub = false;
                cfg[i].pub_option = new RostopicPublisherOption();
                cfg[i].pub_option.cycle_scale = this.update_cycle[i];
                cfg[i].pub_option.latch = false;
                cfg[i].pub_option.queue_size = 1;
            }

            return cfg;
        }
        public bool isAttachedSpecificController()
        {
            return false;
        }

        private void SetTfPos(Pdu pdu_tf, Vector3 pos)
        {
            pdu_tf.GetPduWriteOps().Ref("translation").SetData("x", (double)pos.x);
            pdu_tf.GetPduWriteOps().Ref("translation").SetData("y", (double)pos.y);
            pdu_tf.GetPduWriteOps().Ref("translation").SetData("z", (double)pos.z);
        }
        private void SetTfPosUnity(Pdu pdu_tf, Vector3 pos_unity)
        {
            pdu_tf.GetPduWriteOps().Ref("translation").SetData("x", (double)pos_unity.z);
            pdu_tf.GetPduWriteOps().Ref("translation").SetData("y", (double)-pos_unity.x);
            pdu_tf.GetPduWriteOps().Ref("translation").SetData("z", (double)pos_unity.y);
        }
        private void SetTfPos(Pdu pdu_tf, IPduWriter pdu_odm_pos)
        {
            pdu_tf.GetPduWriteOps().Ref("translation").SetData("x", pdu_odm_pos.GetReadOps().Ref("pose").Ref("pose").Ref("position").GetDataFloat64("x"));
            pdu_tf.GetPduWriteOps().Ref("translation").SetData("y", pdu_odm_pos.GetReadOps().Ref("pose").Ref("pose").Ref("position").GetDataFloat64("y"));
            pdu_tf.GetPduWriteOps().Ref("translation").SetData("z", pdu_odm_pos.GetReadOps().Ref("pose").Ref("pose").Ref("position").GetDataFloat64("z"));
        }
        private void SetTfOrientationUnity(Pdu pdu_tf, Quaternion orientation_unity)
        {
            pdu_tf.GetPduWriteOps().SetData("x", (double)orientation_unity.z);
            pdu_tf.GetPduWriteOps().SetData("y", (double)-orientation_unity.x);
            pdu_tf.GetPduWriteOps().SetData("z", (double)orientation_unity.y);
            pdu_tf.GetPduWriteOps().SetData("w", (double)-orientation_unity.w);
        }
        private void SetTfOrientation(Pdu pdu_tf, Quaternion orientation)
        {
            pdu_tf.GetPduWriteOps().SetData("x", (double)orientation.x);
            pdu_tf.GetPduWriteOps().SetData("y", (double)orientation.y);
            pdu_tf.GetPduWriteOps().SetData("z", (double)orientation.z);
            pdu_tf.GetPduWriteOps().SetData("w", (double)orientation.w);
        }
        private void PublishTf()
        {
            long t = this.current_timestamp;

            Pdu[] tf_pdus = this.pdu_tf.GetWriteOps().Refs("transforms");
            TimeStamp.Set(t, tf_pdus[0]);
            tf_pdus[0].GetPduWriteOps().Ref("header").SetData("frame_id", "odom");
            tf_pdus[0].GetPduWriteOps().SetData("child_frame_id", "base_footprint");
            SetTfPos(tf_pdus[0].Ref("transform"), this.pdu_odometry);
            tf_pdus[0].Ref("transform").SetData("rotation",
                this.pdu_imu.GetReadOps().Ref("orientation"));
        }

        private Vector3 init_pos_unity = Vector3.zero;
        private Vector3 current_pos = Vector3.zero; //ROS 
        private Vector3 prev_pos_unity = Vector3.zero;
        private Vector3 current_pos_unity = Vector3.zero;
        private void CalcOdometry()
        {
            IRobotVector3 pos = this.GetPosition();
            this.current_pos_unity = new Vector3(pos.x, pos.y, pos.z);
            Vector3 delta_pos_unity = this.current_pos_unity - this.prev_pos_unity;
            Vector3 delta_pos = Vector3.zero;

            delta_pos.x = delta_pos_unity.z / 100.0f;
            delta_pos.y = -delta_pos_unity.x / 100.0f;

            this.prev_pos_unity = this.current_pos_unity;

            //Debug.Log("delta_s=" + delta_s + " dx=" + delta_pos.x + " dy=" + delta_pos.y + " angle.y=" + unity_current_angle.y);

            current_pos.x = (this.current_pos_unity.z - this.init_pos_unity.z) / 100.0f;
            current_pos.y = -(this.current_pos_unity.x - this.init_pos_unity.x) / 100.0f;
            //Debug.Log("curr=" + current_pos_unity + " init=" + init_pos_unity + " imu_angle=" + this.imu.transform.rotation.eulerAngles.y);
            //Debug.Log("curr_pos_ros=" + current_pos);
            //Debug.Log("body.x=" + this.transform.position.x + " body.y=" + this.transform.position.y + " body.z=" + this.transform.position.z);

            Vector3 delta_angle = Vector3.zero;
            var unity_delta_angle = this.GetDeltaEulerAngle();//degree, Unity
            delta_angle.x = 0f;
            delta_angle.y = 0f;
            delta_angle.z = unity_delta_angle.y * Mathf.Deg2Rad;

            /********
             * SCALE
             ********/
            //pos
            current_pos.x = current_pos.x * this.scale.odom;
            current_pos.y = current_pos.y * this.scale.odom;
            current_pos.z = current_pos.z * this.scale.odom;

            //delta_pos
            delta_pos.x = delta_pos.x * this.scale.odom;
            delta_pos.y = delta_pos.y * this.scale.odom;
            delta_pos.z = delta_pos.z * this.scale.odom;

            /*
             * PDU
             */
            //header
            TimeStamp.Set(this.pdu_odometry.GetWriteOps().Ref(null));
            this.pdu_odometry.GetWriteOps().Ref("header").SetData("frame_id", "odom");

            //child_frame_id
            this.pdu_odometry.GetWriteOps().SetData("child_frame_id", "base_footprint");
            //pose.pose.position
            this.pdu_odometry.GetWriteOps().Ref("pose").Ref("pose").Ref("position").SetData("x", (double)current_pos.x);
            this.pdu_odometry.GetWriteOps().Ref("pose").Ref("pose").Ref("position").SetData("y", (double)current_pos.y);
            this.pdu_odometry.GetWriteOps().Ref("pose").Ref("pose").Ref("position").SetData("z", (double)current_pos.z);

            //pose.pose.orientation
            this.pdu_odometry.GetWriteOps().Ref("pose").Ref("pose").SetData("orientation",
                this.pdu_imu.GetReadOps().Ref("orientation"));

            //twist.twist.linear
            this.pdu_odometry.GetWriteOps().Ref("twist").Ref("twist").Ref("linear").SetData("x", (double)delta_pos.x / Time.fixedDeltaTime);
            this.pdu_odometry.GetWriteOps().Ref("twist").Ref("twist").Ref("linear").SetData("y", (double)delta_pos.y / Time.fixedDeltaTime);
            this.pdu_odometry.GetWriteOps().Ref("twist").Ref("twist").Ref("linear").SetData("z", (double)delta_pos.z / Time.fixedDeltaTime);
            //twist.twist.angular
            this.pdu_odometry.GetWriteOps().Ref("twist").Ref("twist").Ref("angular").SetData("x", (double)delta_angle.x / Time.fixedDeltaTime);
            this.pdu_odometry.GetWriteOps().Ref("twist").Ref("twist").Ref("angular").SetData("y", (double)delta_angle.y / Time.fixedDeltaTime);
            this.pdu_odometry.GetWriteOps().Ref("twist").Ref("twist").Ref("angular").SetData("z", (double)delta_angle.z / Time.fixedDeltaTime);
        }

    }
}
