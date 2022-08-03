using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class Motor : MonoBehaviour, IRobotPartsController, IRobotPartsSensor
    {
        public int motorNo;
        public int powerConst = 10;
        public int motorPower = 100;

        private IPduReader pdu_reader;
        private IPduWriter pdu_writer;
        private string root_name;
        private PduIoConnector pdu_io;

        private GameObject obj;
        private HingeJoint joint;
        private JointMotor motor;
        private int targetVelocity;
        private int force;
        private bool isStop;
        private Quaternion prevRotation;
        private float deg;
        private Rigidbody rigid_body;

        public void Initialize(GameObject root)
        {
            if (this.obj != null)
            {
                this.SetTargetVelicty(0);
                this.deg = 0.0f;
                return;
            }
            this.obj = root;
            this.root_name = string.Copy(this.obj.transform.name);
            this.pdu_io = PduIoConnector.Get(this.root_name);
            this.pdu_reader = this.pdu_io.GetReader(this.root_name + "_ev3_actuatorPdu");
            if (this.pdu_reader == null)
            {
                throw new ArgumentException("can not found _ev3_actuator pdu:" + this.root_name + "_ev3_actuatorPdu");
            }
            this.pdu_writer = this.pdu_io.GetWriter(this.root_name + "_ev3_sensorPdu");
            if (this.pdu_writer == null)
            {
                throw new ArgumentException("can not found ev3_sensor pdu:" + this.root_name + "_ev3_sensorPdu");
            }

            this.rigid_body = this.GetComponent<Rigidbody>();
            this.isStop = false;
            this.joint = this.GetComponent<HingeJoint>();
            this.prevRotation = this.transform.localRotation;
            this.SetForce(this.motorPower);
        }

        public void SetForce(int force)
        {
            this.force = force;
            this.motor.force = force;
            this.joint.motor = this.motor;
        }
        public void SetStop(bool stop)
        {
            if (stop)
            {
                if (this.isStop == false)
                {
                    this.motor.force = 0;
                    this.motor.targetVelocity = 0;
                    this.joint.motor = this.motor;
                    this.rigid_body.drag = 10F;
                    this.rigid_body.angularDrag = 10F;
                    this.isStop = true;
                    Debug.Log("set stop");
                }
            }
            else
            {
                if (isStop == true)
                {
                    this.rigid_body.drag = 0F;
                    this.rigid_body.angularDrag = 0.05F;
                    this.motor.force = this.force;
                    this.motor.targetVelocity = this.targetVelocity;
                    this.joint.motor = this.motor;
                    this.isStop = false;
                    Debug.Log("released stop");
                }
            }
        }

        public void SetTargetVelicty(int targetVelocity)
        {
            this.targetVelocity = targetVelocity;
            this.motor.targetVelocity = targetVelocity;
            this.joint.motor = this.motor;
        }
        public void ClearDegree()
        {
            this.deg = 0.0f;
        }
        public float GetDegree()
        {
            return this.deg;
        }

        private float Map360To180(float degree)
        {
            if (degree < 180.0f)
            {
                return degree;
            }

            return degree - 360.0f;
        }
        public void UpdateSensorValuesLocal()
        {
            float diff;
            var diff_rot = this.transform.localRotation * Quaternion.Inverse(this.prevRotation);
            diff = Map360To180(diff_rot.eulerAngles.x);
            this.prevRotation = this.transform.localRotation;
            this.deg += diff;

        }
        public void UpdateSensorValues()
        {
            this.UpdateSensorValuesLocal();

            var motor_angle = (uint)this.GetDegree();
            uint[] motor_angles = this.pdu_writer.GetReadOps().GetDataUInt32Array("motor_angle");
            motor_angles[this.motorNo] = motor_angle;
            this.pdu_writer.GetWriteOps().SetData("motor_angle", motor_angles);
        }

        public void SetTargetVelicty(float targetVelocity)
        {
            throw new System.NotImplementedException();
        }

        public float GetCurrentAngle()
        {
            throw new System.NotImplementedException();
        }

        public float GetCurrentAngleVelocity()
        {
            throw new System.NotImplementedException();
        }

        public float GetRadius()
        {
            throw new System.NotImplementedException();
        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            return null;
        }

        public void DoControl()
        {
            var power = this.pdu_reader.GetReadOps().Refs("motors")[this.motorNo].GetDataInt32("power");
            this.SetTargetVelicty(power * this.powerConst);
        }


        public bool isAttachedSpecificController()
        {
            return false;
        }
    }
}

