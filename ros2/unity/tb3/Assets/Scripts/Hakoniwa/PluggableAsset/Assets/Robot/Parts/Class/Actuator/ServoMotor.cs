using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public class ServoMotor : MonoBehaviour, IRobotPartsMotor
    {
        private GameObject root;
        public float power_const = 10;
        private float targetVelocity;
        private ArticulationBody wheel;
        [SerializeField] float forceLimit = 10f;
        [SerializeField] float damping = 2f;

        public RosTopicMessageConfig[] getRosConfig()
        {
            return null;
        }

        public void Initialize(GameObject root)
        {
            if (this.root != null)
            {
                this.SetTargetVelicty(0.0f);
            }
            else
            {
                Debug.Log("ServoMotor init");
                this.root = root;
                this.wheel = this.GetComponent<ArticulationBody>();
            }
        }

        public void SetForce(int force)
        {
            this.power_const = (float)force;
        }


        public void SetTargetVelicty(float targetVelocity)
        {
            float tmp = power_const * targetVelocity;
            this.targetVelocity = tmp;
            Drive(wheel);
        }
        private void Drive(ArticulationBody body)
        {
            if (body == null)
                return;

            ArticulationDrive drive = body.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            drive.targetVelocity = targetVelocity;
            body.xDrive = drive;
        }

    }

}
