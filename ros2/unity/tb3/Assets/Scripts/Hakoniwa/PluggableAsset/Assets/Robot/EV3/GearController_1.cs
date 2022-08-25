using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class GearController_1 : MonoBehaviour
    {
        public GameObject mover;
        public float rotation_angle_per_sec = 1f;
        private float rotation_angle_per_fixupdate = 1f;
        private bool isMovingMode = false;
        private float targetAngle = 90;
        private float currentAngle = 0;
        private GearSensor[] sensors;

        private void Start()
        {
            this.rotation_angle_per_fixupdate = rotation_angle_per_sec / Time.fixedDeltaTime;
            this.sensors = this.GetComponentsInChildren<GearSensor>();
            //Debug.Log("start Gear Control: sensor num=" + this.sensors.Length);
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            if (this.isMovingMode)
            {
                //Debug.Log("obj=" + this.name);
                //Debug.Log("angle=" + this.currentAngle);
                this.mover.transform.Rotate(new Vector3(0, -rotation_angle_per_fixupdate, 0));
                this.currentAngle += rotation_angle_per_fixupdate;
                if (this.currentAngle >= this.targetAngle)
                {
                    this.currentAngle = 0;
                    this.isMovingMode = false;
                }
            }
            else
            {
                foreach (var e in this.sensors)
                {
                    if (e.IsTouched())
                    {
                        this.isMovingMode = true;
                        break;
                    }
                }
            }

        }

    }

}
