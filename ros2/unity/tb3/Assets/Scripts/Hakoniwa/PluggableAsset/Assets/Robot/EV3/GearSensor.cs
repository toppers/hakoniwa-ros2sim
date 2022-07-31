using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class GearSensor : MonoBehaviour
    {
        private bool isTouched = false;

        public bool IsTouched()
        {
            return this.isTouched;
        }

        private void OnTriggerEnter(Collider other)
        {
            this.isTouched = true;
            //Debug.Log("Pressed");
        }
        private void OnTriggerExit(Collider other)
        {
            this.isTouched = false;
            //Debug.Log("NotPressed");
        }
    }

}
