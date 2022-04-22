using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{
    public class TouchSensor : MonoBehaviour, IRobotTouchSensor, IPointerDownHandler, IPointerUpHandler
    {
        private GameObject touchSensor;
        public bool isTouched;

        public void Initialize(System.Object root)
        {
            touchSensor = (GameObject)root;
            this.isTouched = false;
        }
        public bool IsPressed()
        {
            return this.isTouched;
        }

        public void UpdateSensorValues()
        {
            //nothing to do
        }
        private void OnTriggerStay(Collider other)
        {
            this.isTouched = true;
            //Debug.Log("Pressed");
        }
        private void OnTriggerExit(Collider other)
        {
            this.isTouched = false;
            //Debug.Log("NotPressed");
        }


        void IPointerDownHandler.OnPointerDown(PointerEventData eventData)
        {
            this.isTouched = true;
            //Debug.Log("Pressed");
        }


        void IPointerUpHandler.OnPointerUp(PointerEventData eventData)
        {
            this.isTouched = false;
            //Debug.Log("NotPressed");
        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            throw new System.NotImplementedException();
        }
    }
}
