using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.EV3
{

    public class GpsSensor : MonoBehaviour, IRobotGpsSensor
    {
        private GameObject mypos;
        private Vector3 pos;

        public void Initialize(System.Object root)
        {
            if (this.mypos != null)
            {
                return;
            }
            this.mypos = (GameObject)root;
            return;
        }
        public double GeLatitude()
        {
            return this.pos.x;
        }

        public double GetLongitude()
        {
            return this.pos.z;
        }

        public int GetStatus()
        {
            return 0;
        }


        public void UpdateSensorValues()
        {
            pos = new Vector3(this.mypos.transform.position.x, this.mypos.transform.position.y, this.mypos.transform.position.z);
            //Debug.Log("x=" + pos.x + " y=" + pos.y);
            return;
        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            throw new System.NotImplementedException();
        }
    }
}