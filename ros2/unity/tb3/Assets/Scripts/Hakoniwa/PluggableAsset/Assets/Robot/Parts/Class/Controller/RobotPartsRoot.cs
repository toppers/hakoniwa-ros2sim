using Hakoniwa.PluggableAsset.Assets;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts.Class.Controller
{
    class RobotPartsRoot : MonoBehaviour, IInsideAssetController
    {
        private GameObject root;
        private string root_name;
        private GameObject myObject;
        private IRobotPartsController[] controllers = null;
        private IRobotPartsSensor[] sensors = null;

        public void CopySensingDataToPdu()
        {
            foreach (var child in this.sensors)
            {
                if (child.isAttachedSpecificController())
                {
                    continue;
                }
                child.UpdateSensorValues();
            }
            return;
        }

        public void DoActuation()
        {
            foreach (var child in this.controllers)
            {
                child.DoControl();
            }
            return;
        }

        public string GetName()
        {
            return this.root_name;
        }

        public void Initialize()
        {
            this.root = GameObject.Find("Robot");
            this.myObject = GameObject.Find("Robot/" + this.transform.name);
            this.root_name = string.Copy(this.myObject.transform.name);
            this.controllers = this.myObject.GetComponentsInChildren<IRobotPartsController>();
            /*
             * ルートは全コントローラを初期化する
             * 
             * アクチュエータはコントローラ側の責務で初期化する
             */
            foreach (var child in this.controllers)
            {
                child.Initialize(this.myObject);
            }
            /*
             * ルートは特定のコントローラに割り当てられていないセンサを初期化する
             * 
             * 割り当てされているものは，当該コントローラ側の責務で初期化する
             */
            this.sensors = this.myObject.GetComponentsInChildren<IRobotPartsSensor>();
            foreach (var child in this.sensors)
            {
                if (child.isAttachedSpecificController())
                {
                    continue;
                }
                child.Initialize(this.myObject);
            }
        }
    }
}
