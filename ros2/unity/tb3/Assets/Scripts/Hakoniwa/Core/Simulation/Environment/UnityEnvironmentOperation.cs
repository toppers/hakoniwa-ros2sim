using Hakoniwa.GUI;
using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace Hakoniwa.Core.Simulation.Environment
{
    class UnityEnvironmentOperation : IEnvironmentOperation
    {
        private GameObject root;
        private SimStart event_button;

        public UnityEnvironmentOperation()
        {
            this.root = GameObject.Find("Robot");
            var obj = GameObject.Find("StartButton");
            event_button = obj.GetComponentInChildren<SimStart>();
        }
        private Rigidbody[] rigidbodies;
        private ArticulationBody[] articbodies;
        private Vector3[] initial_pos;
        private Quaternion[] initial_angle;
        private bool[] initial_kinematics;
        private bool[] initial_collitions;

        public void Save()
        {
            this.rigidbodies = GameObject.Find("Hakoniwa").GetComponentsInChildren<Rigidbody>();
            this.articbodies = GameObject.Find("Hakoniwa").GetComponentsInChildren<ArticulationBody>();
            this.initial_pos = new Vector3[this.rigidbodies.Length + this.articbodies.Length];
            this.initial_angle = new Quaternion[this.rigidbodies.Length + this.articbodies.Length];
            this.initial_kinematics = new bool[this.rigidbodies.Length + this.articbodies.Length];
            this.initial_collitions = new bool[this.rigidbodies.Length + this.articbodies.Length];
            int i = 0;
            foreach (var rigidbody in rigidbodies)
            {
                Vector3 tmp = rigidbody.transform.position;
                Quaternion angle = rigidbody.transform.rotation;
                this.initial_pos[i] = new Vector3(tmp.x, tmp.y, tmp.z);
                this.initial_angle[i] = new Quaternion(angle.x, angle.y, angle.z, angle.w);
                this.initial_collitions[i] = rigidbody.detectCollisions;
                this.initial_kinematics[i] = rigidbody.isKinematic;
                i++;
            }
            foreach (var articbody in articbodies)
            {
                Vector3 tmp = articbody.transform.position;
                Quaternion angle = articbody.transform.rotation;
                this.initial_pos[i] = new Vector3(tmp.x, tmp.y, tmp.z);
                this.initial_angle[i] = new Quaternion(angle.x, angle.y, angle.z, angle.w);
                //this.initial_kinematics[i] = articbody.immovable;
                i++;
            }
        }
        public void Restore()
        {

            //set iskinematic true
            foreach (var rigidbody in this.rigidbodies)
            {
                //Debug.Log("rigidbody=" + rigidbody + "isKinematic=" + rigidbody.isKinematic);
                rigidbody.isKinematic = true;
                rigidbody.detectCollisions = false;
            }
            foreach (var articbody in articbodies)
            {
                //articbody.immovable = true;
            }
            int i = 0;
            foreach (var rigidbody in rigidbodies)
            {
                rigidbody.transform.position = this.initial_pos[i];
                rigidbody.transform.rotation = this.initial_angle[i];
                i++;
            }
            foreach (var articulationBody in articbodies)
            {
                if (articulationBody.isRoot)
                {
                    articulationBody.transform.position = this.initial_pos[i];
                    articulationBody.transform.rotation = this.initial_angle[i];
                    articulationBody.velocity = Vector3.zero;
                    articulationBody.angularVelocity = Vector3.zero;
                    articulationBody.TeleportRoot(this.initial_pos[i], this.initial_angle[i]);
                    i++;
                }
            }
            i = 0;
            foreach (var rigidbody in rigidbodies)
            {
                rigidbody.isKinematic = this.initial_kinematics[i];
                rigidbody.detectCollisions = this.initial_collitions[i];
                i++;
            }
            foreach (var articbody in articbodies)
            {
                articbody.immovable = this.initial_kinematics[i];
                i++;
            }
            foreach (Transform child in this.root.transform)
            {
                Debug.Log("reset child=" + child.name);
                GameObject obj = root.transform.Find(child.name).gameObject;
                IInsideAssetController ctrl = obj.GetComponentInChildren<IInsideAssetController>();
                ctrl.Initialize();
            }

            this.event_button.ResetEvent();
        }

    }
}
