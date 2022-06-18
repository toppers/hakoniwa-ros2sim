using Hakoniwa.Core;
using Hakoniwa.Core.Simulation;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Hakoniwa.GUI
{
    public class SimTime : MonoBehaviour
    {
        WorldController root;
        GameObject myObject;
        Text simTimeText;
        // Start is called before the first frame update
        void Start()
        {
            this.root = GameObject.Find("Hakoniwa").GetComponent<WorldController>();
            this.myObject = GameObject.Find("GUI/Canvas/SimTime/Value");
            simTimeText = myObject.GetComponent<Text>();
        }

        // Update is called once per frame
        void Update()
        {
            ISimulationController simulator = WorldController.Get();
            long simtime = simulator.GetWorldTime();
            double t = ((double)simtime) / 1000000.0f;
            if (simtime <= 1)
            {
                simTimeText.text = "0.000";
            }
            else
            {
                long tl = (long)(t * 1000);
                t = (double)tl / 1000;
                simTimeText.text = t.ToString();
            }
        }
    }
}
