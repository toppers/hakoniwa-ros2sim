using Hakoniwa.Core.Rpc;
using Hakoniwa.Core.Simulation;
using Hakoniwa.Core.Simulation.Environment;
using Hakoniwa.Core.Utils;
using Hakoniwa.Core.Utils.Logger;
using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Hakoniwa.Core
{
    class UnitySimulator : IInsideWorldSimulatior
    {
        public UnitySimulator()
        {

        }
        public void DoSimulation()
        {
            Physics.Simulate(Time.fixedDeltaTime);
        }
    }

    public class WorldController : MonoBehaviour
    {
        private GameObject root;
        public long maxDelayTime = 20000; /* usec */
        private SimulationController simulator = SimulationController.Get();
        private void InitHakoniwa()
        {
            this.root = GameObject.Find("Robot");
#if UNITY_EDITOR
            string filePath = Directory.GetCurrentDirectory();
#else
            string filePath = AppDomain.CurrentDomain.BaseDirectory;
#endif
            Debug.Log(filePath);
            string configPath = filePath + System.IO.Path.DirectorySeparatorChar + "core_config.json";

            AssetConfigLoader.Load(configPath);
            Debug.Log("HakoniwaCore START");
            RpcServer.StartServer(AssetConfigLoader.core_config.core_ipaddr, AssetConfigLoader.core_config.core_portno);
            simulator.RegisterEnvironmentOperation(new UnityEnvironmentOperation());
            simulator.SaveEnvironment();
            simulator.GetLogger().SetFilePath(AssetConfigLoader.core_config.SymTimeMeasureFilePath);

#if false
            //ログインタイミングでインスタンス化する．
            foreach (Transform child in this.root.transform)
            {
                Debug.Log("child=" + child.name);
                GameObject obj = root.transform.Find(child.name).gameObject;
                IInsideAssetController ctrl = obj.GetComponentInChildren<IInsideAssetController>();
                ctrl.Initialize();
                AssetConfigLoader.AddInsideAsset(ctrl);
                simulator.RegisterInsideAsset(child.name);
            }
#endif
#if false
            LoginRobotInfoType info = new LoginRobotInfoType();
            info.roboname = "TB3RoboModel";
            info.robotype = "TB3RoboModel";
            info.pos.X = 0;
            info.pos.Y = 0;
            info.pos.Z = 0;
            this.Login(info);
#else
            try
            {
                var login_robots = AssetConfigLoader.LoadJsonFile<LoginRobot>("../../../settings/tb3/PhotonLoginRobot.json");
                Debug.Log("Photon Mode");

            } catch (Exception)
            {
                var login_robots = AssetConfigLoader.LoadJsonFile<LoginRobot>("../../../settings/tb3/LoginRobot.json");
                foreach (var e in login_robots.robos)
                {
                    this.Login(e);
                }
            }
#endif
            simulator.SetSimulationWorldTime(
                this.maxDelayTime,
                (long)(Time.fixedDeltaTime * 1000000f));
            simulator.SetInsideWorldSimulator(new UnitySimulator());
            Physics.autoSimulation = false;
        }
        public void Login(LoginRobotInfoType robo)
        {
            string path = "Hakoniwa/Robots/" + robo.robotype;
            var p = Resources.Load<GameObject>(path);
            if (p == null)
            {
                throw new InvalidDataException("ERROR: path is not found:" + path);
            }
            Vector3 pos = new Vector3(robo.pos.X, robo.pos.Y, robo.pos.Z);
            var instance = Instantiate(p, pos, Quaternion.identity) as GameObject;
            instance.name = robo.roboname;
            instance.transform.parent = this.root.transform;

            IInsideAssetController ctrl = instance.GetComponentInChildren<IInsideAssetController>();
            ctrl.Initialize();
            AssetConfigLoader.AddInsideAsset(ctrl);
            simulator.RegisterInsideAsset(robo.roboname);
        }
        void Start()
        {
            try
            {
                this.InitHakoniwa();
            }
            catch (Exception e)
            {
                SimpleLogger.Get().Log(Level.ERROR, e);
                throw e;
            }
        }
        void FixedUpdate()
        {
            try
            {
                this.simulator.Execute();
            }
            catch (Exception e)
            {
                SimpleLogger.Get().Log(Level.ERROR, e);
                throw e;
            }
        }
    }
}
