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
using System.Text;
using System.Runtime.InteropServices;

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
        public long GetDeltaTimeUsec()
        {
            return (long)(Time.fixedDeltaTime * 1000000.0f);
        }
    }

    public class WorldController : MonoBehaviour
    {
        private GameObject root;
        public long maxDelayTime = 20000; /* usec */
        private static ISimulationController isim;
        private static ISimulationAssetManager iasset;

        public static ISimulationController Get()
        {
            return isim;
        }
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
            if (AssetConfigLoader.core_config.cpp_mode != null) {
                Debug.Log("cpp_mode:" + AssetConfigLoader.core_config.cpp_mode);
                Debug.Log("cpp_asset_name:" + AssetConfigLoader.core_config.cpp_asset_name);
                isim = SimulationControllerFactory.Get(AssetConfigLoader.core_config.cpp_asset_name);
            }
            else {
                Debug.Log("cpp_mode: None");
                isim = SimulationControllerFactory.Get(null);
                RpcServer.StartServer(AssetConfigLoader.core_config.core_ipaddr, AssetConfigLoader.core_config.core_portno);
                SimulationController.Get().SetSimulationWorldTime(
                    this.maxDelayTime,
                    (long)(Time.fixedDeltaTime * 1000000f));
            }
            Debug.Log("HakoniwaCore START");

            iasset = isim.GetAssetManager();
            isim.RegisterEnvironmentOperation(new UnityEnvironmentOperation());
            isim.SaveEnvironment();

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
            isim.SetInsideWorldSimulator(new UnitySimulator());
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
            instance.transform.Rotate(new Vector3(robo.angle.X, robo.angle.Y, robo.angle.Z));

            IInsideAssetController ctrl = instance.GetComponentInChildren<IInsideAssetController>();
            ctrl.Initialize();
            AssetConfigLoader.AddInsideAsset(ctrl);
            iasset.RegisterInsideAsset(robo.roboname);
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
                isim.Execute();
            }
            catch (Exception e)
            {
                SimpleLogger.Get().Log(Level.ERROR, e);
                throw e;
            }
        }
        void OnApplicationQuit()
        {
            if (AssetConfigLoader.core_config.cpp_mode != null) {
                Debug.Log("cpp_mode:" + AssetConfigLoader.core_config.cpp_mode);
                Debug.Log("OnApplicationQuit:enter");
                HakoCppWrapper.asset_unregister(new StringBuilder(AssetConfigLoader.core_config.cpp_asset_name));
            }
        }
    }
}
