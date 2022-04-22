using Hakoniwa.Core.Simulation;
using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets;
using Photon.Pun;
using Photon.Realtime;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Hakoniwa.Plugin.Photon
{
    public class HakoniwaPhotonPlugin : MonoBehaviourPunCallbacks
    {
        private GameObject root;
        private SimulationController simulator = SimulationController.Get();

        // Use this for initialization
        void Start()
        {
            root = GameObject.Find("Robot");
            PhotonNetwork.ConnectUsingSettings();
        }
        void OnGUI()
        {
            //ログインの状態を画面上に出力
            GUILayout.Label(PhotonNetwork.NetworkClientState.ToString());
        }
        //ルームに入室前に呼び出される
        public override void OnConnectedToMaster()
        {
            // "room"という名前のルームに参加する（ルームが無ければ作成してから参加する）
            PhotonNetwork.JoinOrCreateRoom("room", new RoomOptions(), TypedLobby.Default);
        }
        public override void OnJoinedLobby()
        {

        }
        public override void OnJoinedRoom()
        {

            Debug.Log("OnJoinedRoom!!!");
            var login_robots = AssetConfigLoader.LoadJsonFile<LoginRobot>("../../../settings/tb3/PhotonLoginRobot.json");
            foreach (var e in login_robots.robos)
            {
                this.Login(e);
            }

        }
        public void Login(LoginRobotInfoType robo)
        {

            Vector3 pos = new Vector3(robo.pos.X, robo.pos.Y, robo.pos.Z);
            var instance = PhotonNetwork.Instantiate(robo.robotype, pos, Quaternion.identity);
            instance.name = robo.roboname;
            instance.transform.parent = this.root.transform;

            IInsideAssetController ctrl = instance.GetComponentInChildren<IInsideAssetController>();
            ctrl.Initialize();
            AssetConfigLoader.AddInsideAsset(ctrl);
            simulator.RegisterInsideAsset(robo.roboname);
        }
    }
}
