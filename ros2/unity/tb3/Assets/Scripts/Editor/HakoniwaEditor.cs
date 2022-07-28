using UnityEngine;
using UnityEditor;
using System.IO;
using Newtonsoft.Json;
using Hakoniwa.PluggableAsset.Assets.Environment;
using Hakoniwa.PluggableAsset;
using System;
using Newtonsoft.Json.Linq;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;

public class HakoniwaEditor : EditorWindow
{
    private static int asset_num;
    private static RosTopicMessageConfigContainer ros_topic_container;
    private static GameObject[] hako_asset_roots;
    private static GameObject[] hako_assets;
    private static LoginRobot login_robots;
    private static JArray micon_settings_json_array = new JArray();
    private static JObject micon_settings_json = new JObject();
    static void Init()
    {
        asset_num = 0;
        hako_asset_roots = null;
        hako_assets = null;
        login_robots = null;
        ros_topic_container = new RosTopicMessageConfigContainer();
        ros_topic_container.robot_num = 0;
        ros_topic_container.hakoenv_num = 0;
        micon_settings_json_array = new JArray();
        micon_settings_json = new JObject();

    }

    static string ConvertToJson(RosTopicMessageConfigContainer cfg)
    {
        return JsonConvert.SerializeObject(cfg, Formatting.Indented);
    }
    static private int GetHakoAssetRoots()
    {
        hako_asset_roots = GameObject.FindGameObjectsWithTag("HakoAssetRoot");
        int ret = 0;
        foreach (var root in hako_asset_roots)
        {
            Debug.Log("asset_root:" + root.gameObject.name);
            var hako_env_root = root.GetComponent<IHakoEnv>();
            Debug.Log("hako_env_root: " + hako_env_root);
            if ((hako_env_root != null) && hako_env_root.getRosConfig() != null)
            {
                ret += hako_env_root.getRosConfig().Length;
            }
            var hako_robo_root = root.GetComponent<IRobotParts>();
            Debug.Log("hako_robo_root: " + hako_robo_root);
            if ((hako_robo_root != null) && hako_robo_root.getRosConfig() != null)
            {
                ret += hako_robo_root.getRosConfig().Length;
            }
        }
        return ret;
    }
    static private void GetHakoAssets(int root_num)
    {
        hako_assets = GameObject.FindGameObjectsWithTag("HakoAsset");
        int len = 0;
        foreach(var e in hako_assets)
        {
            if (e.GetComponent<IHakoEnv>() != null)
            {
                Debug.Log("HakoEnv: " + e.name);
                len += e.GetComponent<IHakoEnv>().getRosConfig().Length;
            }
        }

        foreach (var root in hako_asset_roots)
        {
            var asset_parts = root.GetComponentsInChildren<IRobotParts>();
            if (asset_parts == null)
            {
                continue;
            }
            var hako_robo_root = root.GetComponent<IRobotParts>();
            foreach (var asset in asset_parts)
            {
                if (asset == hako_robo_root)
                {
                    continue;
                }
                if (asset.getRosConfig() != null)
                {
                    Debug.Log("IRobotParts: " + asset);
                    len += asset.getRosConfig().Length;
                }
            }
        }
        Debug.Log("field_num=" + (len + root_num));
        ros_topic_container.fields = new RosTopicMessageConfig[len + root_num];
    }

    static private void GetHakoAssetConfigs(GameObject root)
    {
        IHakoEnv[] hako_assets = root.GetComponentsInChildren<IHakoEnv>();
        foreach(var asset in hako_assets)
        {
            foreach (var e in asset.getRosConfig())
            {
                e.topic_message_name = root.name + "_" + asset.GetAssetName();
                e.robot_name = root.name;
                ros_topic_container.fields[asset_num] = e;
                asset_num++;
                ros_topic_container.hakoenv_num++;            }
        }
    }
    static private void GetRobotAssetConfig(GameObject root)
    {
        IRobotParts[] robot_assets = root.GetComponentsInChildren<IRobotParts>();
        if (robot_assets == null)
        {
            Debug.Log("No robots");
            return;
        }
        ros_topic_container.robot_num++;

        UnityEngine.Object prefab = PrefabUtility.GetCorrespondingObjectFromSource(root);
        var robo = new LoginRobotInfoType();
        robo.roboname = root.transform.name;
        robo.robotype = prefab.name;
        robo.pos.X = root.transform.position.x;
        robo.pos.Y = root.transform.position.y;
        robo.pos.Z = root.transform.position.z;
        robo.angle.X = root.transform.localEulerAngles.x;
        robo.angle.Y = root.transform.localEulerAngles.y;
        robo.angle.Z = root.transform.localEulerAngles.z;
        int index = login_robots.robos.Length;
        Array.Resize(ref login_robots.robos, index + 1);
        login_robots.robos[index] = robo;
        Debug.Log("roboname=" + root.transform.name);
        Debug.Log("robotype=" + prefab.name);
        Debug.Log("pos=" + root.transform.position);
        foreach (var asset in robot_assets)
        {
            Debug.Log("robot:" + asset.ToString());
            var configs = asset.getRosConfig();
            if (configs == null)
            {
                continue;
            }
            foreach ( var e in configs)
            {
                e.topic_message_name = root.name + "_" + e.topic_message_name;
                //e.topic_message_name = e.topic_message_name;
                e.robot_name = root.name;
                ros_topic_container.fields[asset_num] = e;
                asset_num++;
            }
        }
        IMiconSettings micon_settings = root.GetComponentInChildren<IMiconSettings>();
        if ((micon_settings != null) && (micon_settings.isEnabled()))
        {
            var str = micon_settings.GetSettings(robo.roboname);
            var json_data = JObject.Parse(str);
            micon_settings_json_array.Add(new JObject(json_data));
        }
    }


    [MenuItem("Window/Hakoniwa/Generate")]
    static void AssetsUpdate()
    {
        Init();
        int root_num = GetHakoAssetRoots();
        Debug.Log("assets root_num:" + root_num);
        GetHakoAssets(root_num);
        asset_num = 0;

        login_robots = new LoginRobot();
        login_robots.robos = new LoginRobotInfoType[0];
        foreach(var root in hako_asset_roots)
        {
            GetRobotAssetConfig(root);
            GetHakoAssetConfigs(root);
        }
        ros_topic_container.ros_robot_num = ros_topic_container.robot_num - micon_settings_json_array.Count;
        Debug.Log("json:" + ConvertToJson(ros_topic_container));
        try
        {
            File.Delete("../../../settings/tb3/PhotonLoginRobot.json");
        }
        catch (Exception)
        {

        }
        AssetConfigLoader.SaveJsonFile<LoginRobot>("../../../settings/tb3/LoginRobot.json", login_robots);
        File.WriteAllText("../../../settings/tb3/RosTopics.json", ConvertToJson(ros_topic_container));

        if (micon_settings_json_array.Count > 0)
        {
            micon_settings_json.Add(new JProperty("robots", micon_settings_json_array));
            File.WriteAllText("../../../settings/tb3/custom.json", micon_settings_json.ToString());
        }
        else
        {
            File.Delete("../../../settings/tb3/custom.json");
        }
    }
    [MenuItem("Window/Hakoniwa/GeneratePhoton")]
    static void PhotonAssetsUpdate()
    {
        Debug.Log("assets");
        int root_num = GetHakoAssetRoots();
        GetHakoAssets(root_num);
        asset_num = 0;

        login_robots = new LoginRobot();
        login_robots.robos = new LoginRobotInfoType[0];
        foreach (var root in hako_asset_roots)
        {
            GetRobotAssetConfig(root);
            GetHakoAssetConfigs(root);
        }
        Debug.Log("json:" + ConvertToJson(ros_topic_container));
        try
        {
            File.Delete("../../../settings/tb3/LoginRobot.json");
        }
        catch (Exception)
        {

        }
        AssetConfigLoader.SaveJsonFile<LoginRobot>("../../../settings/tb3/PhotonLoginRobot.json", login_robots);
        File.WriteAllText("../../../settings/tb3/RosTopics.json", ConvertToJson(ros_topic_container));
    }


    void OnGUI()
    {
        try
        {

        }
        catch (System.FormatException) { }
    }
}
