using Hakoniwa.PluggableAsset.Assets.Robot.TB3;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TurtleBot3Parts : MonoBehaviour, ITB3Parts
{
    private TB3PartsLoader loader = new TB3PartsLoader();

    private string[] motors = new string[2] {
        "base_footprint/imu_link/wheel_right_link/Wheel",
        "base_footprint/imu_link/wheel_left_link/Wheel"
    };

    public string GetIMU()
    {
        return "base_footprint/imu_link";
    }

    public string GetLaserScan()
    {
        return "base_footprint/imu_link/base_link/base_scan/Scan";
    }
    public string GetCamera()
    {
        return "base_footprint/imu_link/Body/CameraBody/CameraCase";
    }

    public string GetMotor(int index)
    {
        return motors[index];
    }

    string ITB3Parts.GetMotor(int index, out int update_scale)
    {
        update_scale = 1;
        //Debug.Log("this.loader=" + this.loader);
        if (this.loader.GetMotor(index, out update_scale) != null)
        {
            return this.loader.GetMotor(index, out update_scale);
        }
        return motors[index];
    }

    string ITB3Parts.GetIMU(out int update_scale)
    {
        update_scale = 1;
        if (this.loader.GetIMU(out update_scale) != null)
        {
            return this.loader.GetIMU(out update_scale);
        }
        return "base_footprint/imu_link";
    }

    string ITB3Parts.GetLaserScan(out int update_scale)
    {
        update_scale = 1;
        if (this.loader.GetLaserScan(out update_scale) != null)
        {
            return this.loader.GetLaserScan(out update_scale);
        }
        return "base_footprint/imu_link/base_link/base_scan/Scan";
    }

    string ITB3Parts.GetCamera(out int update_scale)
    {
        update_scale = 1;
        if (this.loader.GetCamera(out update_scale) != null)
        {
            return this.loader.GetCamera(out update_scale);
        }
        return "base_footprint/imu_link/Body/CameraBody/CameraCase";
    }

    public void Load()
    {
        this.loader.Load();
        return;
    }
}
