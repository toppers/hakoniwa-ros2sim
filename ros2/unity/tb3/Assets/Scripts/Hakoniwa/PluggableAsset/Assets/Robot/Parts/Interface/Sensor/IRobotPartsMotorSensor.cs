using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public interface IRobotPartsMotorSensor : IRobotPartsSensor
    {
        void ClearDegree();
        float GetCurrentAngle();
        float GetCurrentAngleVelocity();
        float GetDegree();
        float GetRadius();
    }
}
