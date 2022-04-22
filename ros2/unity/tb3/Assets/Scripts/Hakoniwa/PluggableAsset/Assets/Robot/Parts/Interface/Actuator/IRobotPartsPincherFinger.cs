using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public interface IRobotPartsPincherFinger : IRobotPartsActuator
    {
        float CurrentGrip();
        Vector3 GetOpenPosition();

        void UpdateGrip(float grip);
        void ForceOpen();
    }
}

