using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public interface IRobotPartsTouchSensor : IRobotPartsSensor
    {
        bool IsPressed();
    }
}
