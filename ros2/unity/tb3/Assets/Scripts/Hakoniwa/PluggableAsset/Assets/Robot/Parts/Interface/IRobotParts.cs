using Hakoniwa.PluggableAsset;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public interface IRobotParts
    {
        void Initialize(GameObject root);
        RosTopicMessageConfig[] getRosConfig();
    }
}
