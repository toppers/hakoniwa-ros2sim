using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public interface IRobotPartsMotor : IRobotPartsActuator
    {
        void SetForce(int force);
        void SetTargetVelicty(float targetVelocity);
    }
}
