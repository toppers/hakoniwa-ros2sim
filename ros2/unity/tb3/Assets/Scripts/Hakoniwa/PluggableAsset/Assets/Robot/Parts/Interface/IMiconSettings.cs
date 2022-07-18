using System;
namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public interface IMiconSettings
    {
        bool isEnabled();
        string GetSettings(string name);
    }

}

