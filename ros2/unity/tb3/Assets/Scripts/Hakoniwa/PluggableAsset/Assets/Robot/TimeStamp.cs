using Hakoniwa.Core.Utils;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Assets.Robot
{
    class TimeStamp
    {
        public static void Set(Pdu pdu)
        {
            long t = UtilTime.GetUnixTime();
            int t_sec = (int)((long)(t / 1000000));
            uint t_nsec = (uint)((long)(t % 1000000)) * 1000;
            try
            {
                pdu.Ref("header").Ref("stamp").SetData("sec", t_sec);
            } catch (ArgumentException)
            {
                pdu.Ref("header").Ref("stamp").SetData("sec", (uint)t_sec);
            }
            pdu.Ref("header").Ref("stamp").SetData("nanosec", t_nsec);
        }
        public static void Set(long t, Pdu pdu)
        {
            int t_sec = (int)((long)(t / 1000000));
            uint t_nsec = (uint)((long)(t % 1000000)) * 1000;
            try
            {
                pdu.Ref("header").Ref("stamp").SetData("sec", t_sec);
            }
            catch (ArgumentException)
            {
                pdu.Ref("header").Ref("stamp").SetData("sec", (uint)t_sec);
            }
            pdu.Ref("header").Ref("stamp").SetData("nanosec", t_nsec);
        }
    }
}
