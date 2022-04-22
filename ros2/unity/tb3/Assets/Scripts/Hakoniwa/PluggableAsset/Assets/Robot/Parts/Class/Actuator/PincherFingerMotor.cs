using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public class PincherFingerMotor : MonoBehaviour, IRobotPartsPincherFinger
    {
        public float relativeClosedPos;
        public float target_grip;

        private Vector3 openPosition;

        private GameObject root;
        private ArticulationBody articulation_body;


        public RosTopicMessageConfig[] getRosConfig()
        {
            return null;
        }

        public void Initialize(GameObject root)
        {
            if (this.root != null)
            {
                this.UpdateGrip(0.0f);
            }
            else
            {
                Debug.Log("PincherFingerMotor init");
                this.root = root;
                this.articulation_body = this.GetComponent<ArticulationBody>();
            }
            openPosition = transform.localPosition;
            SetLimits();
        }

        public void UpdateGrip(float grip)
        {
            float targetPos = DriveTarget(grip);
            var drive = articulation_body.xDrive;
            drive.target = targetPos;
            articulation_body.xDrive = drive;
        }
        public void ForceOpen()
        {
            UpdateGrip(0f);
        }

        private void SetLimits()
        {
            float openTarget = DriveTarget(0.0f);
            float closedTarget = DriveTarget(1.0f);
            float min = Mathf.Min(openTarget, closedTarget);
            float max = Mathf.Max(openTarget, closedTarget);

            var drive = articulation_body.xDrive;
            drive.lowerLimit = min;
            drive.upperLimit = max;
            articulation_body.xDrive = drive;
        }
        private float DriveTarget(float grip)
        {
            float pos = Mathf.Lerp(openPosition.x, openPosition.x + relativeClosedPos, grip);
            float targetPos = (pos - openPosition.x) * transform.parent.localScale.x;
            return targetPos;
        }

        public float CurrentGrip()
        {
            float grip = Mathf.InverseLerp(openPosition.x, openPosition.x + relativeClosedPos, transform.localPosition.x);
            return grip;
        }

        public Vector3 GetOpenPosition()
        {
            return openPosition;
        }
    }
}
