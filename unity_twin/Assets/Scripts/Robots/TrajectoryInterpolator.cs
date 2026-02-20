using UnityEngine;

namespace MiracleTwin.Robots
{
    /// <summary>
    /// Provides smooth joint trajectory interpolation between waypoints.
    /// Uses quintic polynomial for jerk-limited motion profiles.
    /// </summary>
    public class TrajectoryInterpolator
    {
        private float[] startJoints;
        private float[] endJoints;
        private float duration;
        private float elapsed;
        private bool isActive;

        public bool IsComplete => elapsed >= duration;
        public float Progress => duration > 0 ? Mathf.Clamp01(elapsed / duration) : 1f;

        public TrajectoryInterpolator(int dof = 6)
        {
            startJoints = new float[dof];
            endJoints = new float[dof];
        }

        /// <summary>Start a new trajectory segment.</summary>
        public void SetTarget(float[] current, float[] target, float durationSec)
        {
            for (int i = 0; i < startJoints.Length && i < current.Length; i++)
                startJoints[i] = current[i];
            for (int i = 0; i < endJoints.Length && i < target.Length; i++)
                endJoints[i] = target[i];

            duration = Mathf.Max(0.01f, durationSec);
            elapsed = 0f;
            isActive = true;
        }

        /// <summary>Evaluate the trajectory at the current time.</summary>
        public float[] Evaluate(float dt)
        {
            if (!isActive) return startJoints;

            elapsed += dt;
            float t = Mathf.Clamp01(elapsed / duration);

            // Quintic polynomial: 6t^5 - 15t^4 + 10t^3 (zero velocity/acceleration at boundaries)
            float s = t * t * t * (t * (t * 6f - 15f) + 10f);

            float[] result = new float[startJoints.Length];
            for (int i = 0; i < result.Length; i++)
                result[i] = Mathf.Lerp(startJoints[i], endJoints[i], s);

            if (elapsed >= duration)
                isActive = false;

            return result;
        }

        public void Cancel()
        {
            isActive = false;
        }
    }
}
