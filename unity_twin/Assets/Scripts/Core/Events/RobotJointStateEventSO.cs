using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>Robot joint state data for 6-DOF arms.</summary>
    [System.Serializable]
    public struct RobotJointStateData
    {
        public string robotId;
        public double[] positions;
        public double[] velocities;
        public string gripperState;
    }

    [CreateAssetMenu(menuName = "MIRACLE/Events/Robot Joint State")]
    public class RobotJointStateEventSO : GameEventSO<RobotJointStateData> { }
}
