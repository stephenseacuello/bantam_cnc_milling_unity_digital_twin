using UnityEngine;
using RosMessageTypes.Miracle;
using MiracleTwin.Core;
using MiracleTwin.CNC;

namespace MiracleTwin.Robots
{
    /// <summary>
    /// Coordinates multiple robots based on MIRACLE L5 cognitive layer decisions.
    /// Listens to TaskAward messages and assigns tending jobs to the appropriate robot.
    /// Implements the auction-based allocation: both robots bid, cognitive layer awards to one.
    /// </summary>
    public class MultiAgentCoordinator : MonoBehaviour
    {
        [Header("Event Channels")]
        [SerializeField] private TaskAwardEventSO taskAwardEvent;

        [Header("Robots")]
        [SerializeField] private RobotTendingSequence ned2Tending;
        [SerializeField] private RobotTendingSequence xarm6Tending;

        [Header("CNC References")]
        [SerializeField] private EnclosureLid enclosureLid;
        [SerializeField] private ViseController viseController;

        public string Ned2Status => ned2Tending != null ? ned2Tending.CurrentState.ToString() : "N/A";
        public string XArm6Status => xarm6Tending != null ? xarm6Tending.CurrentState.ToString() : "N/A";
        public string LastAwardedRobot { get; private set; }
        public int TotalCyclesCompleted { get; private set; }

        void OnEnable()
        {
            if (taskAwardEvent != null)
                taskAwardEvent.Register(OnTaskAward);

            if (ned2Tending != null)
                ned2Tending.OnCycleComplete += OnCycleComplete;
            if (xarm6Tending != null)
                xarm6Tending.OnCycleComplete += OnCycleComplete;

            if (ned2Tending != null)
                ned2Tending.OnFault += OnRobotFault;
            if (xarm6Tending != null)
                xarm6Tending.OnFault += OnRobotFault;
        }

        void OnDisable()
        {
            if (taskAwardEvent != null)
                taskAwardEvent.Unregister(OnTaskAward);

            if (ned2Tending != null)
            {
                ned2Tending.OnCycleComplete -= OnCycleComplete;
                ned2Tending.OnFault -= OnRobotFault;
            }
            if (xarm6Tending != null)
            {
                xarm6Tending.OnCycleComplete -= OnCycleComplete;
                xarm6Tending.OnFault -= OnRobotFault;
            }
        }

        private void OnTaskAward(TaskAwardMsg msg)
        {
            if (msg.task_type != "MACHINE_TEND") return;

            string assignedRobot = msg.awarded_agent_id;
            string jobId = msg.job_id;

            Debug.Log($"[MultiAgentCoordinator] Task {jobId} awarded to {assignedRobot}");
            LastAwardedRobot = assignedRobot;

            RobotTendingSequence targetRobot = null;
            RobotTendingSequence fallbackRobot = null;

            if (assignedRobot == "ned2" || assignedRobot == "niryo_ned2")
            {
                targetRobot = ned2Tending;
                fallbackRobot = xarm6Tending;
            }
            else if (assignedRobot == "xarm6" || assignedRobot == "xarm6_lite")
            {
                targetRobot = xarm6Tending;
                fallbackRobot = ned2Tending;
            }

            // Try primary assignment
            if (targetRobot != null && !targetRobot.IsActive)
            {
                targetRobot.StartTending(jobId, enclosureLid, viseController);
                return;
            }

            // Fallback to other robot if primary is busy/faulted
            if (fallbackRobot != null && !fallbackRobot.IsActive)
            {
                Debug.LogWarning($"[MultiAgentCoordinator] {assignedRobot} unavailable, using fallback");
                fallbackRobot.StartTending(jobId, enclosureLid, viseController);
                return;
            }

            Debug.LogWarning($"[MultiAgentCoordinator] No available robot for task {jobId}");
        }

        private void OnCycleComplete()
        {
            TotalCyclesCompleted++;
            Debug.Log($"[MultiAgentCoordinator] Total cycles completed: {TotalCyclesCompleted}");
        }

        private void OnRobotFault(string reason)
        {
            Debug.LogError($"[MultiAgentCoordinator] Robot fault: {reason}");
            // In production, would re-publish task for re-auction
        }

        /// <summary>Manually trigger a tending cycle for testing.</summary>
        public void ManualTend(string robotId, string jobId = "manual_test")
        {
            var msg = new TaskAwardMsg
            {
                job_id = jobId,
                task_type = "MACHINE_TEND",
                awarded_agent_id = robotId
            };
            OnTaskAward(msg);
        }
    }
}
