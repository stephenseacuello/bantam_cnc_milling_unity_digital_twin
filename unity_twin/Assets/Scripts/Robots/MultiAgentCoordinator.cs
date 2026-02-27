using System.Collections.Generic;
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

        private Queue<TaskAwardMsg> pendingTasks = new Queue<TaskAwardMsg>();

        public string Ned2Status => ned2Tending != null ? ned2Tending.CurrentState.ToString() : "N/A";
        public string XArm6Status => xarm6Tending != null ? xarm6Tending.CurrentState.ToString() : "N/A";
        public string LastAwardedRobot { get; private set; }
        public int TotalCyclesCompleted { get; private set; }
        public int PendingTaskCount => pendingTasks.Count;

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

        void Update()
        {
            // Process pending task queue when robots become available
            if (pendingTasks.Count > 0)
            {
                bool ned2Available = ned2Tending != null && !ned2Tending.IsActive
                    && ned2Tending.CurrentState != RobotTendingSequence.TendingState.Fault;
                bool xarm6Available = xarm6Tending != null && !xarm6Tending.IsActive
                    && xarm6Tending.CurrentState != RobotTendingSequence.TendingState.Fault;

                if (ned2Available || xarm6Available)
                {
                    TaskAwardMsg queuedTask = pendingTasks.Dequeue();
                    Debug.Log($"[MultiAgentCoordinator] Processing queued task {queuedTask.job_id}");
                    DispatchTask(queuedTask);
                }
            }
        }

        private void OnTaskAward(TaskAwardMsg msg)
        {
            if (msg.task_type != "MACHINE_TEND") return;
            DispatchTask(msg);
        }

        private void DispatchTask(TaskAwardMsg msg)
        {
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

            // No robot available — queue the task for later processing
            pendingTasks.Enqueue(msg);
            Debug.LogWarning($"[MultiAgentCoordinator] No available robot for task {jobId} — queued (pending: {pendingTasks.Count})");
        }

        private void OnCycleComplete()
        {
            TotalCyclesCompleted++;
            Debug.Log($"[MultiAgentCoordinator] Total cycles completed: {TotalCyclesCompleted}");
        }

        private void OnRobotFault(string reason)
        {
            Debug.LogError($"[MultiAgentCoordinator] Robot fault: {reason}");

            // Re-queue the faulted robot's assigned task so another robot can pick it up
            RobotTendingSequence faultedRobot = null;
            if (ned2Tending != null && ned2Tending.CurrentState == RobotTendingSequence.TendingState.Fault)
                faultedRobot = ned2Tending;
            else if (xarm6Tending != null && xarm6Tending.CurrentState == RobotTendingSequence.TendingState.Fault)
                faultedRobot = xarm6Tending;

            if (faultedRobot != null && !string.IsNullOrEmpty(faultedRobot.AssignedJobId))
            {
                var requeueMsg = new TaskAwardMsg
                {
                    job_id = faultedRobot.AssignedJobId,
                    task_type = "MACHINE_TEND",
                    // Do not specify a robot — let DispatchTask find an available one
                    awarded_agent_id = ""
                };
                pendingTasks.Enqueue(requeueMsg);
                Debug.LogWarning($"[MultiAgentCoordinator] Re-queued task {faultedRobot.AssignedJobId} from faulted robot (pending: {pendingTasks.Count})");
            }
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
