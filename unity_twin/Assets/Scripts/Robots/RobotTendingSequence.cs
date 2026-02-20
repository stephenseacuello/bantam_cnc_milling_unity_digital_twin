using System;
using UnityEngine;
using MiracleTwin.Core;
using MiracleTwin.CNC;

namespace MiracleTwin.Robots
{
    /// <summary>
    /// State machine controlling the full machine tending cycle:
    /// Pick raw stock -> Load CNC -> Wait for machining -> Unload -> Place finished part.
    ///
    /// States:
    ///   IDLE -> APPROACH_STOCK -> PICK_RAW -> APPROACH_CNC -> OPEN_LID ->
    ///   INSERT_WORKPIECE -> CLOSE_VISE -> RETRACT -> CLOSE_LID ->
    ///   WAIT_JOB -> OPEN_LID_UNLOAD -> APPROACH_WORKPIECE -> OPEN_VISE ->
    ///   PICK_FINISHED -> RETRACT_UNLOAD -> CLOSE_LID_FINAL ->
    ///   APPROACH_FINISHED_TRAY -> PLACE_FINISHED -> RETURN_HOME -> IDLE
    /// </summary>
    public class RobotTendingSequence : MonoBehaviour
    {
        public enum TendingState
        {
            Idle,
            ApproachStockTray,
            PickRawBlock,
            ApproachCNC,
            SignalLidOpen,
            WaitLidOpen,
            InsertWorkpiece,
            SignalViseClose,
            WaitViseClose,
            RetractFromCNC,
            SignalLidClose,
            WaitJobComplete,
            SignalLidOpenUnload,
            WaitLidOpenUnload,
            ApproachWorkpiece,
            SignalViseOpen,
            WaitViseOpen,
            PickFinishedPart,
            RetractUnload,
            SignalLidCloseFinal,
            ApproachFinishedTray,
            PlaceFinishedPart,
            ReturnHome,
            Fault
        }

        [Header("References")]
        [SerializeField] private RobotController robotController;
        [SerializeField] private GripperController gripper;
        [SerializeField] private JobStatusEventSO jobStatusEvent;

        [Header("Waypoints")]
        [SerializeField] private Transform stockTrayPosition;
        [SerializeField] private Transform cncApproachPosition;
        [SerializeField] private Transform cncLoadPosition;
        [SerializeField] private Transform cncUnloadPosition;
        [SerializeField] private Transform finishedTrayPosition;
        [SerializeField] private Transform homePosition;

        [Header("Timing")]
        [SerializeField] private float waypointReachThreshold = 0.01f;
        [SerializeField] private float stateTimeout = 30f;

        public TendingState CurrentState { get; private set; } = TendingState.Idle;
        public bool IsActive => CurrentState != TendingState.Idle && CurrentState != TendingState.Fault;
        public string AssignedJobId { get; private set; }

        public event Action<TendingState> OnStateChanged;
        public event Action<string> OnFault;
        public event Action OnCycleComplete;

        private EnclosureLid enclosureLid;
        private ViseController viseController;
        private float stateEnteredTime;
        private bool jobCompleted;

        void OnEnable()
        {
            if (jobStatusEvent != null)
                jobStatusEvent.Register(OnJobStatus);
        }

        void OnDisable()
        {
            if (jobStatusEvent != null)
                jobStatusEvent.Unregister(OnJobStatus);
        }

        void Update()
        {
            if (CurrentState == TendingState.Idle || CurrentState == TendingState.Fault)
                return;

            // Timeout check
            if (Time.time - stateEnteredTime > stateTimeout)
            {
                EnterFault($"State {CurrentState} timed out after {stateTimeout}s");
                return;
            }

            UpdateStateMachine();
        }

        /// <summary>Start a machine tending cycle.</summary>
        public void StartTending(string jobId, EnclosureLid lid, ViseController vise)
        {
            if (IsActive) return;

            AssignedJobId = jobId;
            enclosureLid = lid;
            viseController = vise;
            jobCompleted = false;

            TransitionTo(TendingState.ApproachStockTray);
            Debug.Log($"[RobotTendingSequence] {robotController.RobotId} starting tending for job {jobId}");
        }

        public void Cancel()
        {
            gripper.Release();
            TransitionTo(TendingState.ReturnHome);
        }

        private void UpdateStateMachine()
        {
            switch (CurrentState)
            {
                case TendingState.ApproachStockTray:
                    if (!robotController.IsMoving)
                        TransitionTo(TendingState.PickRawBlock);
                    break;

                case TendingState.PickRawBlock:
                    if (!gripper.IsGripping)
                        gripper.Close();
                    else
                        TransitionTo(TendingState.ApproachCNC);
                    break;

                case TendingState.ApproachCNC:
                    if (!robotController.IsMoving)
                        TransitionTo(TendingState.SignalLidOpen);
                    break;

                case TendingState.SignalLidOpen:
                    enclosureLid?.Open();
                    TransitionTo(TendingState.WaitLidOpen);
                    break;

                case TendingState.WaitLidOpen:
                    if (enclosureLid == null || !enclosureLid.IsMoving)
                        TransitionTo(TendingState.InsertWorkpiece);
                    break;

                case TendingState.InsertWorkpiece:
                    if (!robotController.IsMoving)
                        TransitionTo(TendingState.SignalViseClose);
                    break;

                case TendingState.SignalViseClose:
                    gripper.Release();
                    viseController?.Close();
                    TransitionTo(TendingState.WaitViseClose);
                    break;

                case TendingState.WaitViseClose:
                    if (viseController == null || !viseController.IsMoving)
                        TransitionTo(TendingState.RetractFromCNC);
                    break;

                case TendingState.RetractFromCNC:
                    if (!robotController.IsMoving)
                        TransitionTo(TendingState.SignalLidClose);
                    break;

                case TendingState.SignalLidClose:
                    enclosureLid?.Close();
                    TransitionTo(TendingState.WaitJobComplete);
                    break;

                case TendingState.WaitJobComplete:
                    if (jobCompleted)
                        TransitionTo(TendingState.SignalLidOpenUnload);
                    break;

                case TendingState.SignalLidOpenUnload:
                    enclosureLid?.Open();
                    TransitionTo(TendingState.WaitLidOpenUnload);
                    break;

                case TendingState.WaitLidOpenUnload:
                    if (enclosureLid == null || !enclosureLid.IsMoving)
                        TransitionTo(TendingState.ApproachWorkpiece);
                    break;

                case TendingState.ApproachWorkpiece:
                    if (!robotController.IsMoving)
                        TransitionTo(TendingState.SignalViseOpen);
                    break;

                case TendingState.SignalViseOpen:
                    viseController?.Open();
                    TransitionTo(TendingState.WaitViseOpen);
                    break;

                case TendingState.WaitViseOpen:
                    if (viseController == null || !viseController.IsMoving)
                        TransitionTo(TendingState.PickFinishedPart);
                    break;

                case TendingState.PickFinishedPart:
                    if (!gripper.IsGripping)
                        gripper.Close();
                    else
                        TransitionTo(TendingState.RetractUnload);
                    break;

                case TendingState.RetractUnload:
                    if (!robotController.IsMoving)
                        TransitionTo(TendingState.SignalLidCloseFinal);
                    break;

                case TendingState.SignalLidCloseFinal:
                    enclosureLid?.Close();
                    TransitionTo(TendingState.ApproachFinishedTray);
                    break;

                case TendingState.ApproachFinishedTray:
                    if (!robotController.IsMoving)
                        TransitionTo(TendingState.PlaceFinishedPart);
                    break;

                case TendingState.PlaceFinishedPart:
                    gripper.Release();
                    TransitionTo(TendingState.ReturnHome);
                    break;

                case TendingState.ReturnHome:
                    if (!robotController.IsMoving)
                    {
                        TransitionTo(TendingState.Idle);
                        OnCycleComplete?.Invoke();
                        Debug.Log($"[RobotTendingSequence] {robotController.RobotId} tending cycle complete");
                    }
                    break;
            }
        }

        private void TransitionTo(TendingState newState)
        {
            CurrentState = newState;
            stateEnteredTime = Time.time;
            OnStateChanged?.Invoke(newState);
        }

        private void EnterFault(string reason)
        {
            Debug.LogError($"[RobotTendingSequence] {robotController.RobotId} FAULT: {reason}");
            CurrentState = TendingState.Fault;
            OnFault?.Invoke(reason);
        }

        private void OnJobStatus(RosMessageTypes.Miracle.JobStatusMsg msg)
        {
            if (msg.job_id == AssignedJobId && msg.status == "COMPLETED")
                jobCompleted = true;
        }

        public void ClearFault()
        {
            if (CurrentState == TendingState.Fault)
            {
                TransitionTo(TendingState.Idle);
                robotController.GoHome();
            }
        }
    }
}
