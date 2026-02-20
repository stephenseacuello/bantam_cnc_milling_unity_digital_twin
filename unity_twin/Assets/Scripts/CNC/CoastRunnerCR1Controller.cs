using System;
using UnityEngine;
using RosMessageTypes.Miracle;
using MiracleTwin.Core;

namespace MiracleTwin.CNC
{
    /// <summary>
    /// Profile-driven CNC controller for the Coast Runner CR-1.
    /// Receives MachineState messages via SO event channel and drives
    /// ArticulationBody prismatic joints for X, Y, Z axes.
    ///
    /// Coordinate mapping (ROS → Unity):
    ///   ROS X → Unity X  (left-right, gantry along rails)
    ///   ROS Y → Unity Z  (front-back, table travel)
    ///   ROS Z → Unity -Y (down, spindle head descent)
    ///
    /// Motion fallback chain:
    ///   1. ArticulationBody prismatic joints (Inspector or auto-discovered)
    ///   2. Transform movableHead (Inspector or auto-discovered child)
    /// </summary>
    public class CoastRunnerCR1Controller : MonoBehaviour, ICNCController
    {
        [Header("Machine Profile")]
        [SerializeField] private CNCMachineProfileSO profile;

        [Header("Event Channel")]
        [SerializeField] private MachineStateEventSO machineStateEvent;

        [Header("Axis Joint References")]
        [SerializeField] private ArticulationBody xAxis;
        [SerializeField] private ArticulationBody yAxis;
        [SerializeField] private ArticulationBody zAxis;

        [Header("Spindle")]
        [SerializeField] private Transform spindleRotor;
        [SerializeField] private Transform endMill;

        [Header("Transform Fallback (when no ArticulationBodies)")]
        [SerializeField] private Transform movableHead;

        [Header("Motion Settings")]
        [SerializeField, Range(0.5f, 1.0f)] private float interpolationFactor = 0.8f;

        [Header("Soft Limits")]
        [SerializeField, Range(0.80f, 0.99f)] private float softLimitThreshold = 0.95f;

        [Header("E-Stop Visual")]
        [SerializeField] private Renderer[] estopFlashRenderers;
        [SerializeField] private Color estopFlashColor = new Color(1f, 0.1f, 0.1f, 1f);
        [SerializeField] private float estopFlashFrequency = 3f;

        public string MachineName => profile != null ? profile.machineName : "Unknown CNC";
        public string MachineId => profile?.rosTopicId ?? "cnc2";
        public string Status { get; private set; } = "IDLE";
        public MachineState CurrentMachineState { get; private set; } = MachineState.IDLE;
        public float SpindleRPM { get; private set; }
        public float FeedRate { get; private set; }
        public Vector3 AxisPositionsMM { get; private set; }
        public string CurrentProgram { get; private set; } = "";
        public uint CurrentLine { get; private set; }
        public float SpindleLoad { get; private set; }
        public float CoolantLevel { get; private set; }
        public double CycleTimeElapsed { get; private set; }
        public double CycleTimeRemaining { get; private set; }
        public bool IsNearSoftLimit { get; private set; }

        public Vector3 MinPositionMM => profile != null ? profile.minPosition : Vector3.zero;
        public Vector3 MaxPositionMM => profile != null ? profile.maxPosition : new Vector3(152.4f, 101.6f, 69.85f);
        public float MaxRPM => profile != null ? profile.maxRPM : 23000f;

        public bool SoftLimitX { get; private set; }
        public bool SoftLimitY { get; private set; }
        public bool SoftLimitZ { get; private set; }

        public event Action<MachineState, MachineState> OnMachineStateChanged;

        private Vector3 targetPosMM;
        private Vector3 currentPosMM;
        private Vector3 velocityMM;

        private bool useArticulationBodies;
        private Vector3 headBaseLocalPos;

        private Color[] originalColors;
        private bool estopVisualsInitialized;
        private bool wasInEStop;

        void Start()
        {
            AutoDiscoverJoints();
            useArticulationBodies = (xAxis != null || yAxis != null || zAxis != null);

            if (!useArticulationBodies && movableHead == null)
            {
                movableHead = FindBestMovableChild();
            }

            // Remove stale immovable root AB that blocks transform motion
            if (!useArticulationBodies)
            {
                var rootAB = GetComponent<ArticulationBody>();
                if (rootAB != null && rootAB.immovable)
                {
                    Debug.LogWarning($"[CoastRunnerCR1] Removing stale immovable root ArticulationBody " +
                        "that blocks transform motion. Re-run MIRACLE > Wire Dashboard to clean up.");
                    Destroy(rootAB);
                }

                var toDestroy = new System.Collections.Generic.List<GameObject>();
                foreach (var ab in GetComponentsInChildren<ArticulationBody>())
                {
                    if (ab == null || ab.gameObject == gameObject) continue;
                    if (ab.GetComponentInChildren<MeshRenderer>() == null)
                        toDestroy.Add(ab.gameObject);
                }
                foreach (var go in toDestroy)
                {
                    Debug.LogWarning($"[CoastRunnerCR1] Removing orphaned meshless AB child: {go.name}");
                    Destroy(go);
                }
            }

            if (movableHead != null)
                headBaseLocalPos = movableHead.localPosition;

            if (useArticulationBodies)
            {
                movableHead = null;
                Debug.Log($"[CoastRunnerCR1] Using ArticulationBody joints: " +
                    $"X={xAxis?.name ?? "null"}, Y={yAxis?.name ?? "null"}, Z={zAxis?.name ?? "null"}");
            }
            else if (movableHead != null)
            {
                Debug.Log($"[CoastRunnerCR1] Using Transform motion on: {movableHead.name}");
            }
            else
            {
                Debug.LogWarning("[CoastRunnerCR1] No joints or movable child found — CNC will not animate.");
            }

            InitEStopColors();
        }

        private void InitEStopColors()
        {
            if (estopFlashRenderers == null || estopFlashRenderers.Length == 0) return;
            originalColors = new Color[estopFlashRenderers.Length];
            for (int i = 0; i < estopFlashRenderers.Length; i++)
            {
                if (estopFlashRenderers[i] != null && estopFlashRenderers[i].material != null)
                    originalColors[i] = estopFlashRenderers[i].material.color;
            }
            estopVisualsInitialized = true;
        }

        private void AutoDiscoverJoints()
        {
            if (xAxis != null || yAxis != null || zAxis != null) return;

            var allBodies = GetComponentsInChildren<ArticulationBody>();
            foreach (var ab in allBodies)
            {
                if (ab.isRoot) continue;
                if (ab.jointType != ArticulationJointType.PrismaticJoint) continue;

                // Skip invisible overlay joints (no mesh in subtree)
                if (ab.GetComponentInChildren<MeshRenderer>() == null) continue;

                if (ab.linearLockX == ArticulationDofLock.LimitedMotion && xAxis == null)
                    xAxis = ab;
                else if (ab.linearLockZ == ArticulationDofLock.LimitedMotion && yAxis == null)
                    yAxis = ab;
                else if (ab.linearLockY == ArticulationDofLock.LimitedMotion && zAxis == null)
                    zAxis = ab;
            }
        }

        private Transform FindBestMovableChild()
        {
            string[] headNames = { "head", "spindle", "gantry", "carriage", "slide",
                                    "toolholder", "z_axis", "z-axis", "zaxis",
                                    "headstock", "quill", "ram", "z_head",
                                    "subassembly z" };
            // Exclude electrical/fastener components that falsely match (e.g. "2pin_header" contains "head")
            string[] excludePatterns = { "header", "pin", "screw", "bolt", "nut", "washer",
                                          "connector", "pcb", "cable", "wire", "led", "label" };

            Transform bestMatch = null;
            float bestVolume = 0f;

            foreach (Transform child in GetComponentsInChildren<Transform>())
            {
                if (child == transform) continue;
                var mr = child.GetComponentInChildren<MeshRenderer>();
                if (mr == null) continue;

                string lower = child.name.ToLowerInvariant();

                // Skip excluded parts
                bool excluded = false;
                foreach (var ex in excludePatterns)
                    if (lower.Contains(ex)) { excluded = true; break; }
                if (excluded) continue;

                foreach (var name in headNames)
                {
                    if (lower.Contains(name))
                    {
                        // Prefer largest matching child (CNC heads >> fasteners)
                        float vol = mr.bounds.size.x * mr.bounds.size.y * mr.bounds.size.z;
                        if (bestMatch == null || vol > bestVolume)
                        {
                            bestMatch = child;
                            bestVolume = vol;
                        }
                        break;
                    }
                }
            }

            if (bestMatch != null)
            {
                Debug.Log($"[CoastRunnerCR1] Found movable child by name: {bestMatch.name}");
                return bestMatch;
            }

            // Fallback: largest mesh child (likely the main body or spindle assembly)
            Transform largest = null;
            float largestVol = 0f;
            foreach (var mr in GetComponentsInChildren<MeshRenderer>())
            {
                if (mr.transform == transform) continue;
                float vol = mr.bounds.size.x * mr.bounds.size.y * mr.bounds.size.z;
                if (vol > largestVol)
                {
                    largestVol = vol;
                    largest = mr.transform;
                }
            }
            if (largest != null)
                Debug.Log($"[CoastRunnerCR1] Using largest mesh child as movable: {largest.name}");
            return largest;
        }

        void OnEnable()
        {
            if (machineStateEvent != null)
                machineStateEvent.Register(OnMachineState);
        }

        void OnDisable()
        {
            if (machineStateEvent != null)
                machineStateEvent.Unregister(OnMachineState);
        }

        private void OnMachineState(MachineStateMsg msg)
        {
            // Filter by machine_id — only process messages for this machine
            if (profile != null && !string.IsNullOrEmpty(msg.machine_id) &&
                msg.machine_id != profile.rosTopicId)
                return;

            string newStatusStr = msg.status ?? "UNKNOWN";
            MachineState newState = ParseMachineState(newStatusStr);
            MachineState oldState = CurrentMachineState;

            Status = newStatusStr;
            CurrentMachineState = newState;

            SpindleRPM = (float)msg.spindle_speed;
            FeedRate = (float)msg.feed_rate;
            SpindleLoad = (float)msg.spindle_load;
            CoolantLevel = (float)msg.coolant_level;
            CurrentProgram = msg.current_program ?? "";
            CurrentLine = msg.current_line;
            CycleTimeElapsed = msg.cycle_time_elapsed;
            CycleTimeRemaining = msg.cycle_time_remaining;

            if (msg.axis_positions != null && msg.axis_positions.Length >= 3)
            {
                float rosX = (float)msg.axis_positions[0];
                float rosY = (float)msg.axis_positions[1];
                float rosZ = (float)msg.axis_positions[2];

                targetPosMM = new Vector3(
                    Mathf.Clamp(rosX, MinPositionMM.x, MaxPositionMM.x),
                    Mathf.Clamp(rosY, MinPositionMM.y, MaxPositionMM.y),
                    Mathf.Clamp(rosZ, MinPositionMM.z, MaxPositionMM.z)
                );
            }

            if (msg.axis_velocities != null && msg.axis_velocities.Length >= 3)
            {
                velocityMM = new Vector3(
                    (float)msg.axis_velocities[0],
                    (float)msg.axis_velocities[1],
                    (float)msg.axis_velocities[2]
                );
            }

            HandleStateTransition(oldState, newState);

            if (oldState != newState)
                OnMachineStateChanged?.Invoke(oldState, newState);
        }

        private static MachineState ParseMachineState(string status)
        {
            return status?.ToUpperInvariant() switch
            {
                "IDLE" => MachineState.IDLE,
                "RUNNING" => MachineState.RUNNING,
                "PAUSED" => MachineState.PAUSED,
                "ERROR" => MachineState.ERROR,
                "ESTOP" => MachineState.ESTOP,
                _ => MachineState.UNKNOWN
            };
        }

        private void HandleStateTransition(MachineState oldState, MachineState newState)
        {
            switch (newState)
            {
                case MachineState.ESTOP:
                    SpindleRPM = 0;
                    FeedRate = 0;
                    targetPosMM = currentPosMM;
                    break;
                case MachineState.ERROR:
                    SpindleRPM = 0;
                    FeedRate = 0;
                    break;
                case MachineState.PAUSED:
                    FeedRate = 0;
                    targetPosMM = currentPosMM;
                    break;
            }
        }

        void FixedUpdate()
        {
            if (CurrentMachineState == MachineState.ESTOP ||
                CurrentMachineState == MachineState.ERROR)
            {
                AxisPositionsMM = currentPosMM;
                return;
            }

            currentPosMM = Vector3.Lerp(currentPosMM, targetPosMM, interpolationFactor);
            AxisPositionsMM = currentPosMM;

            if (useArticulationBodies)
            {
                if (xAxis != null) SetDriveTarget(xAxis, currentPosMM.x / 1000f);
                if (yAxis != null) SetDriveTarget(yAxis, currentPosMM.y / 1000f);
                if (zAxis != null) SetDriveTarget(zAxis, -currentPosMM.z / 1000f);
            }
            else if (movableHead != null)
            {
                movableHead.localPosition = headBaseLocalPos + new Vector3(
                    currentPosMM.x / 1000f,
                    -currentPosMM.z / 1000f,
                    currentPosMM.y / 1000f
                );
            }

            CheckSoftLimits();
        }

        void Update()
        {
            if (spindleRotor != null && SpindleRPM > 0 &&
                CurrentMachineState != MachineState.ESTOP &&
                CurrentMachineState != MachineState.ERROR)
            {
                float degreesPerFrame = SpindleRPM * 360f / 60f * Time.deltaTime;
                spindleRotor.Rotate(Vector3.up, degreesPerFrame, Space.Self);
            }
            UpdateEStopVisual();
        }

        private void CheckSoftLimits()
        {
            Vector3 min = MinPositionMM;
            Vector3 max = MaxPositionMM;
            Vector3 range = max - min;

            SoftLimitX = currentPosMM.x >= min.x + range.x * softLimitThreshold || currentPosMM.x <= min.x + range.x * (1f - softLimitThreshold);
            SoftLimitY = currentPosMM.y >= min.y + range.y * softLimitThreshold || currentPosMM.y <= min.y + range.y * (1f - softLimitThreshold);
            SoftLimitZ = currentPosMM.z >= min.z + range.z * softLimitThreshold || currentPosMM.z <= min.z + range.z * (1f - softLimitThreshold);
            IsNearSoftLimit = SoftLimitX || SoftLimitY || SoftLimitZ;
        }

        private void UpdateEStopVisual()
        {
            bool inEStop = CurrentMachineState == MachineState.ESTOP;
            if (!estopVisualsInitialized || originalColors == null) return;

            if (inEStop)
            {
                float t = (Mathf.Sin(Time.time * estopFlashFrequency * 2f * Mathf.PI) + 1f) * 0.5f;
                for (int i = 0; i < estopFlashRenderers.Length; i++)
                {
                    if (estopFlashRenderers[i] != null && estopFlashRenderers[i].material != null)
                        estopFlashRenderers[i].material.color = Color.Lerp(originalColors[i], estopFlashColor, t);
                }
            }
            else if (wasInEStop)
            {
                for (int i = 0; i < estopFlashRenderers.Length; i++)
                {
                    if (estopFlashRenderers[i] != null && estopFlashRenderers[i].material != null)
                        estopFlashRenderers[i].material.color = originalColors[i];
                }
            }
            wasInEStop = inEStop;
        }

        public Vector3 GetToolTipWorldPosition() => endMill != null ? endMill.position : transform.position;
        public Vector3 GetToolDirection() => endMill != null ? -endMill.up : -Vector3.up;

        public void SetTargetPosition(Vector3 positionMM)
        {
            Vector3 min = MinPositionMM;
            Vector3 max = MaxPositionMM;
            targetPosMM = new Vector3(
                Mathf.Clamp(positionMM.x, min.x, max.x),
                Mathf.Clamp(positionMM.y, min.y, max.y),
                Mathf.Clamp(positionMM.z, min.z, max.z)
            );
        }

        public void SetSpindleSpeed(float rpm) => SpindleRPM = Mathf.Clamp(rpm, 0, MaxRPM);
        public void SetFeedRate(float mmPerMin) => FeedRate = Mathf.Max(0, mmPerMin);

        public void EmergencyStop()
        {
            MachineState oldState = CurrentMachineState;
            Status = "ESTOP";
            CurrentMachineState = MachineState.ESTOP;
            SpindleRPM = 0;
            FeedRate = 0;
            targetPosMM = currentPosMM;
            if (oldState != MachineState.ESTOP)
                OnMachineStateChanged?.Invoke(oldState, MachineState.ESTOP);
        }

        public void Home()
        {
            MachineState oldState = CurrentMachineState;
            targetPosMM = MinPositionMM;
            SpindleRPM = 0;
            FeedRate = 0;
            Status = "IDLE";
            CurrentMachineState = MachineState.IDLE;
            if (oldState != MachineState.IDLE)
                OnMachineStateChanged?.Invoke(oldState, MachineState.IDLE);
        }

        private static void SetDriveTarget(ArticulationBody ab, float target)
        {
            var drive = ab.xDrive;
            drive.target = target;
            ab.xDrive = drive;
        }
    }
}
