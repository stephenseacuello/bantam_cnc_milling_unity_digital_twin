using System;
using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.CNC;
using MiracleTwin.Core;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Executes a G-code program by driving the CNC controller through parsed
    /// toolpath segments. Bridges GCodeParser/GCodeInterpreter to ICNCController.
    ///
    /// Handles linear moves (G0/G1) via position lerp and arc moves (G2/G3) via
    /// angular interpolation around the arc center. Integrates with SimulationClock
    /// for time scaling and pause/resume support.
    /// </summary>
    public class GCodeExecutor : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private MonoBehaviour cncControllerObject;
        [SerializeField] private CuttingSimulationManager cuttingSimManager;

        [Header("Settings")]
        [Tooltip("Speed multiplier for G0 rapid moves relative to programmed feedrate.")]
        [SerializeField] private float rapidSpeedMultiplier = 5f;

        // Interface reference resolved at runtime
        private ICNCController cncController;

        // G-code processing
        private GCodeInterpreter interpreter = new();
        private List<ToolpathSegment> segments;
        private int currentSegmentIndex;
        private float segmentProgress; // 0..1 within current segment

        // Execution state
        public bool IsExecuting { get; private set; }
        public bool IsPausedByUser { get; private set; }
        public float Progress => segments != null && segments.Count > 0
            ? (currentSegmentIndex + segmentProgress) / segments.Count
            : 0f;
        public int CurrentGCodeLine => segments != null && currentSegmentIndex < segments.Count
            ? segments[currentSegmentIndex].gcodeLine
            : 0;
        public int TotalSegments => segments?.Count ?? 0;
        public float TotalDuration { get; private set; }
        public float ElapsedDuration { get; private set; }

        /// <summary>Read-only access to parsed toolpath segments (for preview rendering).</summary>
        public IReadOnlyList<ToolpathSegment> Segments => segments;
        /// <summary>Current segment index for progress tracking.</summary>
        public int CurrentSegmentIndex => currentSegmentIndex;

        public event Action OnExecutionComplete;
        public event Action<int> OnSegmentChanged;
        public event Action<IReadOnlyList<ToolpathSegment>> OnProgramLoaded;

        void Awake()
        {
            cncController = cncControllerObject as ICNCController;
        }

        void Start()
        {
            // Retry controller resolution if Awake ran before the controller was assigned
            if (cncController == null && cncControllerObject != null)
                cncController = cncControllerObject as ICNCController;
        }

        /// <summary>
        /// Parse and execute a G-code program string. Resets any previous execution.
        /// </summary>
        public void LoadAndExecute(string gcodeText)
        {
            if (string.IsNullOrWhiteSpace(gcodeText))
            {
                Debug.LogWarning("[GCodeExecutor] Empty program, nothing to execute.");
                return;
            }

            if (cncController == null)
            {
                Debug.LogError("[GCodeExecutor] No ICNCController assigned.");
                return;
            }

            interpreter.Reset();
            segments = interpreter.Interpret(gcodeText);

            if (segments.Count == 0)
            {
                Debug.LogWarning("[GCodeExecutor] Program produced zero toolpath segments.");
                return;
            }

            currentSegmentIndex = 0;
            segmentProgress = 0f;
            IsPausedByUser = false;
            ElapsedDuration = 0f;
            TotalDuration = GCodeInterpreter.CalculateTotalTime(segments);
            IsExecuting = true;

            // Position the CNC at the first segment's start
            cncController.SetTargetPosition(segments[0].startPos);

            Debug.Log($"[GCodeExecutor] Executing program: {segments.Count} segments, " +
                      $"estimated duration {TotalDuration:F1}s");

            OnProgramLoaded?.Invoke(segments);
        }

        /// <summary>Pause execution (user-initiated, separate from SimulationClock pause).</summary>
        public void Pause()
        {
            if (IsExecuting) IsPausedByUser = true;
        }

        /// <summary>Resume from a user-initiated pause.</summary>
        public void Resume()
        {
            IsPausedByUser = false;
        }

        /// <summary>Stop execution and reset state.</summary>
        public void Stop()
        {
            IsExecuting = false;
            IsPausedByUser = false;
            segments = null;
            currentSegmentIndex = 0;
            segmentProgress = 0f;
            ElapsedDuration = 0f;
        }

        void FixedUpdate()
        {
            if (!IsExecuting || IsPausedByUser) return;
            if (cncController == null || segments == null || segments.Count == 0) return;

            // Respect SimulationClock pause
            var clock = SimulationClock.Instance;
            if (clock != null && clock.IsPaused) return;

            // Calculate time step with clock speed scaling
            float dt = Time.fixedDeltaTime;
            if (clock != null && clock.CurrentMode == SimulationClock.Mode.Accelerated)
                dt *= clock.SpeedMultiplier;

            AdvanceExecution(dt);
        }

        private void AdvanceExecution(float dt)
        {
            while (dt > 0f && currentSegmentIndex < segments.Count)
            {
                var seg = segments[currentSegmentIndex];
                float duration = seg.Duration;

                // Apply rapid speed multiplier for G0 moves
                if (seg.type == SegmentType.Rapid)
                    duration /= rapidSpeedMultiplier;

                // Guard against zero-duration segments
                if (duration <= 0.0001f)
                {
                    // Jump instantly to end position
                    cncController.SetTargetPosition(seg.endPos);
                    AdvanceToNextSegment();
                    continue;
                }

                float remainingInSegment = (1f - segmentProgress) * duration;

                if (dt >= remainingInSegment)
                {
                    // Finish this segment
                    dt -= remainingInSegment;
                    ElapsedDuration += remainingInSegment;
                    cncController.SetTargetPosition(seg.endPos);
                    cncController.SetSpindleSpeed(seg.spindleRPM);
                    cncController.SetFeedRate(seg.feedRate);
                    AdvanceToNextSegment();
                }
                else
                {
                    // Partial advance within this segment
                    float advance = dt / duration;
                    segmentProgress += advance;
                    ElapsedDuration += dt;
                    dt = 0f;

                    Vector3 pos = InterpolateSegment(seg, segmentProgress);
                    cncController.SetTargetPosition(pos);
                    cncController.SetSpindleSpeed(seg.spindleRPM);
                    cncController.SetFeedRate(seg.feedRate);
                }
            }

            // Check if we've completed all segments
            if (currentSegmentIndex >= segments.Count)
            {
                CompleteExecution();
            }
        }

        private void AdvanceToNextSegment()
        {
            currentSegmentIndex++;
            segmentProgress = 0f;

            if (currentSegmentIndex < segments.Count)
                OnSegmentChanged?.Invoke(currentSegmentIndex);
        }

        /// <summary>
        /// Interpolate position within a toolpath segment at parameter t (0..1).
        /// Handles linear and arc segment types.
        /// </summary>
        private Vector3 InterpolateSegment(ToolpathSegment seg, float t)
        {
            t = Mathf.Clamp01(t);

            if (seg.type == SegmentType.CWArc || seg.type == SegmentType.CCWArc)
            {
                return InterpolateArc(seg, t);
            }

            // Linear interpolation for Rapid and Linear moves
            return Vector3.Lerp(seg.startPos, seg.endPos, t);
        }

        /// <summary>
        /// Interpolate along a circular arc in the XY plane.
        /// G2 = clockwise, G3 = counter-clockwise.
        /// </summary>
        private Vector3 InterpolateArc(ToolpathSegment seg, float t)
        {
            Vector3 center = seg.arcCenter;

            // Vectors from center to start/end (in XY plane)
            Vector2 startDir = new(seg.startPos.x - center.x, seg.startPos.y - center.y);
            Vector2 endDir = new(seg.endPos.x - center.x, seg.endPos.y - center.y);

            float startAngle = Mathf.Atan2(startDir.y, startDir.x);
            float endAngle = Mathf.Atan2(endDir.y, endDir.x);
            float radius = startDir.magnitude;

            // Determine sweep direction
            float sweep;
            if (seg.type == SegmentType.CWArc)
            {
                sweep = startAngle - endAngle;
                if (sweep <= 0) sweep += 2f * Mathf.PI;
                sweep = -sweep; // CW is negative direction
            }
            else
            {
                sweep = endAngle - startAngle;
                if (sweep <= 0) sweep += 2f * Mathf.PI;
            }

            float angle = startAngle + sweep * t;

            // Interpolate Z linearly (helical arcs)
            float z = Mathf.Lerp(seg.startPos.z, seg.endPos.z, t);

            return new Vector3(
                center.x + radius * Mathf.Cos(angle),
                center.y + radius * Mathf.Sin(angle),
                z
            );
        }

        private void CompleteExecution()
        {
            IsExecuting = false;
            Debug.Log($"[GCodeExecutor] Program complete. {segments.Count} segments executed, " +
                      $"elapsed {ElapsedDuration:F1}s");
            OnExecutionComplete?.Invoke();
        }

        /// <summary>
        /// Seek to a specific segment index (for scrubbing/debugging).
        /// Positions the CNC at the start of that segment.
        /// </summary>
        public void SeekToSegment(int index)
        {
            if (segments == null || index < 0 || index >= segments.Count) return;

            currentSegmentIndex = index;
            segmentProgress = 0f;
            cncController?.SetTargetPosition(segments[index].startPos);
            OnSegmentChanged?.Invoke(index);
        }
    }
}
