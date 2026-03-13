using System;
using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.CNC;
using MiracleTwin.Core;
using MiracleTwin.UI;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Executes a G-code program by driving the CNC controller through parsed
    /// toolpath segments. Bridges GCodeParser/GCodeInterpreter to ICNCController.
    ///
    /// Handles linear moves (G0/G1) via position lerp and arc moves (G2/G3) via
    /// angular interpolation around the arc center. Integrates with SimulationClock
    /// for time scaling and pause/resume support.
    ///
    /// Supports a dry-run preview mode that simulates the entire toolpath without
    /// affecting the physical machine, reporting predicted forces, wear, and
    /// collision risks via the OnPreviewComplete event.
    /// </summary>
    public class GCodeExecutor : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private MonoBehaviour cncControllerObject;
        [SerializeField] private CuttingSimulationManager cuttingSimManager;
        [SerializeField] private CNCMachineProfileSO machineProfile;

        [Header("Settings")]
        [Tooltip("Speed multiplier for G0 rapid moves relative to programmed feedrate.")]
        [SerializeField] private float rapidSpeedMultiplier = 5f;

        [Header("Security")]
        [Tooltip("When true, refuse to execute G-code with Invalid signatures.")]
        [SerializeField] private bool requireSignedGCode = false;
        [Tooltip("Public key PEM for signature verification (TextAsset).")]
        [SerializeField] private TextAsset signaturePublicKey;

        [Header("Chart Integration")]
        [SerializeField] private ForceChart forceChart;

        [Header("Lookahead")]
        [SerializeField] private GCodeLookahead lookahead;
        [SerializeField] private float lookaheadInterval = 0.5f;
        [SerializeField] private bool pauseOnLookaheadWarning = false;

        [Header("Collision Response")]
        [Tooltip("Auto-pause execution on FixtureCollision or OutOfEnvelope severity.")]
        [SerializeField] private bool autoPauseOnCollision = true;

        [Header("Preview Mode")]
        [Tooltip("When true, execution simulates the toolpath without sending commands to the CNC controller.")]
        [SerializeField] private bool previewMode = false;

        /// <summary>Whether the executor is currently in dry-run preview mode.</summary>
        public bool IsPreviewMode => previewMode;

        /// <summary>Fired when a dry-run preview completes with the full analysis report.</summary>
        public event Action<PreviewReport> OnPreviewComplete;

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
        private float lastLookaheadTime;

        /// <summary>Read-only access to parsed toolpath segments (for preview rendering).</summary>
        public IReadOnlyList<ToolpathSegment> Segments => segments;
        /// <summary>Current segment index for progress tracking.</summary>
        public int CurrentSegmentIndex => currentSegmentIndex;

        public event Action OnExecutionComplete;
        public event Action<int> OnSegmentChanged;
        public event Action<IReadOnlyList<ToolpathSegment>> OnProgramLoaded;
        public event Action<IReadOnlyList<LookaheadResult>> OnLookaheadUpdated;

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

            // Signature verification
            var sigResult = GCodeSignatureVerifier.Verify(
                gcodeText,
                signaturePublicKey != null ? signaturePublicKey.text : null
            );

            if (sigResult == SignatureResult.Invalid)
            {
                Debug.LogError("[GCodeExecutor] G-code signature is INVALID — execution aborted.");
                return;
            }

            if (sigResult == SignatureResult.Missing && requireSignedGCode)
            {
                Debug.LogError("[GCodeExecutor] G-code is unsigned and requireSignedGCode is enabled — execution aborted.");
                return;
            }

            if (sigResult == SignatureResult.Missing)
            {
                Debug.LogWarning("[GCodeExecutor] G-code is unsigned. Set requireSignedGCode=true to enforce signing.");
            }

            interpreter.Reset();
            segments = interpreter.Interpret(gcodeText);

            if (segments.Count == 0)
            {
                Debug.LogWarning("[GCodeExecutor] Program produced zero toolpath segments.");
                return;
            }

            // Validate against machine limits
            var validation = GCodeValidator.ValidateProgram(segments, machineProfile);
            foreach (var warn in validation.Warnings)
                Debug.LogWarning($"[GCodeValidator] {warn}");
            foreach (var err in validation.Errors)
                Debug.LogError($"[GCodeValidator] {err}");
            if (!validation.IsValid)
            {
                Debug.LogError("[GCodeExecutor] Program failed validation — execution aborted.");
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
                    // In preview mode, skip CNC controller commands
                    if (!previewMode)
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

                    // In preview mode, skip CNC controller commands
                    if (!previewMode)
                    {
                        cncController.SetTargetPosition(seg.endPos);
                        cncController.SetSpindleSpeed(seg.spindleRPM);
                        cncController.SetFeedRate(seg.feedRate);
                    }
                    AdvanceToNextSegment();
                }
                else
                {
                    // Partial advance within this segment
                    float advance = dt / duration;
                    segmentProgress += advance;
                    ElapsedDuration += dt;
                    dt = 0f;

                    // In preview mode, skip CNC controller commands
                    if (!previewMode)
                    {
                        Vector3 pos = InterpolateSegment(seg, segmentProgress);
                        cncController.SetTargetPosition(pos);
                        cncController.SetSpindleSpeed(seg.spindleRPM);
                        cncController.SetFeedRate(seg.feedRate);
                    }
                }
            }

            // Trigger lookahead analysis periodically
            if (lookahead != null && segments != null && currentSegmentIndex < segments.Count)
            {
                if (Time.time - lastLookaheadTime > lookaheadInterval)
                {
                    lastLookaheadTime = Time.time;
                    var results = lookahead.RunLookahead(segments, currentSegmentIndex);
                    OnLookaheadUpdated?.Invoke(results);

                    // Forward anomaly annotations to the force chart
                    UpdateForceChartAnnotations(results);

                    // Log and react to collision detections by severity
                    bool shouldPauseForCollision = false;
                    foreach (var r in results)
                    {
                        if (!r.collisionFlag) continue;

                        switch (r.collisionSeverity)
                        {
                            case FixtureProfile.CollisionSeverity.OutOfEnvelope:
                                Debug.LogError($"[GCodeExecutor] COLLISION: Segment {r.segmentIndex} — tool outside work envelope.");
                                shouldPauseForCollision = true;
                                break;
                            case FixtureProfile.CollisionSeverity.FixtureCollision:
                                Debug.LogError($"[GCodeExecutor] COLLISION: Segment {r.segmentIndex} — tool tip hits fixture.");
                                shouldPauseForCollision = true;
                                break;
                            case FixtureProfile.CollisionSeverity.ShankCollision:
                                Debug.LogWarning($"[GCodeExecutor] COLLISION: Segment {r.segmentIndex} — tool shank hits fixture.");
                                shouldPauseForCollision = true;
                                break;
                            case FixtureProfile.CollisionSeverity.NearMiss:
                                Debug.LogWarning($"[GCodeExecutor] Near-miss at segment {r.segmentIndex} — tool within 2x clearance of fixture.");
                                break;
                        }
                    }

                    if (autoPauseOnCollision && shouldPauseForCollision)
                    {
                        Debug.LogError("[GCodeExecutor] Fixture collision detected — auto-pausing execution.");
                        IsPausedByUser = true;
                    }
                    else if (pauseOnLookaheadWarning && lookahead.HasWarnings)
                    {
                        Debug.LogWarning("[GCodeExecutor] Lookahead detected warnings — pausing execution.");
                        IsPausedByUser = true;
                    }
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

        /// <summary>
        /// Forward anomaly markers from lookahead/prediction results to the force chart.
        /// Converts LookaheadResult entries that exceed thresholds into AnomalyAnnotation
        /// instances and pushes them to the ForceChart for visual overlay.
        /// </summary>
        public void UpdateForceChartAnnotations(IReadOnlyList<LookaheadResult> results)
        {
            if (forceChart == null || results == null) return;

            var annotations = new List<AnomalyAnnotation>();

            foreach (var r in results)
            {
                // Force critical/warning
                if (r.peakForceN >= 180f)
                {
                    annotations.Add(new AnomalyAnnotation
                    {
                        blockIndex = r.segmentIndex,
                        markerType = "FORCE_CRITICAL",
                        severity = Mathf.Clamp01(r.peakForceN / 250f),
                        predictedValue = r.peakForceN,
                        threshold = 180f,
                        recommendation = $"Reduce feed rate by {Mathf.RoundToInt((r.peakForceN - 180f) / 180f * 100f)}%"
                    });
                }
                else if (r.peakForceN >= 150f)
                {
                    annotations.Add(new AnomalyAnnotation
                    {
                        blockIndex = r.segmentIndex,
                        markerType = "FORCE_WARNING",
                        severity = Mathf.Clamp01(r.peakForceN / 250f),
                        predictedValue = r.peakForceN,
                        threshold = 150f,
                        recommendation = "Monitor force levels closely"
                    });
                }

                // Chatter risk
                if (r.chatterRiskScore >= 0.7f)
                {
                    annotations.Add(new AnomalyAnnotation
                    {
                        blockIndex = r.segmentIndex,
                        markerType = "CHATTER_HIGH",
                        severity = r.chatterRiskScore,
                        predictedValue = r.chatterRiskScore,
                        threshold = 0.7f,
                        recommendation = "Adjust spindle speed to avoid resonance"
                    });
                }

                // Collision
                if (r.collisionFlag)
                {
                    annotations.Add(new AnomalyAnnotation
                    {
                        blockIndex = r.segmentIndex,
                        markerType = "TOOL_COLLISION",
                        severity = 1.0f,
                        predictedValue = 1.0f,
                        threshold = 0.0f,
                        recommendation = "Verify toolpath clearance at this block"
                    });
                }

                // Wear contribution
                if (r.cumulativeWearMM >= 0.25f)
                {
                    annotations.Add(new AnomalyAnnotation
                    {
                        blockIndex = r.segmentIndex,
                        markerType = "WEAR_CRITICAL",
                        severity = Mathf.Clamp01(r.cumulativeWearMM / 0.30f),
                        predictedValue = r.cumulativeWearMM,
                        threshold = 0.30f,
                        recommendation = "Schedule tool change before VB_max exceeded"
                    });
                }
            }

            forceChart.SetAnomalyAnnotations(annotations);
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

        // ── Preview / Dry-Run Mode ──────────────────────────────────────────

        /// <summary>
        /// Enter preview mode: parses and validates the G-code, then runs
        /// a full dry-run analysis over every block without sending any
        /// commands to the CNC controller or modifying the workpiece.
        /// </summary>
        public void EnterPreviewMode(string gcodeText)
        {
            if (string.IsNullOrWhiteSpace(gcodeText))
            {
                Debug.LogWarning("[GCodeExecutor] Empty program, nothing to preview.");
                return;
            }

            // Signature verification
            var sigResult = GCodeSignatureVerifier.Verify(
                gcodeText,
                signaturePublicKey != null ? signaturePublicKey.text : null
            );

            if (sigResult == SignatureResult.Invalid)
            {
                Debug.LogError("[GCodeExecutor] G-code signature is INVALID — preview aborted.");
                return;
            }

            if (sigResult == SignatureResult.Missing && requireSignedGCode)
            {
                Debug.LogError("[GCodeExecutor] G-code is unsigned and requireSignedGCode is enabled — preview aborted.");
                return;
            }

            interpreter.Reset();
            segments = interpreter.Interpret(gcodeText);

            if (segments.Count == 0)
            {
                Debug.LogWarning("[GCodeExecutor] Program produced zero toolpath segments — nothing to preview.");
                return;
            }

            // Validate against machine limits
            var validation = GCodeValidator.ValidateProgram(segments, machineProfile);
            foreach (var warn in validation.Warnings)
                Debug.LogWarning($"[GCodeValidator] {warn}");
            foreach (var err in validation.Errors)
                Debug.LogError($"[GCodeValidator] {err}");

            previewMode = true;
            currentSegmentIndex = 0;
            segmentProgress = 0f;

            OnProgramLoaded?.Invoke(segments);
            RunPreview();
        }

        /// <summary>
        /// Exit preview mode and clear all preview data.
        /// </summary>
        public void ExitPreviewMode()
        {
            previewMode = false;
            segments = null;
            currentSegmentIndex = 0;
            segmentProgress = 0f;
            IsExecuting = false;
            ElapsedDuration = 0f;
        }

        /// <summary>
        /// Run a complete dry-run analysis over ALL segments using the
        /// lookahead engine. Accumulates per-segment predictions for force,
        /// power, wear, collision, and chatter risk, then fires OnPreviewComplete
        /// with the full report.
        /// </summary>
        public void RunPreview()
        {
            if (segments == null || segments.Count == 0)
            {
                Debug.LogWarning("[GCodeExecutor] No segments loaded — cannot run preview.");
                return;
            }

            var report = new PreviewReport
            {
                totalBlocks = segments.Count,
                segments = new List<PreviewReport.PreviewSegment>(segments.Count)
            };

            // Run full lookahead over ALL blocks (not the default 50)
            List<LookaheadResult> lookaheadResults = null;
            if (lookahead != null)
            {
                lookaheadResults = lookahead.RunLookahead(segments, 0, segments.Count);
            }

            float cumulativeWear = 0f;
            float estimatedTime = 0f;
            float peakForce = 0f;
            float peakPower = 0f;
            int collisionCount = 0;
            int chatterHigh = 0;
            int chatterMedium = 0;

            for (int i = 0; i < segments.Count; i++)
            {
                var seg = segments[i];
                var previewSeg = new PreviewReport.PreviewSegment
                {
                    blockIndex = i,
                    startPos = seg.startPos,
                    endPos = seg.endPos
                };

                // Accumulate time from feed rate and distance
                float segLength = Vector3.Distance(seg.startPos, seg.endPos);
                if (seg.type == SegmentType.Rapid)
                {
                    // Rapid moves use a high feed rate estimate
                    float rapidFeed = seg.feedRate > 0 ? seg.feedRate * rapidSpeedMultiplier : 10000f;
                    estimatedTime += (segLength / rapidFeed) * 60f;
                }
                else if (seg.feedRate > 0.01f)
                {
                    estimatedTime += (segLength / seg.feedRate) * 60f;
                }

                // Pull predictions from the lookahead results
                if (lookaheadResults != null && i < lookaheadResults.Count)
                {
                    var la = lookaheadResults[i];
                    previewSeg.forceN = la.peakForceN;
                    previewSeg.powerW = la.powerW;
                    previewSeg.collisionSeverity = la.collisionSeverity;

                    // Wear contribution is the delta from previous cumulative wear
                    float wearDelta = la.cumulativeWearMM - cumulativeWear;
                    previewSeg.wearContribution = Mathf.Max(0f, wearDelta);
                    cumulativeWear = la.cumulativeWearMM;

                    // Track peaks
                    if (la.peakForceN > peakForce) peakForce = la.peakForceN;
                    if (la.powerW > peakPower) peakPower = la.powerW;

                    // Collision count
                    if (la.collisionFlag) collisionCount++;

                    // Chatter risk classification
                    if (la.chatterRiskScore >= 0.7f)
                    {
                        previewSeg.chatterRisk = "HIGH";
                        chatterHigh++;
                    }
                    else if (la.chatterRiskScore >= 0.4f)
                    {
                        previewSeg.chatterRisk = "MEDIUM";
                        chatterMedium++;
                    }
                    else
                    {
                        previewSeg.chatterRisk = "LOW";
                    }

                    // Assign risk color based on combined risk assessment
                    previewSeg.riskColor = DetermineRiskColor(la);
                }
                else
                {
                    previewSeg.chatterRisk = "LOW";
                    previewSeg.riskColor = Color.green;
                }

                report.segments.Add(previewSeg);
            }

            report.estimatedTimeSeconds = estimatedTime;
            report.peakForceN = peakForce;
            report.peakPowerW = peakPower;
            report.totalWearMM = cumulativeWear;
            report.collisionCount = collisionCount;
            report.chatterRiskHighCount = chatterHigh;
            report.chatterRiskMediumCount = chatterMedium;

            // Estimate remaining tool life from total wear
            const float VBmax = 0.30f;
            float remainingWearBudget = VBmax - cumulativeWear;
            if (remainingWearBudget > 0 && estimatedTime > 0 && cumulativeWear > 0)
                report.estimatedToolLifeMin = (remainingWearBudget / cumulativeWear) * (estimatedTime / 60f);
            else if (cumulativeWear <= 0)
                report.estimatedToolLifeMin = float.MaxValue;
            else
                report.estimatedToolLifeMin = 0f;

            Debug.Log($"[GCodeExecutor] Preview complete: {report.totalBlocks} blocks, " +
                      $"est. {report.estimatedTimeSeconds:F1}s, {report.collisionCount} collisions, " +
                      $"peak force {report.peakForceN:F1}N");

            OnPreviewComplete?.Invoke(report);
        }

        /// <summary>
        /// Determine a risk color for a segment based on collision, chatter, and force levels.
        /// Green = safe, Yellow = medium risk, Red = collision or high chatter risk.
        /// </summary>
        private Color DetermineRiskColor(LookaheadResult la)
        {
            // Red: collision detected or high chatter risk
            if (la.collisionFlag)
                return Color.red;
            if (la.chatterRiskScore >= 0.7f)
                return Color.red;

            // Yellow: medium chatter risk, near-miss, or elevated force
            if (la.chatterRiskScore >= 0.4f)
                return Color.yellow;
            if (la.collisionSeverity == FixtureProfile.CollisionSeverity.NearMiss)
                return Color.yellow;

            // Green: safe
            return Color.green;
        }

        /// <summary>
        /// Full report produced by a dry-run preview analysis.
        /// Contains aggregate statistics and per-segment predictions.
        /// </summary>
        [Serializable]
        public class PreviewReport
        {
            public int totalBlocks;
            public float estimatedTimeSeconds;
            public float peakForceN;
            public float peakPowerW;
            public float totalWearMM;
            public float estimatedToolLifeMin;
            public int collisionCount;
            public int chatterRiskHighCount;
            public int chatterRiskMediumCount;
            public List<PreviewSegment> segments;

            [Serializable]
            public class PreviewSegment
            {
                public int blockIndex;
                public Vector3 startPos;
                public Vector3 endPos;
                public float forceN;
                public float powerW;
                public float wearContribution;
                public FixtureProfile.CollisionSeverity collisionSeverity;
                public string chatterRisk; // "LOW", "MEDIUM", "HIGH"
                public Color riskColor;    // Green=safe, Yellow=warning, Red=danger
            }
        }
    }
}
