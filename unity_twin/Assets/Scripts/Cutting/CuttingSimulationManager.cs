using UnityEngine;
using MiracleTwin.Core;
using MiracleTwin.CNC;
using MiracleTwin.Visualization;
using RosMessageTypes.Miracle;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Master orchestrator for the cutting simulation.
    /// Coordinates: VoxelWorkpiece, CuttingForceEngine, ThermalModel,
    /// ToolWearModel, ChipFormation, SurfaceRoughness, GCodeInterpreter.
    ///
    /// Runs each FixedUpdate when cutting is active, publishing
    /// CuttingStateData through the CuttingStateEventSO channel.
    ///
    /// Includes:
    /// - Stability lobe check: warns when RPM/DOC enters an unstable chatter zone
    /// - Auto-pause on VBmax: pauses simulation when tool reaches end-of-life
    /// - Full subsystem reset on SimulationClock.OnReset
    /// </summary>
    public class CuttingSimulationManager : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private MonoBehaviour cncControllerObject;
        private ICNCController cncController;
        [SerializeField] private VoxelWorkpiece voxelWorkpiece;
        [SerializeField] private CuttingForceEngine forceEngine;
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private ToolDefinition activeTool;
        [SerializeField] private StabilityLobeChart stabilityLobeChart;
        [SerializeField] private StabilityLobePredictor stabilityLobePredictor;
        [SerializeField] private GCodeExecutor executor;

        /// <summary>Whether the linked GCodeExecutor is currently in preview mode.</summary>
        public bool IsPreviewActive => executor != null && executor.IsPreviewMode;

        [Header("Simulation Settings")]
        [SerializeField] private bool autoStartCutting = false;
        [SerializeField] private float minimumCuttingRPM = 1000f;
        [SerializeField] private float minimumFeedRate = 10f;

        [Header("Stability Lobe")]
        [Tooltip("Warn when operating point enters unstable zone on the stability lobe diagram.")]
        [SerializeField] private bool enableStabilityLobeCheck = true;
        [SerializeField] private float stabilityCheckInterval = 0.25f;

        [Header("Sensor Fusion")]
        [Tooltip("When SensorDataBridge has live micro-ROS data, use real sensor readings to " +
                 "augment/override simulation models (temperature, chatter detection, power validation).")]
        [SerializeField] private bool enableSensorFusion = true;

        // Physics models
        private ThermalModel thermalModel;
        private ToolWearModel toolWearModel;

        // Sensor bridge (auto-discovered)
        private SensorDataBridge sensorBridge;

        // State
        public bool IsCutting { get; private set; }
        public CuttingStateData CurrentState { get; private set; }
        public float TotalMaterialRemoved { get; private set; } // mm³
        public float TotalCuttingTime { get; private set; }     // seconds
        public bool IsToolEndOfLife { get; private set; }
        public bool IsInUnstableZone { get; private set; }

        private Vector3 previousToolPosition;
        private bool wasToolInWorkpiece;
        private float lastStabilityCheckTime;
        private ChatterRisk lastChatterRisk = ChatterRisk.LOW;
        private bool toolEndOfLifePauseTriggered;

        void Awake()
        {
            thermalModel = new ThermalModel();
            toolWearModel = new ToolWearModel();

            cncController = cncControllerObject as ICNCController;
            if (cncController == null && cncControllerObject != null)
                Debug.LogError("[CuttingSimulationManager] cncControllerObject does not implement ICNCController");
        }

        void OnEnable()
        {
            if (SimulationClock.Instance != null)
                SimulationClock.Instance.OnReset += OnSimulationReset;
        }

        void OnDisable()
        {
            if (SimulationClock.Instance != null)
                SimulationClock.Instance.OnReset -= OnSimulationReset;
        }

        void Start()
        {
            if (cncController != null)
                previousToolPosition = cncController.GetToolTipWorldPosition();

            // Subscribe to OnReset again in case SimulationClock was created after OnEnable
            if (SimulationClock.Instance != null)
            {
                SimulationClock.Instance.OnReset -= OnSimulationReset;
                SimulationClock.Instance.OnReset += OnSimulationReset;
            }

            // Auto-discover SensorDataBridge for micro-ROS sensor fusion
            sensorBridge = FindFirstObjectByType<SensorDataBridge>();
            if (sensorBridge != null)
                Debug.Log("[CuttingSimulationManager] SensorDataBridge found — sensor fusion available when micro-ROS sensors online.");
        }

        void FixedUpdate()
        {
            if (cncController == null) return;

            // Skip physics when the CNC controller is disabled (machine not selected)
            if (cncControllerObject != null && !cncControllerObject.enabled) return;

            // Do not advance if tool is end-of-life and auto-paused
            if (toolEndOfLifePauseTriggered) return;

            Vector3 currentToolPos = cncController.GetToolTipWorldPosition();
            float rpm = cncController.SpindleRPM;
            float feedRate = cncController.FeedRate;
            float dt = Time.fixedDeltaTime;

            // Determine if we should be cutting
            bool shouldCut = rpm >= minimumCuttingRPM &&
                             feedRate >= minimumFeedRate &&
                             cncController.Status == "RUNNING";

            if (shouldCut)
            {
                UpdateCutting(currentToolPos, rpm, feedRate, dt);
            }
            else if (IsCutting)
            {
                StopCutting();
            }
            else
            {
                // Passive thermal cooldown
                thermalModel.Cooldown(dt);
            }

            previousToolPosition = currentToolPos;

            // Publish state
            PublishState(rpm, feedRate);
        }

        private void UpdateCutting(Vector3 toolPos, float rpm, float feedRate, float dt)
        {
            // In preview mode, skip all state-modifying operations (voxel subtraction, wear)
            if (IsPreviewActive) return;

            IsCutting = true;
            TotalCuttingTime += dt;

            float toolRadius = (activeTool != null ? activeTool.diameter : 6.35f) / 2000f; // mm→m

            // 1. Voxel subtraction
            int removedVoxels = 0;
            if (voxelWorkpiece != null)
            {
                removedVoxels = voxelWorkpiece.SubtractTool(previousToolPosition, toolPos, toolRadius);
            }

            // Estimate actual cutting depths from voxel engagement
            float fz = activeTool != null
                ? activeTool.CalculateChipLoad(rpm, feedRate)
                : feedRate / (rpm * 2f);
            float ae = activeTool != null ? Mathf.Min(activeTool.diameter * 0.5f, activeTool.diameter) : 3.175f;
            float ap = 1.5f; // Typical axial DOC for 1/4" endmill in 6061-T6 Al

            // 2. Cutting forces (Altintas model)
            if (forceEngine != null)
            {
                forceEngine.Calculate(rpm, fz, ap, ae, toolWearModel.VB);
            }

            // 3. Thermal model
            float V_mpm = activeTool != null ? activeTool.CalculateCuttingSpeed(rpm)
                                             : Mathf.PI * 6.35f * rpm / 1000f;
            float power = forceEngine != null ? forceEngine.PowerWatts : 0f;
            thermalModel.Update(V_mpm, fz, ap, power, dt);

            // 4. Tool wear (with temperature-wear coupling)
            toolWearModel.Update(V_mpm, fz, ap, dt, thermalModel.ToolTemperature);

            // Track total material removed
            if (removedVoxels > 0)
            {
                float voxelVolumeMM3 = VoxelWorkpiece.VoxelSize.x * VoxelWorkpiece.VoxelSize.y *
                                       VoxelWorkpiece.VoxelSize.z * 1e9f;
                TotalMaterialRemoved += removedVoxels * voxelVolumeMM3;
            }

            // 5. Feed current wear into stability predictor for wear-adjusted lobes
            if (stabilityLobePredictor != null)
            {
                float previousLimit = stabilityLobePredictor.LastStabilityLimit;
                stabilityLobePredictor.UpdateToolWear(toolWearModel.VB);

                // Warn when stability limit crosses below current depth of cut
                float newLimit = stabilityLobePredictor.CalculateStabilityLimit(rpm);
                if (previousLimit > ap && newLimit <= ap)
                {
                    Debug.LogWarning($"[CuttingSimulationManager] WEAR-STABILITY WARNING: " +
                                     $"Tool wear (VB={toolWearModel.VB:F3}mm) has shifted stability limit " +
                                     $"below current depth of cut ({ap:F2}mm). " +
                                     $"New stability limit: {newLimit:F2}mm. Consider reducing DOC or replacing tool.");
                }
            }

            // 6. Stability lobe check (periodic, not every frame)
            if (enableStabilityLobeCheck && Time.time - lastStabilityCheckTime >= stabilityCheckInterval)
            {
                lastStabilityCheckTime = Time.time;
                CheckStabilityLobe(rpm, ap);
            }

            // 7. Auto-pause on end-of-life (VBmax reached)
            IsToolEndOfLife = toolWearModel.IsEndOfLife;
            if (IsToolEndOfLife && !toolEndOfLifePauseTriggered)
            {
                toolEndOfLifePauseTriggered = true;
                Debug.LogWarning("[CuttingSimulationManager] Tool end-of-life reached (VB >= VBmax). " +
                                 "Simulation auto-paused. Replace tool and call ResetToolWear() to continue.");

                // Pause the simulation clock
                if (SimulationClock.Instance != null)
                    SimulationClock.Instance.Pause();

                StopCutting();
            }
        }

        /// <summary>
        /// Check whether current RPM and depth of cut fall within the unstable
        /// zone of the stability lobe diagram. Logs a warning when transitioning
        /// from stable to unstable.
        /// </summary>
        private void CheckStabilityLobe(float rpm, float depthMM)
        {
            if (stabilityLobeChart == null) return;

            bool wasUnstable = IsInUnstableZone;
            stabilityLobeChart.UpdateOperatingPoint(rpm, depthMM);
            IsInUnstableZone = !stabilityLobeChart.IsStable;

            if (IsInUnstableZone && !wasUnstable)
            {
                Debug.LogWarning($"[CuttingSimulationManager] CHATTER WARNING: Operating point " +
                                 $"(RPM={rpm:F0}, DOC={depthMM:F2}mm) is in the unstable zone " +
                                 $"of the stability lobe diagram. Reduce depth or adjust spindle speed.");
            }
            else if (!IsInUnstableZone && wasUnstable)
            {
                Debug.Log("[CuttingSimulationManager] Operating point returned to stable zone.");
            }

            // Enhanced chatter prediction using StabilityLobePredictor
            if (stabilityLobePredictor != null)
            {
                var risk = stabilityLobePredictor.EvaluateChatterRisk(rpm, depthMM);
                if (risk != lastChatterRisk)
                {
                    if (risk == ChatterRisk.HIGH)
                    {
                        float recommendedRPM = stabilityLobePredictor.RecommendStableRPM(rpm, depthMM);
                        Debug.LogWarning($"[CuttingSimulationManager] CHATTER RISK HIGH at RPM={rpm:F0}, DOC={depthMM:F2}mm. " +
                                         $"Recommended stable RPM: {recommendedRPM:F0}");
                    }
                    else if (risk == ChatterRisk.MEDIUM)
                    {
                        Debug.LogWarning($"[CuttingSimulationManager] CHATTER RISK MEDIUM at RPM={rpm:F0}, DOC={depthMM:F2}mm. " +
                                         $"Stability limit: {stabilityLobePredictor.LastStabilityLimit:F2}mm");
                    }
                    lastChatterRisk = risk;
                }

                // Publish stability recommendation to adaptive controller when risk is elevated
                if (risk == ChatterRisk.MEDIUM || risk == ChatterRisk.HIGH)
                {
                    PublishStabilityRecommendation(rpm, depthMM);
                }
            }
        }

        /// <summary>
        /// Get a stability recommendation from the predictor and publish it
        /// via MiracleBridge for the adaptive controller to consume.
        /// </summary>
        private void PublishStabilityRecommendation(float rpm, float depthMM)
        {
            if (stabilityLobePredictor == null || MiracleBridge.Instance == null) return;

            string machineId = MiracleBridge.Instance.MachineId;
            var recommendation = stabilityLobePredictor.GetStabilityRecommendation(machineId, rpm, depthMM);
            MiracleBridge.Instance.PublishStabilityRecommendation(recommendation);
        }

        private void StopCutting()
        {
            IsCutting = false;
        }

        private void PublishState(float rpm, float feedRate)
        {
            var state = new CuttingStateData
            {
                spindleRPM = rpm,
                feedRate = feedRate,
                toolPosition = cncController.GetToolTipWorldPosition(),
                toolDirection = cncController.GetToolDirection(),
                axialDepth = 1.5f,
                radialDepth = activeTool != null ? activeTool.diameter * 0.5f : 3.175f,
                toolDiameter = activeTool != null ? activeTool.diameter : 6.35f,
                isCutting = IsCutting,
                currentGCodeLine = (int)cncController.CurrentLine,

                forceFx = forceEngine != null ? forceEngine.AverageForce.x : 0,
                forceFy = forceEngine != null ? forceEngine.AverageForce.y : 0,
                forceFz = forceEngine != null ? forceEngine.AverageForce.z : 0,
                powerWatts = forceEngine != null ? forceEngine.PowerWatts : 0,
                torqueNm = forceEngine != null ? forceEngine.TorqueNm : 0,
                mrr = forceEngine != null ? forceEngine.MRR : 0,

                toolTemperature = thermalModel.ToolTemperature,
                interfaceTemperature = thermalModel.InterfaceTemperature,

                flankWearVB = toolWearModel.VB,
                wearPercentage = toolWearModel.WearPercentage,
            };

            // Chip formation + surface roughness
            if (IsCutting)
            {
                float V = activeTool != null ? activeTool.CalculateCuttingSpeed(rpm)
                                             : Mathf.PI * 6.35f * rpm / 1000f;
                float fz = activeTool != null
                    ? activeTool.CalculateChipLoad(rpm, feedRate)
                    : feedRate / (rpm * 2f);
                var chip = ChipFormationModel.Calculate(V, fz, activeTool?.diameter ?? 6.35f);
                state.chipThicknessRatio = chip.chipThicknessRatio;
                state.shearAngleDeg = chip.shearAngle;
                state.chipCurlRadius = chip.chipCurlRadius;
                state.chipVelocity = chip.chipVelocity;

                // Surface roughness (kinematic + Brammertz + vibration + wear-ploughing)
                float noseRadius = activeTool != null ? activeTool.noseRadius : 0.4f;
                float vibAmp = IsInUnstableZone ? 5f : 0f; // µm estimate when chatter detected
                float Ra = SurfaceRoughnessModel.CalculateRa(fz, noseRadius, vibAmp, toolWearModel.VB);
                state.surfaceRoughnessRa = Ra;
                state.surfaceRoughnessRz = SurfaceRoughnessModel.CalculateRz(Ra);
                state.surfaceGrade = SurfaceRoughnessModel.GetSurfaceGrade(Ra);
            }

            // Sensor fusion: when micro-ROS sensors are online, augment simulation with real data
            if (enableSensorFusion && sensorBridge != null && sensorBridge.HasSensorData)
            {
                ApplySensorFusion(ref state);
            }

            CurrentState = state;
            cuttingStateEvent?.Raise(state);
        }

        /// <summary>
        /// Blend real micro-ROS sensor readings with simulation predictions.
        /// Real sensor data takes priority; simulation fills gaps.
        /// </summary>
        private void ApplySensorFusion(ref CuttingStateData state)
        {
            byte health = sensorBridge.SensorHealthMask;

            // Temperature sensor online (bit 2) → override tool temperature with real reading
            if ((health & 0x04) != 0)
            {
                state.toolTemperature = sensorBridge.SensorTemperature;
            }

            // Vibration sensor online (bit 0) → use real vibration for chatter detection
            if ((health & 0x01) != 0)
            {
                bool sensorChatter = sensorBridge.IsChatterDetected();
                if (sensorChatter && !IsInUnstableZone)
                {
                    IsInUnstableZone = true;
                    Debug.LogWarning("[CuttingSimulationManager] CHATTER DETECTED by vibration sensor " +
                        $"(magnitude: {sensorBridge.VibrationMagnitude:F0} mm/s²).");
                }

                // Use real vibration amplitude for surface roughness estimation
                if (IsCutting && state.surfaceRoughnessRa > 0)
                {
                    float realVibAmp = sensorBridge.VibrationMagnitude * 0.001f; // mm/s² → µm estimate
                    float fz = activeTool != null
                        ? activeTool.CalculateChipLoad(state.spindleRPM, state.feedRate)
                        : state.feedRate / (state.spindleRPM * 2f);
                    float noseRadius = activeTool != null ? activeTool.noseRadius : 0.4f;
                    state.surfaceRoughnessRa = SurfaceRoughnessModel.CalculateRa(
                        fz, noseRadius, realVibAmp, toolWearModel.VB);
                    state.surfaceRoughnessRz = SurfaceRoughnessModel.CalculateRz(state.surfaceRoughnessRa);
                    state.surfaceGrade = SurfaceRoughnessModel.GetSurfaceGrade(state.surfaceRoughnessRa);
                }
            }

            // Current sensor online (bit 1) → validate force predictions
            if ((health & 0x02) != 0)
            {
                float realPower = sensorBridge.EstimatedPowerFromCurrent();
                // If real power diverges significantly from predicted, log warning
                if (IsCutting && state.powerWatts > 0 && Mathf.Abs(realPower - state.powerWatts) > state.powerWatts * 0.5f)
                {
                    Debug.LogWarning($"[CuttingSimulationManager] Power mismatch: sensor={realPower:F1}W vs model={state.powerWatts:F1}W " +
                        "(>50% divergence — check tool condition or cutting parameters).");
                }
            }
        }

        /// <summary>
        /// Called when SimulationClock.OnReset fires. Resets all cutting
        /// subsystems to their initial state.
        /// </summary>
        private void OnSimulationReset()
        {
            ResetSimulation();
        }

        // ── Preview Force Estimation ─────────────────────────────────────

        // Altintas coefficients (must match GCodeLookahead/CuttingForceEngine)
        private const float PreviewKtc = 796f;
        private const float PreviewKrc = 168f;
        private const float PreviewKte = 14.5f;
        private const float PreviewKre = 10.2f;
        private const float PreviewDefaultAxialDepth = 1.0f; // mm
        private const int PreviewDefaultFlutes = 2;
        private const float PreviewDefaultToolDiameter = 6.35f; // mm

        /// <summary>
        /// Estimate the cutting force for a move between two points at the given
        /// feed rate and spindle speed. This is a pure calculation that does NOT
        /// modify any simulation state (voxels, wear, temperature).
        /// </summary>
        /// <returns>Estimated resultant force in Newtons.</returns>
        public float GetPreviewForceEstimate(Vector3 start, Vector3 end, float feed, float rpm)
        {
            if (rpm < 1f || feed < 0.01f) return 0f;

            float fz = feed / (rpm * PreviewDefaultFlutes);
            float chipThickness = fz;

            float Ft = PreviewKtc * PreviewDefaultAxialDepth * chipThickness + PreviewKte * PreviewDefaultAxialDepth;
            float Fr = PreviewKrc * PreviewDefaultAxialDepth * chipThickness + PreviewKre * PreviewDefaultAxialDepth;

            return Mathf.Sqrt(Ft * Ft + Fr * Fr);
        }

        /// <summary>
        /// Full reset of all cutting simulation subsystems.
        /// </summary>
        public void ResetSimulation()
        {
            IsCutting = false;
            TotalMaterialRemoved = 0;
            TotalCuttingTime = 0;
            IsToolEndOfLife = false;
            IsInUnstableZone = false;
            toolEndOfLifePauseTriggered = false;
            lastStabilityCheckTime = 0;

            thermalModel.Reset();
            toolWearModel.Reset();
            voxelWorkpiece?.Reset();

            if (forceEngine != null)
            {
                // Reset force engine outputs by calculating with zero params
                forceEngine.Calculate(0, 0, 0, 0, 0);
            }

            if (cncController != null)
                previousToolPosition = cncController.GetToolTipWorldPosition();

            Debug.Log("[CuttingSimulationManager] Full simulation reset completed.");
        }

        /// <summary>
        /// Reset only tool wear state (for tool change), allowing cutting to resume
        /// after an end-of-life auto-pause.
        /// </summary>
        public void ResetToolWear()
        {
            toolWearModel.Reset();
            IsToolEndOfLife = false;
            toolEndOfLifePauseTriggered = false;
            Debug.Log("[CuttingSimulationManager] Tool wear reset (tool change).");
        }

        /// <summary>
        /// Replace the VoxelWorkpiece reference at runtime (for machine switching).
        /// </summary>
        public void SetVoxelWorkpiece(VoxelWorkpiece wp)
        {
            voxelWorkpiece = wp;
        }
    }
}
