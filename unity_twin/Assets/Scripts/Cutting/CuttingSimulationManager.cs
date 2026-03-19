using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using MiracleTwin.Core;
using MiracleTwin.CNC;
using MiracleTwin.Visualization;
using RosMessageTypes.Miracle;

namespace MiracleTwin.Cutting
{
    // ── GD&T Enums & Data Classes ──────────────────────────────────────

    /// <summary>
    /// Geometric tolerance types per ASME Y14.5 / ISO 1101.
    /// </summary>
    public enum ToleranceType
    {
        POSITION,
        FLATNESS,
        PARALLELISM,
        PERPENDICULARITY,
        CONCENTRICITY,
        CIRCULARITY,
        CYLINDRICITY,
        PROFILE,
        RUNOUT,
        TOTAL_RUNOUT
    }

    /// <summary>
    /// Defines a single GD&amp;T tolerance specification for a machined feature.
    /// </summary>
    [Serializable]
    public class ToleranceSpec
    {
        public ToleranceType toleranceType;
        public string featureId;
        public float nominalValue;       // mm
        public float toleranceZone;      // mm  (total tolerance band)
        public string datumReference;    // e.g. "A", "A|B"
        public string materialCondition; // MMC, LMC, RFS
    }

    /// <summary>
    /// Result of analyzing a feature against its tolerance specification.
    /// </summary>
    [Serializable]
    public class ToleranceResult
    {
        public ToleranceSpec spec;
        public float actualDeviation;        // mm
        public bool isInTolerance;
        public float percentOfTolerance;     // 0-100
        public string riskLevel;             // LOW, MEDIUM, HIGH, OUT_OF_SPEC
        public List<string> contributingFactors;
        public float predictedDriftPerHour;
    }

    /// <summary>
    /// Analyzes geometric tolerances for machined features based on
    /// tool deflection, thermal expansion, and wear compensation.
    /// </summary>
    public class GeometricToleranceAnalyzer
    {
        /// <summary>Registered tolerance specifications.</summary>
        public List<ToleranceSpec> specs { get; private set; } = new List<ToleranceSpec>();

        /// <summary>Coefficient of thermal expansion for 6061-T6 aluminum (1/K).</summary>
        private const float AluminumCTE = 11.7e-6f;

        /// <summary>Reference / ambient temperature in Celsius.</summary>
        private const float ReferenceTemp = 20f;

        /// <summary>Default feature length used when nominal value is available (mm).</summary>
        private const float DefaultFeatureLength = 50f;

        /// <summary>Add a tolerance specification to be tracked.</summary>
        public void AddSpec(ToleranceSpec spec)
        {
            if (spec == null) throw new ArgumentNullException(nameof(spec));
            specs.Add(spec);
        }

        /// <summary>
        /// Analyze a single feature given current deviation sources.
        /// Total deviation = toolDeflection + thermalExpansion − wearCompensation
        /// </summary>
        public ToleranceResult AnalyzeFeature(
            string featureId,
            float toolDeflection,
            float thermalExpansion,
            float wearCompensation)
        {
            var spec = specs.Find(s => s.featureId == featureId);
            if (spec == null)
                throw new ArgumentException($"No spec found for feature '{featureId}'");

            float totalDeviation = toolDeflection + thermalExpansion - wearCompensation;
            float absDeviation = Mathf.Abs(totalDeviation);

            float halfZone = spec.toleranceZone / 2f;
            float percent = halfZone > 0f ? (absDeviation / halfZone) * 100f : (absDeviation > 0f ? float.MaxValue : 0f);

            string risk;
            if (percent < 50f) risk = "LOW";
            else if (percent < 80f) risk = "MEDIUM";
            else if (percent < 100f) risk = "HIGH";
            else risk = "OUT_OF_SPEC";

            var factors = new List<string>();
            if (Mathf.Abs(toolDeflection) > 0.001f)
                factors.Add($"tool_deflection:{toolDeflection:F4}mm");
            if (Mathf.Abs(thermalExpansion) > 0.001f)
                factors.Add($"thermal_expansion:{thermalExpansion:F4}mm");
            if (Mathf.Abs(wearCompensation) > 0.001f)
                factors.Add($"wear_compensation:{wearCompensation:F4}mm");

            return new ToleranceResult
            {
                spec = spec,
                actualDeviation = totalDeviation,
                isInTolerance = percent <= 100f,
                percentOfTolerance = percent,
                riskLevel = risk,
                contributingFactors = factors,
                predictedDriftPerHour = 0f
            };
        }

        /// <summary>
        /// Predict tolerance risk for every registered spec given current machine state.
        /// Uses aluminum CTE for thermal expansion: deltaL = CTE * deltaT * featureLength.
        /// </summary>
        public List<ToleranceResult> PredictToleranceRisk(
            float currentTemp,
            float toolWearMM,
            float spindleRunout)
        {
            var results = new List<ToleranceResult>();
            float deltaT = currentTemp - ReferenceTemp;

            foreach (var spec in specs)
            {
                float featureLength = spec.nominalValue > 0f ? spec.nominalValue : DefaultFeatureLength;
                float thermalExpansion = AluminumCTE * deltaT * featureLength;
                float toolDeflection = spindleRunout + toolWearMM * 0.1f;
                float wearCompensation = 0f; // no active compensation in prediction

                var result = AnalyzeFeature(spec.featureId, toolDeflection, thermalExpansion, wearCompensation);
                // Estimate drift per hour based on thermal trend and wear rate
                float driftPerHour = Mathf.Abs(AluminumCTE * 2f * featureLength) + toolWearMM * 0.05f;
                result.predictedDriftPerHour = driftPerHour;
                results.Add(result);
            }

            return results;
        }

        /// <summary>
        /// Returns all features whose tolerance utilisation is above 70 %.
        /// </summary>
        public List<ToleranceResult> GetCriticalFeatures()
        {
            // Re-evaluate every spec with zero inputs to get baseline;
            // callers should use PredictToleranceRisk and filter instead.
            // This convenience method returns specs previously evaluated
            // through PredictToleranceRisk at the default machine state.
            return PredictToleranceRisk(ReferenceTemp, 0f, 0f)
                .Where(r => r.percentOfTolerance >= 70f)
                .ToList();
        }

        /// <summary>
        /// Estimate a tool-path compensation offset for the given feature (mm).
        /// Returns half the current predicted deviation so the controller can
        /// shift the tool path to centre the error band.
        /// </summary>
        public float EstimateCompensation(string featureId)
        {
            var spec = specs.Find(s => s.featureId == featureId);
            if (spec == null)
                throw new ArgumentException($"No spec found for feature '{featureId}'");

            // Predict at a mild elevated state to give a useful offset
            float featureLength = spec.nominalValue > 0f ? spec.nominalValue : DefaultFeatureLength;
            float estimatedThermal = AluminumCTE * 5f * featureLength; // assume 5 °C rise
            return estimatedThermal / 2f;
        }
    }
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

    // ── Machine Digital Passport ────────────────────────────────────────

    /// <summary>
    /// Core identity information for a CNC machine tool.
    /// </summary>
    [Serializable]
    public class MachineIdentity
    {
        public string serialNumber;
        public string manufacturer;
        public string model;
        public int yearOfManufacture;
        public string controllerType;
        public float maxSpindleRpm;
        public float maxFeedRate;       // mm/min
        public Vector3 workEnvelope;    // X, Y, Z travel in mm
        public int axisCount;
    }

    /// <summary>
    /// A single maintenance event in the machine's lifecycle.
    /// Type must be one of: "preventive", "corrective", "predictive".
    /// </summary>
    [Serializable]
    public class MaintenanceRecord
    {
        public string recordId;
        public string date;             // ISO-8601 date string
        public string type;             // preventive | corrective | predictive
        public string description;
        public string technician;
        public List<string> partsReplaced;
        public float downtimeHours;
        public float cost;
    }

    /// <summary>
    /// A single calibration measurement for one machine parameter.
    /// </summary>
    [Serializable]
    public class CalibrationRecord
    {
        public string recordId;
        public string date;             // ISO-8601 date string
        public string parameter;        // e.g. "X_axis_backlash", "spindle_runout"
        public float measuredValue;
        public float nominalValue;
        public float tolerance;
        public bool passed;
        public string calibratedBy;
    }

    /// <summary>
    /// Health summary snapshot returned by <see cref="MachineDigitalPassport.GetHealthSummary"/>.
    /// </summary>
    [Serializable]
    public class HealthSummary
    {
        public float uptimePercentage;
        public Dictionary<string, bool> calibrationStatus;
        public float maintenanceCompliancePercentage;
        public float mtbf;
        public int totalPartsProduced;
        public float totalOperatingHours;
    }

    /// <summary>
    /// Comprehensive digital passport for a CNC machine tool.
    /// Tracks identity, maintenance history, calibration records,
    /// operating hours, and parts produced over the machine's lifecycle.
    /// Provides analytics for health monitoring, MTBF, and calibration compliance.
    /// </summary>
    public class MachineDigitalPassport
    {
        public MachineIdentity identity { get; set; }
        public List<MaintenanceRecord> maintenanceHistory { get; private set; } = new List<MaintenanceRecord>();
        public List<CalibrationRecord> calibrationHistory { get; private set; } = new List<CalibrationRecord>();
        public float totalOperatingHours { get; set; }
        public int totalPartsProduced { get; set; }

        public MachineDigitalPassport(MachineIdentity identity)
        {
            this.identity = identity ?? throw new ArgumentNullException(nameof(identity));
        }

        /// <summary>Append a maintenance record to the history.</summary>
        public void AddMaintenanceRecord(MaintenanceRecord record)
        {
            if (record == null) throw new ArgumentNullException(nameof(record));
            maintenanceHistory.Add(record);
        }

        /// <summary>Append a calibration record to the history.</summary>
        public void AddCalibrationRecord(CalibrationRecord record)
        {
            if (record == null) throw new ArgumentNullException(nameof(record));
            calibrationHistory.Add(record);
        }

        /// <summary>Filter maintenance records by type (preventive, corrective, predictive).</summary>
        public List<MaintenanceRecord> GetMaintenanceByType(string type)
        {
            return maintenanceHistory.Where(r => r.type == type).ToList();
        }

        /// <summary>
        /// Returns a dictionary mapping each calibrated parameter to the pass/fail
        /// result of its most recent calibration.
        /// </summary>
        public Dictionary<string, bool> GetCalibrationStatus()
        {
            var status = new Dictionary<string, bool>();
            // Group by parameter and pick the latest record (last in list) for each
            var grouped = calibrationHistory
                .GroupBy(r => r.parameter)
                .ToDictionary(g => g.Key, g => g.Last().passed);

            foreach (var kvp in grouped)
                status[kvp.Key] = kvp.Value;

            return status;
        }

        /// <summary>
        /// Overall machine health summary: uptime percentage, calibration status,
        /// maintenance compliance, MTBF, and production statistics.
        /// </summary>
        public HealthSummary GetHealthSummary()
        {
            float totalDowntime = maintenanceHistory.Sum(r => r.downtimeHours);
            float uptime = totalOperatingHours > 0f
                ? ((totalOperatingHours - totalDowntime) / totalOperatingHours) * 100f
                : 100f;
            // Clamp to [0, 100]
            uptime = Mathf.Clamp(uptime, 0f, 100f);

            var calStatus = GetCalibrationStatus();

            // Maintenance compliance: percentage of calibrated parameters that passed
            float compliance = 100f;
            if (calStatus.Count > 0)
            {
                int passedCount = calStatus.Values.Count(v => v);
                compliance = ((float)passedCount / calStatus.Count) * 100f;
            }

            return new HealthSummary
            {
                uptimePercentage = uptime,
                calibrationStatus = calStatus,
                maintenanceCompliancePercentage = compliance,
                mtbf = GetMTBF(),
                totalPartsProduced = totalPartsProduced,
                totalOperatingHours = totalOperatingHours
            };
        }

        /// <summary>
        /// Check whether a specific calibration parameter is due for recalibration.
        /// Returns true if no calibration exists for the parameter or if the last
        /// calibration was performed more than <paramref name="intervalDays"/> ago.
        /// </summary>
        public bool IsCalibrationDue(string parameter, float intervalDays)
        {
            var records = calibrationHistory
                .Where(r => r.parameter == parameter)
                .ToList();

            if (records.Count == 0)
                return true;

            var lastRecord = records.Last();

            DateTime lastDate;
            if (!DateTime.TryParse(lastRecord.date, out lastDate))
                return true;

            double daysSince = (DateTime.Now - lastDate).TotalDays;
            return daysSince >= intervalDays;
        }

        /// <summary>
        /// Mean Time Between Failures calculated from corrective maintenance records.
        /// Returns 0 if there are fewer than 2 corrective records or dates cannot be parsed.
        /// MTBF = average interval between consecutive corrective maintenance events.
        /// </summary>
        public float GetMTBF()
        {
            var correctiveRecords = maintenanceHistory
                .Where(r => r.type == "corrective")
                .ToList();

            if (correctiveRecords.Count < 2)
                return 0f;

            // Parse dates and sort
            var dates = new List<DateTime>();
            foreach (var record in correctiveRecords)
            {
                DateTime dt;
                if (DateTime.TryParse(record.date, out dt))
                    dates.Add(dt);
            }

            if (dates.Count < 2)
                return 0f;

            dates.Sort();

            // Calculate average interval in hours between consecutive failures
            double totalHours = 0;
            for (int i = 1; i < dates.Count; i++)
            {
                totalHours += (dates[i] - dates[i - 1]).TotalHours;
            }

            return (float)(totalHours / (dates.Count - 1));
        }
    }

    // ── Touch Probe Measurement Simulator ─────────────────────────────

    /// <summary>
    /// A single probed point captured during a G38.2 touch probe cycle.
    /// </summary>
    [Serializable]
    public class ProbePoint
    {
        public Vector3 position;
        public Vector3 normal;
        public float measuredValue;
        public float nominalValue;
        public float deviation;
        public string timestamp;
    }

    /// <summary>
    /// Result of a probe measurement cycle for a single feature.
    /// </summary>
    [Serializable]
    public class ProbeResult
    {
        public List<ProbePoint> points;
        public string featureType;          // bore, boss, plane, edge, slot
        public float measuredDiameter;
        public Vector3 measuredPosition;
        public float formError;
        public float positionError;
        public bool passed;
    }

    /// <summary>
    /// Encapsulates a complete probe measurement cycle including nominal
    /// specification, captured points, and the computed result.
    /// </summary>
    [Serializable]
    public class ProbeCycle
    {
        public string cycleId;
        public string featureType;
        public float nominalDiameter;
        public Vector3 nominalPosition;
        public float tolerance;
        public List<ProbePoint> points;
        public ProbeResult result;
    }

    /// <summary>
    /// Simulates touch probe measurement cycles (G38.2 probing moves) for
    /// bore, boss, and plane features.  Generates synthetic probe data with
    /// configurable Gaussian noise, computes form errors (circularity /
    /// flatness), and produces formatted measurement reports.
    /// </summary>
    public class ProbeMeasurementSimulator
    {
        /// <summary>Standard deviation of measurement noise in mm.</summary>
        private float noiseStdDev;

        /// <summary>Random number generator used for Gaussian noise.</summary>
        private System.Random rng;

        /// <summary>Running counter for cycle IDs.</summary>
        private int cycleCounter;

        public ProbeMeasurementSimulator(float noiseStdDev = 0.002f, int? seed = null)
        {
            this.noiseStdDev = noiseStdDev;
            this.rng = seed.HasValue ? new System.Random(seed.Value) : new System.Random();
            this.cycleCounter = 0;
        }

        // ── Noise Generation ──────────────────────────────────────────

        /// <summary>
        /// Generate a Gaussian-distributed random value using the Box-Muller transform.
        /// </summary>
        private float GaussianNoise()
        {
            double u1 = 1.0 - rng.NextDouble(); // avoid log(0)
            double u2 = rng.NextDouble();
            double z = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
            return (float)(z * noiseStdDev);
        }

        /// <summary>Generate the next unique cycle identifier.</summary>
        private string NextCycleId()
        {
            cycleCounter++;
            return $"PROBE-{cycleCounter:D4}";
        }

        // ── Bore Measurement ──────────────────────────────────────────

        /// <summary>
        /// Simulate a bore (internal cylinder) probing cycle.
        /// Points are distributed evenly around the bore circumference with
        /// Gaussian noise applied to the radial measurement.
        /// </summary>
        public ProbeCycle SimulateBoreMeasurement(
            Vector3 center,
            float nominalDia,
            float tolerance,
            int numPoints = 8)
        {
            if (numPoints < 3)
                throw new ArgumentException("At least 3 points are required for bore measurement.");

            float nominalRadius = nominalDia / 2f;
            var points = new List<ProbePoint>();
            string ts = DateTime.UtcNow.ToString("o");

            for (int i = 0; i < numPoints; i++)
            {
                float angle = (2f * Mathf.PI * i) / numPoints;
                float noise = GaussianNoise();
                float measuredRadius = nominalRadius + noise;

                Vector3 normal = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0f);
                Vector3 pos = center + normal * measuredRadius;

                points.Add(new ProbePoint
                {
                    position = pos,
                    normal = normal,
                    measuredValue = measuredRadius * 2f,
                    nominalValue = nominalDia,
                    deviation = noise * 2f,
                    timestamp = ts
                });
            }

            float formError = CalculateFormError(points, "bore");
            float measuredDia = points.Average(p => p.measuredValue);
            float positionError = Vector3.Distance(
                CalculateMeasuredCenter(points, center),
                center);
            bool passed = Mathf.Abs(measuredDia - nominalDia) <= tolerance && formError <= tolerance;

            var result = new ProbeResult
            {
                points = points,
                featureType = "bore",
                measuredDiameter = measuredDia,
                measuredPosition = CalculateMeasuredCenter(points, center),
                formError = formError,
                positionError = positionError,
                passed = passed
            };

            return new ProbeCycle
            {
                cycleId = NextCycleId(),
                featureType = "bore",
                nominalDiameter = nominalDia,
                nominalPosition = center,
                tolerance = tolerance,
                points = points,
                result = result
            };
        }

        // ── Boss Measurement ──────────────────────────────────────────

        /// <summary>
        /// Simulate a boss (external cylinder) probing cycle.
        /// Points are probed inward from outside the boss circumference.
        /// </summary>
        public ProbeCycle SimulateBossMeasurement(
            Vector3 center,
            float nominalDia,
            float tolerance,
            int numPoints = 8)
        {
            if (numPoints < 3)
                throw new ArgumentException("At least 3 points are required for boss measurement.");

            float nominalRadius = nominalDia / 2f;
            var points = new List<ProbePoint>();
            string ts = DateTime.UtcNow.ToString("o");

            for (int i = 0; i < numPoints; i++)
            {
                float angle = (2f * Mathf.PI * i) / numPoints;
                float noise = GaussianNoise();
                float measuredRadius = nominalRadius + noise;

                // Normal points inward (toward center) for boss measurement
                Vector3 outward = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0f);
                Vector3 normal = -outward;
                Vector3 pos = center + outward * measuredRadius;

                points.Add(new ProbePoint
                {
                    position = pos,
                    normal = normal,
                    measuredValue = measuredRadius * 2f,
                    nominalValue = nominalDia,
                    deviation = noise * 2f,
                    timestamp = ts
                });
            }

            float formError = CalculateFormError(points, "boss");
            float measuredDia = points.Average(p => p.measuredValue);
            float positionError = Vector3.Distance(
                CalculateMeasuredCenter(points, center),
                center);
            bool passed = Mathf.Abs(measuredDia - nominalDia) <= tolerance && formError <= tolerance;

            var result = new ProbeResult
            {
                points = points,
                featureType = "boss",
                measuredDiameter = measuredDia,
                measuredPosition = CalculateMeasuredCenter(points, center),
                formError = formError,
                positionError = positionError,
                passed = passed
            };

            return new ProbeCycle
            {
                cycleId = NextCycleId(),
                featureType = "boss",
                nominalDiameter = nominalDia,
                nominalPosition = center,
                tolerance = tolerance,
                points = points,
                result = result
            };
        }

        // ── Plane Measurement ─────────────────────────────────────────

        /// <summary>
        /// Simulate a surface flatness probing cycle.
        /// Points are distributed in a grid across the plane surface with
        /// Gaussian noise applied along the surface normal.
        /// </summary>
        public ProbeCycle SimulatePlaneMeasurement(
            Vector3 origin,
            Vector3 normal,
            float width,
            float length,
            int numPoints = 9)
        {
            if (numPoints < 3)
                throw new ArgumentException("At least 3 points are required for plane measurement.");

            normal = normal.normalized;

            // Build a local coordinate frame on the plane
            Vector3 tangent1 = Vector3.Cross(normal, Vector3.right).normalized;
            if (tangent1.magnitude < 0.01f)
                tangent1 = Vector3.Cross(normal, Vector3.forward).normalized;
            Vector3 tangent2 = Vector3.Cross(normal, tangent1).normalized;

            var points = new List<ProbePoint>();
            string ts = DateTime.UtcNow.ToString("o");

            int gridSize = Mathf.Max(2, Mathf.CeilToInt(Mathf.Sqrt(numPoints)));
            int generated = 0;

            for (int i = 0; i < gridSize && generated < numPoints; i++)
            {
                for (int j = 0; j < gridSize && generated < numPoints; j++)
                {
                    float u = gridSize > 1 ? (float)i / (gridSize - 1) : 0.5f;
                    float v = gridSize > 1 ? (float)j / (gridSize - 1) : 0.5f;

                    Vector3 localOffset = tangent1 * (u - 0.5f) * width
                                        + tangent2 * (v - 0.5f) * length;
                    float noise = GaussianNoise();
                    Vector3 pos = origin + localOffset + normal * noise;

                    points.Add(new ProbePoint
                    {
                        position = pos,
                        normal = normal,
                        measuredValue = noise,
                        nominalValue = 0f,
                        deviation = noise,
                        timestamp = ts
                    });
                    generated++;
                }
            }

            float formError = CalculateFormError(points, "plane");

            var result = new ProbeResult
            {
                points = points,
                featureType = "plane",
                measuredDiameter = 0f,
                measuredPosition = origin,
                formError = formError,
                positionError = 0f,
                passed = formError <= 0.01f  // default 10 µm flatness tolerance
            };

            return new ProbeCycle
            {
                cycleId = NextCycleId(),
                featureType = "plane",
                nominalDiameter = 0f,
                nominalPosition = origin,
                tolerance = 0.01f,
                points = points,
                result = result
            };
        }

        // ── Form Error Calculation ────────────────────────────────────

        /// <summary>
        /// Calculate the form error for a set of probed points.
        /// For bore/boss features: circularity (max radius − min radius).
        /// For plane features: flatness (max deviation − min deviation).
        /// </summary>
        public float CalculateFormError(List<ProbePoint> points, string featureType)
        {
            if (points == null || points.Count == 0)
                return 0f;

            switch (featureType)
            {
                case "bore":
                case "boss":
                    // Circularity: difference between max and min measured radii
                    float maxRadius = points.Max(p => p.measuredValue / 2f);
                    float minRadius = points.Min(p => p.measuredValue / 2f);
                    return maxRadius - minRadius;

                case "plane":
                    // Flatness: range of deviations along the normal
                    float maxDev = points.Max(p => p.deviation);
                    float minDev = points.Min(p => p.deviation);
                    return maxDev - minDev;

                default:
                    // Generic: range of deviations
                    float maxD = points.Max(p => Mathf.Abs(p.deviation));
                    return maxD;
            }
        }

        // ── Measurement Report ────────────────────────────────────────

        /// <summary>
        /// Generate a formatted measurement report for a probe cycle.
        /// Includes feature type, nominal / measured values, form error,
        /// position error, and a PASS/FAIL verdict.
        /// </summary>
        public string GetMeasurementReport(ProbeCycle cycle)
        {
            if (cycle == null)
                throw new ArgumentNullException(nameof(cycle));
            if (cycle.result == null)
                throw new ArgumentException("ProbeCycle has no result.");

            var r = cycle.result;
            var lines = new List<string>
            {
                "═══════════════════════════════════════════════════",
                $"  PROBE MEASUREMENT REPORT — {cycle.cycleId}",
                "═══════════════════════════════════════════════════",
                $"  Feature Type    : {cycle.featureType}",
            };

            if (cycle.featureType == "bore" || cycle.featureType == "boss")
            {
                lines.Add($"  Nominal Diameter: {cycle.nominalDiameter:F4} mm");
                lines.Add($"  Measured Diameter: {r.measuredDiameter:F4} mm");
                lines.Add($"  Diameter Error  : {Mathf.Abs(r.measuredDiameter - cycle.nominalDiameter):F4} mm");
            }

            lines.Add($"  Nominal Position: ({cycle.nominalPosition.x:F4}, {cycle.nominalPosition.y:F4}, {cycle.nominalPosition.z:F4})");
            lines.Add($"  Measured Position: ({r.measuredPosition.x:F4}, {r.measuredPosition.y:F4}, {r.measuredPosition.z:F4})");
            lines.Add($"  Form Error      : {r.formError:F4} mm");
            lines.Add($"  Position Error  : {r.positionError:F4} mm");
            lines.Add($"  Tolerance       : {cycle.tolerance:F4} mm");
            lines.Add($"  Points Measured : {r.points.Count}");
            lines.Add($"  Verdict         : {(r.passed ? "PASS" : "FAIL")}");
            lines.Add("═══════════════════════════════════════════════════");

            return string.Join("\n", lines);
        }

        // ── Helpers ───────────────────────────────────────────────────

        /// <summary>
        /// Compute the centre of the measured points by averaging positions,
        /// projected back along each point's normal to account for radial
        /// measurement geometry.
        /// </summary>
        private Vector3 CalculateMeasuredCenter(List<ProbePoint> points, Vector3 nominalCenter)
        {
            if (points.Count == 0) return nominalCenter;

            Vector3 sum = Vector3.zero;
            foreach (var p in points)
                sum += p.position;

            return sum / points.Count;
        }
    }

    // ── Spindle Bearing Health Monitor ───────────────────────────────

    /// <summary>
    /// A single bearing sensor reading capturing temperature, vibration, and load data.
    /// </summary>
    [Serializable]
    public class BearingReading
    {
        public string timestamp;          // ISO-8601
        public float temperature;         // °C
        public float vibrationRms;        // mm/s  (root-mean-square)
        public float vibrationPeak;       // mm/s  (peak value)
        public float axialLoad;           // N
        public float radialLoad;          // N
    }

    /// <summary>
    /// Health report produced by <see cref="SpindleBearingMonitor"/>.
    /// </summary>
    [Serializable]
    public class BearingHealthReport
    {
        public float overallHealth;                  // 0-100
        public string temperatureStatus;             // "normal", "elevated", "critical"
        public string vibrationStatus;               // "normal", "elevated", "critical"
        public string lubricationStatus;             // "good", "marginal", "poor"
        public float estimatedRemainingHours;
        public List<string> recommendations = new List<string>();
    }

    /// <summary>
    /// Temperature trend result returned by <see cref="SpindleBearingMonitor.GetTemperatureTrend"/>.
    /// </summary>
    [Serializable]
    public class TemperatureTrend
    {
        public float average;
        public float slope;
        public bool isRising;
    }

    /// <summary>
    /// Vibration spectrum summary returned by <see cref="SpindleBearingMonitor.GetVibrationSpectrum"/>.
    /// </summary>
    [Serializable]
    public class VibrationSpectrum
    {
        public float rmsAvg;
        public float peakAvg;
        public float crestFactor;
    }

    /// <summary>
    /// Monitors spindle bearing health through vibration and temperature analysis.
    ///
    /// Health scoring weights:
    ///   Temperature  30 %
    ///   Vibration    40 %
    ///   Load         20 %
    ///   Trend        10 %
    ///
    /// Thresholds:
    ///   Temperature – normal &lt; 50 °C, elevated &lt; 70 °C, critical &ge; 70 °C
    ///   Vibration   – normal &lt; 2.5 mm/s, elevated &lt; 5.0 mm/s, critical &ge; 5.0 mm/s
    /// </summary>
    public class SpindleBearingMonitor
    {
        // ── Constants ────────────────────────────────────────────────

        private const int MaxReadings = 1000;
        private const int TrendWindow = 50;

        // Temperature thresholds (°C)
        private const float TempNormalMax = 50f;
        private const float TempElevatedMax = 70f;

        // Vibration RMS thresholds (mm/s)
        private const float VibNormalMax = 2.5f;
        private const float VibElevatedMax = 5.0f;

        // Scoring weights
        private const float WeightTemperature = 0.30f;
        private const float WeightVibration   = 0.40f;
        private const float WeightLoad        = 0.20f;
        private const float WeightTrend       = 0.10f;

        // Load reference (rated load for scoring – readings above this reduce score)
        private const float RatedAxialLoad  = 5000f;  // N
        private const float RatedRadialLoad = 3000f;  // N

        // ── State ────────────────────────────────────────────────────

        private List<BearingReading> readings = new List<BearingReading>();

        // ── Recording ────────────────────────────────────────────────

        /// <summary>
        /// Store a bearing reading.  The buffer is capped at 1 000 entries;
        /// the oldest readings are discarded when the limit is reached.
        /// </summary>
        public void RecordReading(BearingReading reading)
        {
            if (reading == null) return;

            readings.Add(reading);
            if (readings.Count > MaxReadings)
                readings.RemoveAt(0);
        }

        /// <summary>Current number of stored readings.</summary>
        public int ReadingCount => readings.Count;

        // ── Health Report ────────────────────────────────────────────

        /// <summary>
        /// Analyse recent readings and produce a comprehensive health report.
        /// </summary>
        public BearingHealthReport GetHealthReport()
        {
            var report = new BearingHealthReport();

            if (readings.Count == 0)
            {
                report.overallHealth = 100f;
                report.temperatureStatus = "normal";
                report.vibrationStatus = "normal";
                report.lubricationStatus = "good";
                report.estimatedRemainingHours = 0f;
                report.recommendations.Add("No readings recorded yet.");
                return report;
            }

            // --- Temperature scoring (30 %) ---
            float avgTemp = readings.Average(r => r.temperature);
            float tempScore = ScoreTemperature(avgTemp);
            report.temperatureStatus = ClassifyTemperature(avgTemp);

            // --- Vibration scoring (40 %) ---
            var spectrum = GetVibrationSpectrum();
            float vibScore = ScoreVibration(spectrum.rmsAvg);
            report.vibrationStatus = ClassifyVibration(spectrum.rmsAvg);

            // --- Load scoring (20 %) ---
            float avgAxial  = readings.Average(r => r.axialLoad);
            float avgRadial = readings.Average(r => r.radialLoad);
            float loadScore = ScoreLoad(avgAxial, avgRadial);

            // --- Trend scoring (10 %) ---
            var trend = GetTemperatureTrend();
            float trendScore = ScoreTrend(trend);

            // --- Overall health ---
            report.overallHealth = Mathf.Clamp(
                tempScore  * WeightTemperature +
                vibScore   * WeightVibration   +
                loadScore  * WeightLoad        +
                trendScore * WeightTrend,
                0f, 100f);

            // --- Lubrication status (derived from vibration crest factor) ---
            report.lubricationStatus = ClassifyLubrication(spectrum.crestFactor);

            // --- Remaining life estimate (simple linear projection) ---
            report.estimatedRemainingHours = PredictRemainingLife(0f, 20000f);

            // --- Recommendations ---
            if (report.temperatureStatus == "critical")
                report.recommendations.Add("Immediate inspection required: bearing temperature critical.");
            else if (report.temperatureStatus == "elevated")
                report.recommendations.Add("Monitor bearing temperature closely; consider coolant check.");

            if (report.vibrationStatus == "critical")
                report.recommendations.Add("Immediate inspection required: vibration levels critical.");
            else if (report.vibrationStatus == "elevated")
                report.recommendations.Add("Schedule vibration analysis; check bearing pre-load.");

            if (report.lubricationStatus == "poor")
                report.recommendations.Add("Re-lubricate bearings immediately.");
            else if (report.lubricationStatus == "marginal")
                report.recommendations.Add("Plan bearing lubrication at next maintenance window.");

            if (trend.isRising && trend.slope > 0.5f)
                report.recommendations.Add("Temperature trend is rising; investigate root cause.");

            if (report.recommendations.Count == 0)
                report.recommendations.Add("Bearing health is within normal parameters.");

            return report;
        }

        // ── Temperature Trend ────────────────────────────────────────

        /// <summary>
        /// Compute average temperature, linear slope, and rising flag from
        /// the last <see cref="TrendWindow"/> readings (or all if fewer).
        /// </summary>
        public TemperatureTrend GetTemperatureTrend()
        {
            var result = new TemperatureTrend();

            if (readings.Count == 0)
                return result;

            var window = readings.Count <= TrendWindow
                ? readings
                : readings.GetRange(readings.Count - TrendWindow, TrendWindow);

            result.average = window.Average(r => r.temperature);

            // Simple least-squares slope (y = temperature, x = index)
            if (window.Count >= 2)
            {
                float n = window.Count;
                float sumX = 0f, sumY = 0f, sumXY = 0f, sumX2 = 0f;
                for (int i = 0; i < window.Count; i++)
                {
                    float x = i;
                    float y = window[i].temperature;
                    sumX  += x;
                    sumY  += y;
                    sumXY += x * y;
                    sumX2 += x * x;
                }
                float denom = n * sumX2 - sumX * sumX;
                result.slope = denom != 0f ? (n * sumXY - sumX * sumY) / denom : 0f;
            }

            result.isRising = result.slope > 0f;
            return result;
        }

        // ── Vibration Spectrum ───────────────────────────────────────

        /// <summary>
        /// Compute RMS average, peak average and crest factor from recent readings.
        /// </summary>
        public VibrationSpectrum GetVibrationSpectrum()
        {
            var result = new VibrationSpectrum();

            if (readings.Count == 0)
                return result;

            result.rmsAvg  = readings.Average(r => r.vibrationRms);
            result.peakAvg = readings.Average(r => r.vibrationPeak);
            result.crestFactor = result.rmsAvg > 0f
                ? result.peakAvg / result.rmsAvg
                : 0f;

            return result;
        }

        // ── Remaining Life Prediction ────────────────────────────────

        /// <summary>
        /// Estimate remaining bearing life hours based on current health
        /// degradation rate.  <paramref name="currentHours"/> is the number
        /// of operating hours already accumulated and <paramref name="maxHours"/>
        /// is the manufacturer-rated L10 life.
        /// </summary>
        public float PredictRemainingLife(float currentHours, float maxHours)
        {
            if (readings.Count == 0)
                return maxHours - currentHours;

            // Health-based degradation factor (lower health = faster degradation)
            float avgTemp = readings.Average(r => r.temperature);
            float avgVib  = readings.Average(r => r.vibrationRms);

            float tempFactor = avgTemp < TempNormalMax ? 1.0f
                             : avgTemp < TempElevatedMax ? 0.7f
                             : 0.3f;

            float vibFactor = avgVib < VibNormalMax ? 1.0f
                            : avgVib < VibElevatedMax ? 0.6f
                            : 0.2f;

            float degradationFactor = (tempFactor + vibFactor) / 2f;

            float remaining = (maxHours - currentHours) * degradationFactor;
            return Mathf.Max(remaining, 0f);
        }

        // ── Scoring Helpers ──────────────────────────────────────────

        private float ScoreTemperature(float avgTemp)
        {
            if (avgTemp < TempNormalMax)
                return 100f;
            if (avgTemp < TempElevatedMax)
                return Mathf.Lerp(100f, 50f, (avgTemp - TempNormalMax) / (TempElevatedMax - TempNormalMax));
            // critical
            return Mathf.Lerp(50f, 0f, Mathf.Clamp01((avgTemp - TempElevatedMax) / 30f));
        }

        private float ScoreVibration(float rmsAvg)
        {
            if (rmsAvg < VibNormalMax)
                return 100f;
            if (rmsAvg < VibElevatedMax)
                return Mathf.Lerp(100f, 50f, (rmsAvg - VibNormalMax) / (VibElevatedMax - VibNormalMax));
            return Mathf.Lerp(50f, 0f, Mathf.Clamp01((rmsAvg - VibElevatedMax) / 5f));
        }

        private float ScoreLoad(float axial, float radial)
        {
            float axialRatio  = Mathf.Clamp01(axial  / RatedAxialLoad);
            float radialRatio = Mathf.Clamp01(radial / RatedRadialLoad);
            float loadRatio   = Mathf.Max(axialRatio, radialRatio);
            return Mathf.Lerp(100f, 0f, loadRatio);
        }

        private float ScoreTrend(TemperatureTrend trend)
        {
            if (!trend.isRising)
                return 100f;
            // Penalise rising temperature; slope > 1 °C/reading ≈ worst case
            return Mathf.Lerp(100f, 0f, Mathf.Clamp01(trend.slope));
        }

        // ── Classification Helpers ───────────────────────────────────

        private string ClassifyTemperature(float temp)
        {
            if (temp < TempNormalMax)  return "normal";
            if (temp < TempElevatedMax) return "elevated";
            return "critical";
        }

        private string ClassifyVibration(float rms)
        {
            if (rms < VibNormalMax)  return "normal";
            if (rms < VibElevatedMax) return "elevated";
            return "critical";
        }

        private string ClassifyLubrication(float crestFactor)
        {
            // A crest factor near 1.0-1.5 indicates smooth operation (good lube).
            // Higher crest factors suggest impulsive vibration (poor lube / damage).
            if (crestFactor < 3.0f)  return "good";
            if (crestFactor < 5.0f)  return "marginal";
            return "poor";
        }
    }

    // ── Axis Backlash Compensation ──────────────────────────────────────

    /// <summary>
    /// Per-axis backlash configuration including measured backlash, applied
    /// compensation, last calibration date, and most-recent move direction.
    /// </summary>
    [Serializable]
    public class AxisBacklash
    {
        public string axisName;           // X, Y, Z, A, B, C
        public float backlashMm;          // measured mechanical backlash (mm)
        public float compensationMm;      // compensation value applied (mm)
        public string lastCalibrated;     // ISO-8601 date string
        public int direction;             // +1 or -1 for last move direction

        public AxisBacklash(string axisName, float backlashMm)
        {
            this.axisName = axisName;
            this.backlashMm = backlashMm;
            this.compensationMm = backlashMm;   // default: compensate fully
            this.lastCalibrated = DateTime.UtcNow.ToString("o");
            this.direction = 1;
        }
    }

    /// <summary>
    /// Result of a backlash verification test on a single axis.
    /// </summary>
    [Serializable]
    public class BacklashTestResult
    {
        public string axisName;
        public float measuredBacklash;
        public float appliedCompensation;
        public float residualError;
        public string testDate;
        public bool passed;
    }

    /// <summary>
    /// Manages per-axis backlash compensation values for CNC machines.
    /// Tracks mechanical backlash, applies directional compensation on
    /// direction reversals, and supports calibration verification testing.
    /// </summary>
    public class BacklashCompensationManager
    {
        private readonly Dictionary<string, AxisBacklash> _axes =
            new Dictionary<string, AxisBacklash>();

        /// <summary>Maximum allowable residual error (mm) for a test to pass.</summary>
        public const float PassThresholdMm = 0.005f;

        public BacklashCompensationManager()
        {
            // Initialise standard linear axes with a conservative default backlash.
            _axes["X"] = new AxisBacklash("X", 0.01f);
            _axes["Y"] = new AxisBacklash("Y", 0.01f);
            _axes["Z"] = new AxisBacklash("Z", 0.01f);
        }

        // ── Set / Get ───────────────────────────────────────────────────

        /// <summary>
        /// Set (or update) the backlash value for a given axis.  If the axis
        /// does not yet exist it is created.
        /// </summary>
        public void SetBacklash(string axis, float value)
        {
            if (string.IsNullOrEmpty(axis))
                throw new ArgumentException("axis must not be null or empty");
            if (value < 0f)
                throw new ArgumentException("backlash value must be non-negative");

            if (_axes.ContainsKey(axis))
            {
                _axes[axis].backlashMm = value;
                _axes[axis].compensationMm = value;
                _axes[axis].lastCalibrated = DateTime.UtcNow.ToString("o");
            }
            else
            {
                _axes[axis] = new AxisBacklash(axis, value);
            }
        }

        /// <summary>
        /// Return the current backlash configuration for an axis, or null
        /// if the axis is not registered.
        /// </summary>
        public AxisBacklash GetBacklash(string axis)
        {
            if (string.IsNullOrEmpty(axis))
                return null;
            return _axes.ContainsKey(axis) ? _axes[axis] : null;
        }

        // ── Compensation ────────────────────────────────────────────────

        /// <summary>
        /// Calculate the compensation offset to apply when moving the given
        /// axis in <paramref name="moveDirection"/> (+1 or -1).  A non-zero
        /// offset is only returned when the direction reverses.
        /// </summary>
        public float ApplyCompensation(string axis, int moveDirection)
        {
            if (string.IsNullOrEmpty(axis))
                throw new ArgumentException("axis must not be null or empty");
            if (moveDirection != 1 && moveDirection != -1)
                throw new ArgumentException("moveDirection must be +1 or -1");
            if (!_axes.ContainsKey(axis))
                throw new KeyNotFoundException($"Axis '{axis}' is not registered");

            AxisBacklash ab = _axes[axis];
            if (moveDirection != ab.direction)
            {
                ab.direction = moveDirection;
                return ab.compensationMm;
            }
            return 0f;
        }

        // ── Testing / Verification ──────────────────────────────────────

        /// <summary>
        /// Run a backlash verification test.  Compares a <paramref name="measuredValue"/>
        /// (actual backlash observed at the machine) against the currently
        /// configured compensation and returns a <see cref="BacklashTestResult"/>.
        /// </summary>
        public BacklashTestResult RunBacklashTest(string axis, float measuredValue)
        {
            if (string.IsNullOrEmpty(axis))
                throw new ArgumentException("axis must not be null or empty");
            if (!_axes.ContainsKey(axis))
                throw new KeyNotFoundException($"Axis '{axis}' is not registered");

            AxisBacklash ab = _axes[axis];
            float residual = Mathf.Abs(measuredValue - ab.compensationMm);

            return new BacklashTestResult
            {
                axisName = axis,
                measuredBacklash = measuredValue,
                appliedCompensation = ab.compensationMm,
                residualError = residual,
                testDate = DateTime.UtcNow.ToString("o"),
                passed = residual <= PassThresholdMm
            };
        }

        // ── Query helpers ───────────────────────────────────────────────

        /// <summary>Return all registered axis configurations.</summary>
        public List<AxisBacklash> GetAllAxes()
        {
            return new List<AxisBacklash>(_axes.Values);
        }

        /// <summary>
        /// Check whether an axis's calibration is older than
        /// <paramref name="maxAgeDays"/> days.
        /// </summary>
        public bool NeedsRecalibration(string axis, int maxAgeDays)
        {
            if (string.IsNullOrEmpty(axis))
                throw new ArgumentException("axis must not be null or empty");
            if (!_axes.ContainsKey(axis))
                throw new KeyNotFoundException($"Axis '{axis}' is not registered");

            DateTime lastCal = DateTime.Parse(
                _axes[axis].lastCalibrated,
                null,
                System.Globalization.DateTimeStyles.RoundtripKind);
            return (DateTime.UtcNow - lastCal).TotalDays > maxAgeDays;
        }

        /// <summary>
        /// Sum of compensation values across every registered axis.
        /// </summary>
        public float GetTotalCompensation()
        {
            float total = 0f;
            foreach (var ab in _axes.Values)
                total += ab.compensationMm;
            return total;
        }
    }

    // ── Rest Machining Detector ─────────────────────────────────────

    /// <summary>
    /// Describes a single region of unmachined (rest) material left by a
    /// larger tool that requires cleanup with a smaller tool.
    /// </summary>
    [Serializable]
    public class RestRegion
    {
        public Vector3 center;
        public Vector3 extentMm;
        public float volumeMm3;
        public float cornerRadius;
        public float requiredToolDiaMm;
        public float depth;
    }

    /// <summary>
    /// Aggregate analysis of rest-material regions produced by the detector.
    /// </summary>
    [Serializable]
    public class RestAnalysis
    {
        public List<RestRegion> regions = new List<RestRegion>();
        public float totalRestVolumeMm3;
        public float largestRegionVolumeMm3;
        public float suggestedToolDiaMm;
        public float estimatedCleanupTimeMin;
        public int regionCount;
    }

    /// <summary>
    /// Detects unmachined material (rest material) left by larger tools that
    /// needs cleanup with smaller tools.  Analyses internal corners, fillet
    /// regions, and recommends appropriate cleanup tooling and time estimates.
    /// </summary>
    public class RestMachiningDetector
    {
        // ── Corner rest-volume calculation ─────────────────────────────

        /// <summary>
        /// Calculate the volume of rest material left in a single internal
        /// corner by a tool of diameter <paramref name="toolDia"/>.
        /// The corner has the given <paramref name="cornerRadius"/> and
        /// <paramref name="depth"/>.
        ///
        /// The rest material is the difference between the square corner
        /// and the arc swept by the tool:
        ///   V = (R^2 - pi/4 * R^2) * depth   where R = toolDia / 2
        /// but only the portion beyond the desired corner radius matters.
        /// When the tool radius equals the corner radius, rest volume is zero.
        /// </summary>
        public float GetRestVolume(float toolDia, float cornerRadius, float depth)
        {
            if (toolDia <= 0f || depth <= 0f)
                return 0f;

            float toolRadius = toolDia / 2f;

            // If the tool fits within the corner radius, no rest material
            if (toolRadius <= cornerRadius)
                return 0f;

            // Rest area in cross-section: quarter-circle area of tool minus
            // quarter-circle area that the desired corner would occupy.
            // A_rest = (1 - pi/4) * R_tool^2 - (1 - pi/4) * R_corner^2
            //        = (1 - pi/4) * (R_tool^2 - R_corner^2)
            float factor = 1f - Mathf.PI / 4f;
            float area = factor * (toolRadius * toolRadius - cornerRadius * cornerRadius);

            return area * depth;
        }

        // ── Analyse internal corners of a rectangular pocket ──────────

        /// <summary>
        /// Find rest-material regions in the four internal corners of a
        /// rectangular pocket machined with the given tool diameter.
        /// </summary>
        public RestAnalysis AnalyzeCorners(
            float pocketWidth, float pocketLength, float pocketDepth, float toolDiameter)
        {
            RestAnalysis analysis = new RestAnalysis();

            if (pocketWidth <= 0f || pocketLength <= 0f ||
                pocketDepth <= 0f || toolDiameter <= 0f)
                return analysis;

            float toolRadius = toolDiameter / 2f;
            float cornerRadius = 0f; // sharp internal corners

            float vol = GetRestVolume(toolDiameter, cornerRadius, pocketDepth);
            if (vol <= 0f)
                return analysis;

            // Four corners of the pocket
            Vector3[] corners = new Vector3[]
            {
                new Vector3(0f, 0f, 0f),
                new Vector3(pocketWidth, 0f, 0f),
                new Vector3(pocketWidth, pocketLength, 0f),
                new Vector3(0f, pocketLength, 0f)
            };

            foreach (Vector3 c in corners)
            {
                RestRegion region = new RestRegion
                {
                    center = c,
                    extentMm = new Vector3(toolRadius, toolRadius, pocketDepth),
                    volumeMm3 = vol,
                    cornerRadius = toolRadius, // radius left by the tool
                    requiredToolDiaMm = toolRadius, // need a tool with radius < toolRadius
                    depth = pocketDepth
                };
                analysis.regions.Add(region);
            }

            analysis.regionCount = analysis.regions.Count;
            analysis.totalRestVolumeMm3 = vol * analysis.regionCount;
            analysis.largestRegionVolumeMm3 = vol;
            analysis.suggestedToolDiaMm = SuggestCleanupTool(toolRadius);
            analysis.estimatedCleanupTimeMin = 0f;

            return analysis;
        }

        // ── Analyse fillet regions ────────────────────────────────────

        /// <summary>
        /// Detect rest material along a fillet where the tool diameter is
        /// too large to machine the desired fillet radius.  The rest region
        /// is spread along the <paramref name="pathLength"/>.
        /// </summary>
        public RestAnalysis AnalyzeFillets(
            float filletRadius, float toolDiameter, float pathLength)
        {
            RestAnalysis analysis = new RestAnalysis();

            if (filletRadius <= 0f || toolDiameter <= 0f || pathLength <= 0f)
                return analysis;

            float toolRadius = toolDiameter / 2f;

            if (toolRadius <= filletRadius)
                return analysis; // tool can already machine the fillet

            // Rest cross-section area — same (1 - pi/4) approach
            float factor = 1f - Mathf.PI / 4f;
            float restArea = factor * (toolRadius * toolRadius - filletRadius * filletRadius);
            float vol = restArea * pathLength;

            RestRegion region = new RestRegion
            {
                center = new Vector3(pathLength / 2f, 0f, 0f),
                extentMm = new Vector3(pathLength, toolRadius - filletRadius, 0f),
                volumeMm3 = vol,
                cornerRadius = toolRadius,
                requiredToolDiaMm = filletRadius * 2f,
                depth = pathLength
            };

            analysis.regions.Add(region);
            analysis.regionCount = 1;
            analysis.totalRestVolumeMm3 = vol;
            analysis.largestRegionVolumeMm3 = vol;
            analysis.suggestedToolDiaMm = SuggestCleanupTool(filletRadius);
            analysis.estimatedCleanupTimeMin = 0f;

            return analysis;
        }

        // ── Tool recommendation ──────────────────────────────────────

        /// <summary>
        /// Suggest a cleanup tool diameter that can reach into corners
        /// with the given <paramref name="maxRestCornerRadius"/>.
        /// The tool diameter must be less than 2 * cornerRadius.
        /// Returns 80 % of the theoretical maximum for safety margin.
        /// </summary>
        public float SuggestCleanupTool(float maxRestCornerRadius)
        {
            if (maxRestCornerRadius <= 0f)
                return 0f;

            // Maximum tool diameter that can fit: 2 * cornerRadius
            // Apply 80 % safety factor
            return 2f * maxRestCornerRadius * 0.8f;
        }

        // ── Cleanup time estimation ──────────────────────────────────

        /// <summary>
        /// Estimate the time required to machine the given rest regions
        /// at the specified feed rate and stepover.
        /// </summary>
        public float EstimateCleanupTime(
            List<RestRegion> regions, float feedRateMmPerMin, float stepoverMm)
        {
            if (regions == null || regions.Count == 0)
                return 0f;
            if (feedRateMmPerMin <= 0f || stepoverMm <= 0f)
                return 0f;

            float totalTime = 0f;

            foreach (RestRegion region in regions)
            {
                // Approximate the number of passes needed to cover the rest area
                float width = Mathf.Max(region.extentMm.x, region.extentMm.y);
                float passCount = Mathf.Ceil(width / stepoverMm);

                // Each pass traverses the depth direction
                float passLength = region.depth;

                // Total path length for this region
                float pathLength = passCount * passLength;

                // Time = distance / feed rate
                totalTime += pathLength / feedRateMmPerMin;
            }

            return totalTime;
        }
    }
}
