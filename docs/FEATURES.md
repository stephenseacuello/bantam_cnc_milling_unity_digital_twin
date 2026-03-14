# MIRACLE Features Reference

Quick-reference catalog of every major capability in the MIRACLE CNC milling digital twin system. Each entry states what the feature does, where to find it in the codebase, and any closely related subsystems.

All ROS 2 package paths are relative to `miracle_ws/src/`. Unity paths are relative to the repository root.

---

## Predictive Digital Twin

### Cutting Force Simulation

- **What**: Altintas mechanistic cutting force model. Default coefficients for 6061-T6 / HSS: Ktc=796 N/mm^2, Krc=168 N/mm^2, Kac=80 N/mm^2. Computes tangential, radial, and axial forces per tooth per revolution.
- **Where**: `miracle_twin/miracle_twin/cutting_sim_proxy.py` -- `CuttingSimProxy`; `unity_twin/Assets/Scripts/Cutting/CuttingForceEngine.cs`
- **Related**: Tool deflection via cantilever beam model, surface roughness prediction in `unity_twin/Assets/Scripts/Cutting/SurfaceRoughnessModel.cs`

### Tool Wear Prediction

- **What**: Extended Taylor tool-life equation (V * T^0.125 * f^0.5 * ap^0.15 = 300). Three-stage flank wear progression (break-in, steady-state, accelerated). End-of-life threshold VBmax = 0.30 mm.
- **Where**: `miracle_twin/miracle_twin/cutting_sim_proxy.py` -- `CuttingSimProxy.calculate_wear`; `unity_twin/Assets/Scripts/Cutting/ToolWearModel.cs`

### Thermal Modeling

- **What**: Five-zone lumped-parameter thermal model (spindle, workpiece, tool_holder, coolant, ambient) with inter-zone conduction, convection, and heat generation from cutting.
- **Where**: `miracle_twin/miracle_twin/cutting_sim_proxy.py` -- `ThermalModel`; `unity_twin/Assets/Scripts/Cutting/ThermalModel.cs`

### Chatter Prediction

- **What**: Zeroth-Order Approximation (ZOA) stability lobe theory. Critical axial depth limit: ap_lim = -1 / (2 * Ktc * N * Re[G(jw)]). Generates stability lobe diagrams for spindle speed selection.
- **Where**: `unity_twin/Assets/Scripts/Cutting/StabilityLobePredictor.cs`

### G-Code Lookahead

- **What**: 50-block predictive scan that pre-computes force, power, temperature, wear, and chatter risk for each upcoming block before execution.
- **Where**: `unity_twin/Assets/Scripts/Cutting/GCodeLookahead.cs`

### Collision Detection

- **What**: Fixture AABB collision checking with 2 mm safety margin on rapid moves. Arc moves are sampled at 8 points to catch mid-arc interference.
- **Where**: `unity_twin/Assets/Scripts/Cutting/GCodeLookahead.cs` -- `CheckFixtureCollisions`
- **Related**: Fixture definitions in `unity_twin/Assets/Scripts/Cutting/FixtureProfile.cs`

### Path Smoothing

- **What**: Corner analysis with trapezoidal feed profiling and bidirectional velocity planning. Smooths acceleration transitions between adjacent G-code blocks.
- **Where**: `unity_twin/Assets/Scripts/Cutting/GCodeLookahead.cs` -- `ToolPathSmoother`

### Vibration Analysis

- **What**: DFT-based spectrum analysis of spindle vibration. Detects chatter (non-harmonic peaks), estimates tool runout, and assesses bearing health.
- **Where**: `unity_twin/Assets/Scripts/Cutting/StabilityLobePredictor.cs` -- `SpindleVibrationAnalyzer`

### Block Telemetry

- **What**: Per-block actual-vs-predicted force/temperature tracking with drift detection and automatic calibration triggers when prediction error exceeds threshold.
- **Where**: `miracle_twin/miracle_twin/block_telemetry.py` -- `BlockTelemetryTracker`

### Anomaly Markers

- **What**: Risk classification labels applied to lookahead blocks: FORCE_WARNING, FORCE_CRITICAL, THERMAL, WEAR, CHATTER, SURFACE_QUALITY, TOOL_END_OF_LIFE.
- **Where**: `miracle_twin/miracle_twin/prediction_runner.py` -- `_identify_anomaly_risks`

### Program Optimization

- **What**: Per-block feed and speed optimization using force headroom analysis. Identifies blocks where feedrate can be increased or must be reduced.
- **Where**: `miracle_twin/miracle_twin/cutting_sim_proxy.py` -- `CuttingSimProxy.optimize_program`

### Chip Load Monitor

- **What**: Chip thinning correction for radial engagement, material removal rate (MRR) calculation, specific cutting energy, and recutting risk detection.
- **Where**: `miracle_twin/miracle_twin/cutting_sim_proxy.py` -- `ChipLoadMonitor`

### Workholding Analysis

- **What**: Compares clamping force against cutting force resultant. Evaluates lift-off risk, rotation risk, and generates setup recommendations for fixture placement.
- **Where**: `miracle_twin/miracle_twin/cutting_sim_proxy.py` -- `WorkholdingAnalyzer`

### Coolant Optimization

- **What**: Material-aware coolant strategy recommendation (dry, mist, flood, high_pressure, cryogenic) with cost scoring and environmental impact rating.
- **Where**: `miracle_twin/miracle_twin/cutting_sim_proxy.py` -- `CoolantOptimizer`

### Geometric Tolerance (GD&T)

- **What**: Supports 10 GD&T characteristic types. Predicts deviations from thermal expansion and tool deflection. Classifies tolerance risk as OK, WARNING, or VIOLATION.
- **Where**: `unity_twin/Assets/Scripts/Cutting/CuttingSimulationManager.cs` -- `GeometricToleranceAnalyzer`

### Material Database

- **What**: Ten pre-loaded workpiece materials: 6061-T6, 7075-T6, 1018, 4140, D2, 304-SS, 316-SS, Ti-6Al-4V, Inconel-718, PEEK. Each entry carries cutting coefficients, thermal properties, and recommended parameters.
- **Where**: `miracle_twin/miracle_twin/tool_library.py` -- `MaterialDatabase`; `unity_twin/Assets/Scripts/Cutting/MaterialDatabase.cs`

### Tool Calibration

- **What**: EMA-based per-machine calibration of cutting force coefficients. Calibration state persists as JSON so it survives restarts.
- **Where**: `miracle_twin/miracle_twin/tool_library.py` -- `ToolCalibrationData`

---

## Situational Awareness

### Alert Correlation

- **What**: Groups alerts within a configurable time window. Applies YAML-defined correlation rules and enriches alerts with G-code context. Detects recurring alerts on the same G-code block across runs.
- **Where**: `miracle_scada/miracle_scada/alert_correlator.py` -- `AlertCorrelatorNode`

### Anomaly Pattern Library

- **What**: Stores learned failure signatures and matches new anomaly clusters against them (>70% similarity threshold). Persists patterns in JSONL format.
- **Where**: `miracle_scada/miracle_scada/alert_correlator.py` -- `AnomalyPatternLibrary`

### Root Cause Analysis

- **What**: Bayesian evidence scoring across eight failure modes: TOOL_WEAR, CHATTER, THERMAL_DRIFT, COOLANT_FAILURE, SPINDLE_BEARING, MATERIAL_DEFECT, PROGRAMMING_ERROR, FIXTURING_ISSUE.
- **Where**: `miracle_cognitive/miracle_cognitive/interface/explanation_generator.py` -- `RootCauseAnalyzer`

### Explainable AI

- **What**: Three-level explanations (summary, detail, counterfactual) with per-feature contribution scores so operators understand why a prediction was made.
- **Where**: `miracle_cognitive/miracle_cognitive/interface/explanation_generator.py` -- `ExplanationGeneratorNode`

### Fleet Overview

- **What**: Multi-machine dashboard grid showing health rings, remaining useful life (RUL) bars, OEE trend sparklines, and utilization heatmaps.
- **Where**: `unity_twin/Assets/Scripts/UI/FleetOverviewPanel.cs`

### Decision Support

- **What**: Ranked recommendations panel with causal trajectory preview, what-if parameter slider, and spindle load balancing across the fleet.
- **Where**: `unity_twin/Assets/Scripts/UI/DecisionSupportPanel.cs`

### Shift Reports

- **What**: Automated end-of-shift OEE summary, pending issue list, recommended actions, and shift-over-shift comparison.
- **Where**: `miracle_scada/miracle_scada/kpi_calculator.py` -- `ShiftReportGenerator`

### Alarm Escalation

- **What**: Configurable multi-level escalation policies (NOTIFY, PAGE, AUTO_STOP, LOCKOUT). Supports auto-acknowledge of stale alarms and escalation timers.
- **Where**: `miracle_scada/miracle_scada/alarm_manager.py` -- `EscalationEngine`

### Notifications

- **What**: Multi-channel dispatch (dashboard, email, SMS, MQTT, webhook) with quiet-hours enforcement, per-channel rate limiting, and deduplication.
- **Where**: `miracle_scada/miracle_scada/alarm_manager.py` -- `NotificationDispatcher`

### Action Ranking

- **What**: Multi-criteria scoring of recommended actions. Weights: risk reduction 0.35, RUL impact 0.25, cycle time 0.20, surface quality 0.10, confidence 0.10.
- **Where**: `miracle_cognitive/miracle_cognitive/interface/action_ranker.py`

### Process Capability

- **What**: Calculates Cp, Cpk, Pp, Ppk. Maintains control charts with Western Electric run rules. Predicts scrap rate from capability indices.
- **Where**: `miracle_scada/miracle_scada/kpi_calculator.py` -- `CapabilityProfiler`

---

## Closed-Loop Control

### Adaptive Feedrate

- **What**: Hysteresis state machine with states NORMAL, FORCE_LIMITED, CHATTER_LIMITED, THERMAL_LIMITED, WEAR_LIMITED. 5-second debounce prevents oscillation; feed ramps at 5% per cycle.
- **Where**: `miracle_twin/miracle_twin/adaptive_controller.py` -- `AdaptiveControllerNode`

### Preemptive Control

- **What**: Consumes anomaly markers from the lookahead and applies feed/speed overrides N blocks ahead of the problem, preventing threshold violations before they occur.
- **Where**: `miracle_twin/miracle_twin/adaptive_controller.py` -- `_process_anomaly_markers`

### Causal Simulation

- **What**: Physics-informed transfer functions for what-if analysis. Example elasticities: feed to force 0.8, feed to surface roughness 2.0, feed to tool life -2.0.
- **Where**: `miracle_cognitive/miracle_cognitive/knowledge/causal_inference.py` -- `simulate_intervention`

### G-Code Macros

- **What**: M98/G65 call syntax with parameter substitution and O-word flow control parsing. Ships with four built-in macros. Nesting is depth-limited to prevent runaway expansion.
- **Where**: `miracle_cnc/miracle_cnc/gcode_executor.py` -- `MacroLibrary`
- **Related**: Unity-side execution in `unity_twin/Assets/Scripts/Cutting/GCodeExecutor.cs`

---

## Manufacturing Intelligence

### Digital Thread

- **What**: SHA-256 hash-chained genealogy record per part. Tracks material lot, tool serial, prediction snapshots, and energy consumption through the full production lifecycle.
- **Where**: `miracle_mes/miracle_mes/digital_thread.py` -- `DigitalThreadNode`

### Energy Tracking

- **What**: Subsystem-level power integration (spindle, axes, coolant, auxiliary). Computes carbon footprint, idle-power waste, and enables program-vs-program energy comparison.
- **Where**: `miracle_mes/miracle_mes/digital_thread.py` -- `EnergyTracker`

### Job Scheduling

- **What**: Priority-queue job dispatcher. Alarms can pause or block individual jobs. Integrates with the maintenance scheduler to avoid conflicts.
- **Where**: `miracle_mes/miracle_mes/job_scheduler.py` -- `JobSchedulerNode`

### Predictive Maintenance

- **What**: Triggers maintenance work orders automatically when RUL drops below threshold or prediction drift exceeds bounds. Can interrupt running jobs if severity warrants it.
- **Where**: `miracle_mes/miracle_mes/job_scheduler.py` -- `MaintenanceScheduler`

### Queue Optimization

- **What**: Multi-factor scoring of queued jobs (urgency, setup affinity, tool availability). Batches jobs that share the same setup to minimize changeover time.
- **Where**: `miracle_mes/miracle_mes/job_scheduler.py` -- `JobQueueOptimizer`

### OEE

- **What**: Overall Equipment Effectiveness = Availability x Performance x Quality. Quality factor incorporates predictive quality signals from anomaly markers, not just post-inspection rejects.
- **Where**: `miracle_scada/miracle_scada/kpi_calculator.py` -- `KPICalculatorNode`

---

## Security and Resiliency

### G-Code Signing

- **What**: Ed25519 digital signatures on G-code files. Includes a CLI signing tool and a runtime verifier that rejects unsigned or tampered programs before execution.
- **Where**: `miracle_security/miracle_security/gcode_signer.py`; `unity_twin/Assets/Scripts/Cutting/GCodeSignatureVerifier.cs`

### Encrypted Audit Log

- **What**: AES-256-GCM encryption of audit entries. Ed25519-signed hash chain for tamper detection. Supports scheduled key rotation.
- **Where**: `miracle_security/miracle_security/secure_storage.py` -- `SecureStorage`

### Intrusion Detection (IDS)

- **What**: Monitors ROS 2 traffic for rate anomalies, burst patterns, oversized payloads, and high-entropy content. Auto-quarantines suspicious nodes.
- **Where**: `miracle_security/miracle_security/intrusion_detection.py` -- `IntrusionDetectionNode`

### Network Partition Detection

- **What**: Heartbeat-based partition detector classifying connectivity as FULL, PARTIAL, or INTERMITTENT. Selects recovery strategy based on partition type.
- **Where**: `miracle_resiliency/miracle_resiliency/recovery_orchestrator.py` -- `PartitionDetector`

### Chaos Engineering

- **What**: Fault injection framework supporting network delay, node kill, CPU/memory stress, and message drop scenarios. Automatic cleanup on test completion or timeout.
- **Where**: `miracle_resiliency/miracle_resiliency/chaos_injector.py`; `miracle_resiliency/miracle_resiliency/fault_executor.py`

### Recovery Orchestration

- **What**: Manages lifecycle transitions (deactivate, cleanup, configure, activate) for failed nodes. Uses topological sort for dependency-aware restart ordering. Retries with backoff.
- **Where**: `miracle_resiliency/miracle_resiliency/recovery_orchestrator.py` -- `RecoveryOrchestratorNode`

---

## Visualization and UI

### Voxel Material Removal

- **What**: Real-time volumetric subtraction of the workpiece as the tool moves along the G-code path. Mesh regenerated via marching cubes.
- **Where**: `unity_twin/Assets/Scripts/Cutting/VoxelWorkpiece.cs`; `unity_twin/Assets/Scripts/Cutting/VoxelGridData.cs`; `unity_twin/Assets/Scripts/Cutting/MarchingCubesRenderer.cs`

### Force Arrow Renderer

- **What**: 3D vector arrows on the tool tip showing real-time cutting force direction and magnitude.
- **Where**: `unity_twin/Assets/Scripts/Visualization/ForceArrowRenderer.cs`

### Heat Map Overlay

- **What**: Color-coded thermal overlay on the workpiece surface driven by the thermal model.
- **Where**: `unity_twin/Assets/Scripts/Visualization/HeatMapOverlay.cs`

### Surface Roughness Overlay

- **What**: Visual roughness texture on machined faces reflecting predicted Ra values.
- **Where**: `unity_twin/Assets/Scripts/Visualization/SurfaceRoughnessOverlay.cs`

### Chip Particle Effects

- **What**: Particle system representing chip ejection, scaled by MRR and chip type.
- **Where**: `unity_twin/Assets/Scripts/Visualization/ChipParticleController.cs`

### Toolpath Preview

- **What**: 3D line rendering of upcoming G-code moves color-coded by feedrate or risk level.
- **Where**: `unity_twin/Assets/Scripts/Visualization/ToolpathPreview.cs`

### Wear Indicator

- **What**: Visual indicator on the tool showing current flank wear stage and remaining life.
- **Where**: `unity_twin/Assets/Scripts/Visualization/WearIndicator.cs`

### Dashboard Overlay

- **What**: Real-time charts for force, thermal, tool wear, and power. Alert timeline panel and notification toasts.
- **Where**: `unity_twin/Assets/Scripts/UI/DashboardOverlay.cs`; `unity_twin/Assets/Scripts/UI/ForceChart.cs`; `unity_twin/Assets/Scripts/UI/ThermalChart.cs`; `unity_twin/Assets/Scripts/UI/ToolWearChart.cs`; `unity_twin/Assets/Scripts/UI/PowerChart.cs`; `unity_twin/Assets/Scripts/UI/AlertTimelinePanel.cs`; `unity_twin/Assets/Scripts/UI/NotificationToastManager.cs`

### Runtime File Browser

- **What**: In-app file browser for loading G-code programs at runtime without restarting the simulation.
- **Where**: `unity_twin/Assets/Scripts/UI/RuntimeFileBrowser.cs`

### G-Code Editor

- **What**: In-app G-code text editor with syntax awareness.
- **Where**: `unity_twin/Assets/Scripts/UI/GCodeEditor.cs`

### Simulation Control Panel

- **What**: Play/pause/step controls, speed multiplier, and mode selection (real-time, accelerated, replay).
- **Where**: `unity_twin/Assets/Scripts/UI/SimulationControlPanel.cs`

### Multi-Machine Switching

- **What**: Runtime selection between machine profiles (Bantam Explorer, Coast Runner CR-1).
- **Where**: `unity_twin/Assets/Scripts/CNC/CNCMachineSelector.cs`

### Replay Mode

- **What**: Records simulation state and allows full session playback with scrubbing.
- **Where**: `unity_twin/Assets/Scripts/Core/DataRecorder.cs`; `unity_twin/Assets/Scripts/Core/ReplayController.cs`

---

## Audio

### Cutting Sound

- **What**: Procedural cutting audio driven by engagement, force, and chatter state.
- **Where**: `unity_twin/Assets/Scripts/Audio/CuttingSoundController.cs`

### Spindle Sound

- **What**: Procedural spindle audio scaled by RPM with harmonic content.
- **Where**: `unity_twin/Assets/Scripts/Audio/SpindleSoundController.cs`

---

## Infrastructure

### ROS 2 Bridge

- **What**: WebSocket bridge between Unity and the ROS 2 graph, transporting all custom message types.
- **Where**: `miracle_ws/src/miracle_unity_bridge/`; `unity_twin/Assets/Scripts/Core/MessageDispatcher.cs`

### Simulation Clock

- **What**: Deterministic clock for synchronized time across all Unity subsystems. Supports pause, step, and variable speed.
- **Where**: `unity_twin/Assets/Scripts/Core/SimulationClock.cs`

### Configuration

- **What**: Externalized YAML/JSON config for all tunable parameters, loaded at startup.
- **Where**: `unity_twin/Assets/Scripts/Core/MiracleConfig.cs`

### Performance Monitor

- **What**: Frame-time and memory profiling with GPU fallback detection.
- **Where**: `unity_twin/Assets/Scripts/Core/PerformanceMonitor.cs`

### Prometheus Exporter

- **What**: Exposes ROS 2 node metrics in Prometheus format for Grafana dashboards.
- **Where**: `miracle_scada/miracle_scada/prometheus_exporter.py`

### Historian

- **What**: Time-series storage of all sensor and prediction data for post-run analysis.
- **Where**: `miracle_scada/miracle_scada/historian.py`

### Discovery Server

- **What**: ROS 2 node discovery and registration service.
- **Where**: `miracle_scada/miracle_scada/discovery_server.py`

### Sensor Fusion

- **What**: Combines multiple sensor streams into a fused state estimate for the CNC machine.
- **Where**: `miracle_cnc/miracle_cnc/sensor_fusion.py`

### Local Watchdog

- **What**: Per-node health monitor that detects hangs and triggers recovery.
- **Where**: `miracle_cnc/miracle_cnc/local_watchdog.py`

### SPC Monitor

- **What**: Statistical process control monitor for in-process quality tracking.
- **Where**: `miracle_cnc/miracle_cnc/spc_monitor.py`
