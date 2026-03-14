# Changelog

All notable changes to the MIRACLE CNC Milling Digital Twin will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

## [0.4.0] - 2026-03-14

### Added

**Predictive Digital Twin**
- Real cutting simulation proxy (Altintas force model, Taylor wear, Python port)
- G-code lookahead engine with collision detection and path smoothing
- Chatter prediction via ZOA stability lobe theory with wear adjustment
- Tool deflection model (cantilever beam) and surface roughness prediction
- Multi-zone thermal model (5 zones: spindle, workpiece, tool_holder, coolant, ambient)
- Block telemetry tracking with drift detection and auto-calibration
- Predictive anomaly markers with force/thermal/wear/chatter/surface risk identification
- Program-level optimization with per-block feed/speed suggestions
- Chip load monitoring with thinning correction, MRR, and recutting risk
- Workholding force analysis with lift-off and rotation risk detection
- Tool path corner analysis with trapezoidal feed profiling
- Spindle vibration spectrum analysis with DFT-based chatter detection

**Situational Awareness**
- Alert correlation engine with YAML-configurable rules and G-code context
- Anomaly pattern library with learned failure signatures and JSONL persistence
- Root cause analyzer with Bayesian evidence scoring across 8 CNC failure modes
- Fleet overview panel with predictive health data, RUL bars, and OEE trending
- Decision support panel with causal trajectory preview and what-if simulation
- Machine utilization heatmap with state timeline and idle analysis
- Operator shift handoff reports with OEE summary and recommendations
- Multi-channel notification dispatcher (dashboard, email, SMS, MQTT, webhook)
- Alarm escalation policy engine with configurable multi-level escalation
- Explainable AI with 3-level explanations and feature contribution ranking
- Action ranking with multi-criteria cost-benefit scoring

**Closed-Loop Control**
- Adaptive feedrate controller with hysteresis state machine and debounce
- Preemptive control from anomaly markers (N blocks ahead)
- Forward causal simulation with physics-based transfer functions
- Coolant optimization advisory with material-aware recommendations
- G-code macro expansion engine (M98/G65 with parameter substitution)

**Manufacturing Intelligence**
- Process capability profiler (Cp/Cpk/Pp/Ppk) with Western Electric rules
- Predictive OEE with anomaly-driven quality/scrap prediction
- Energy consumption tracker with carbon footprint and idle waste detection
- Material genealogy in digital thread (batch traceability)
- Predictive maintenance scheduler with RUL-triggered auto-scheduling
- Job queue priority optimizer with setup batching and bottleneck identification
- Workpiece material database (10 pre-loaded materials with cutting parameters)
- Tool calibration data with EMA learning and per-machine JSON persistence
- Prediction tracking in digital thread with accuracy trending
- Geometric tolerance (GD&T) analysis for 10 tolerance types

**Security & Resiliency**
- G-code Ed25519 signing and verification
- Secure audit storage with AES-256-GCM encryption and hash chain verification
- Network partition detector with heartbeat-based classification
- Recovery orchestrator with real lifecycle transitions
- Key rotation and chain compaction for audit logs
- Operator feedback loop with acceptance/effectiveness tracking

**Knowledge & Reasoning**
- Knowledge graph with JSONL persistence and atomic writes
- Causal inference with temporal evidence decay and threading safety (RLock)
- Reasoning engine with simulated actions and physics-based outcomes

**Testing & Infrastructure**
- Test suite grown from 547 to 2739 tests (5x increase)
- miracle_core/test conftest for mock pollution prevention
- miracle_security/test conftest for submodule restoration
- GitHub Actions CI/CD pipeline (test, lint, Docker build)
- Root README with architecture overview and quick start
- Pinned Python dependencies (requirements.txt, requirements-dev.txt)
- Kubernetes RBAC, NetworkPolicy, PDB, HPA, Secrets
- Assembly definition splitting (Editor, Testing, Testing.Editor)
- Centralized logging with Grafana Loki and Promtail
- G-Code validator with machine limit checks
- ROS2 API reference documentation
- Performance benchmark suite
- Null safety validation (OnValidate) for event channels
- Apache 2.0 LICENSE file
- This CHANGELOG

## [0.3.0] - 2026-03-05

### Added
- Runtime G-code file browser for standalone builds
- Multi-machine switching with chart clearing and visualization toggling
- Digital twin replay mode (record/replay buttons, timeline scrubber)
- Prometheus metrics exporter node
- Grafana dashboards with auto-provisioning
- Configuration externalization (YAML defaults, env var overrides, ScriptableObject)
- 100+ new Python tests (Kafka, OPC-UA, IDS, e2e pipeline, HMI bridge)
- Unity dashboard integration tests (UXML element validation)
- ProfilerMarker instrumentation on hot paths
- Voxel metrics in PerformanceMonitor

### Fixed
- GPU compute shader fallback for unsupported hardware
- DashboardWiring no-machine warning
- HMI bridge RuntimeError during event loop shutdown
- Keyboard input guard for UI focus
- E-STOP toggle behavior

## [0.2.0] - 2026-03-04

### Added
- ROS2 lifecycle nodes with heartbeat mixin
- Multi-machine subscription support
- Kubernetes manifests (namespace, deployment, StatefulSet, ConfigMap)
- Circuit breaker pattern for resilient connections
- Lifecycle autostart utility
- Docker Compose orchestration (ROS2, micro-ROS, MQTT, Kafka, ZooKeeper)

## [0.1.0] - 2026-03-04

### Added
- Initial Unity digital twin for Bantam Tools Explorer CNC
- G-code execution pipeline with real-time simulation
- Voxel-based material removal with GPU compute shaders
- Marching cubes mesh rendering
- Dashboard UI with force, thermal, wear, and power charts
- ROS2 bridge (ROS-TCP-Connector) with ScriptableObject event system
- Coast Runner CR-1 CNC model support
- Robot arm controllers (Niryo Ned2, xArm6)
- Multi-agent task coordination
- Cutting force, thermal, tool wear, surface roughness models
- Audio feedback (cutting and spindle sounds)
- Visualization overlays (heat map, force arrows, surface roughness)
