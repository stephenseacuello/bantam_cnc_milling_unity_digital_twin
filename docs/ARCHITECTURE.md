# MIRACLE Architecture Overview

This document describes the architecture of the MIRACLE (Manufacturing Intelligence, Resilience, Autonomy, Cognition, Learning Engine) CNC milling digital twin system. It is intended for developers who want to understand, debug, or extend the system.

---

## 1. System Architecture

MIRACLE is a two-tier system: a Unity 3D digital twin for real-time visualization and a ROS2 Jazzy backend organized into five ISA-95-inspired layers.

```
┌─────────────────────────────────────────────────────┐
│                  Unity Digital Twin                  │
│  ┌─────────┐ ┌──────────┐ ┌────────────┐           │
│  │ Voxel   │ │ G-Code   │ │ Dashboard  │           │
│  │ Cutting │ │ Executor │ │ UI Panels  │           │
│  │ Engine  │ │+ Lookahead│ │            │           │
│  └────┬────┘ └────┬─────┘ └─────┬──────┘           │
│       │           │             │                    │
│  ┌────┴───────────┴─────────────┴──────┐            │
│  │          MiracleBridge (TCP)          │            │
│  └──────────────────┬───────────────────┘            │
└─────────────────────┼────────────────────────────────┘
                      │ ROS-TCP-Connector (port 10000)
┌─────────────────────┼────────────────────────────────┐
│                ROS2 Jazzy Backend                     │
│  ┌─────────┐ ┌──────────┐ ┌──────────┐ ┌─────────┐ │
│  │ L1: CNC │ │L2: SCADA │ │ L3: MES  │ │L4: Sec  │ │
│  │ Control │ │ + AI     │ │ + Twin   │ │+ Resil  │ │
│  └─────────┘ └──────────┘ └──────────┘ └─────────┘ │
│                    ┌──────────┐                      │
│                    │L5: Cogn  │                      │
│                    │ Autonomy │                      │
│                    └──────────┘                      │
└──────────────────────────────────────────────────────┘
```

The Unity application connects to the ROS2 backend over TCP via `ROS-TCP-Connector` (Unity side) and `miracle_unity_bridge` (ROS2 side). All ROS2 messages flow through this single bridge, and `MessageDispatcher` on the Unity side routes them to the correct subscribers.

---

## 2. ROS2 Package Map

All packages live under `miracle_ws/src/`. Every node extends `MiracleLifecycleNode` (from `miracle_core`) unless noted otherwise.

| Package | Layer | Purpose | Key Modules | Key Classes |
|---------|-------|---------|-------------|-------------|
| `miracle_core` | L0 | Shared infrastructure: lifecycle base, QoS, config, heartbeat | `lifecycle_node_base.py`, `heartbeat_mixin.py`, `qos_profiles.py`, `exceptions.py`, `config_loader.py` | `MiracleLifecycleNode`, `HeartbeatMixin`, `ConfigLoader` |
| `miracle_cnc` | L1 | G-code execution with macro expansion, machine state publishing, sensor fusion | `gcode_executor.py`, `state_publisher.py`, `sensor_fusion.py`, `spc_monitor.py`, `local_watchdog.py` | `GCodeExecutor`, `StatePublisher`, `SensorFusion` |
| `miracle_bridges` | L1 | Protocol bridges to industrial fieldbus/messaging systems | `kafka_bridge.py`, `opc_ua_bridge.py`, `modbus_bridge.py`, `sparkplug_bridge.py`, `mtconnect_agent.py` | `KafkaBridge`, `OpcUaBridge`, `ModbusBridge` |
| `miracle_twin` | L2 | Cutting simulation proxy, prediction pipeline, adaptive parameter control | `cutting_sim_proxy.py`, `prediction_runner.py`, `adaptive_controller.py`, `tool_library.py`, `block_telemetry.py` | `CuttingSimProxy`, `PredictionRunner`, `AdaptiveController` |
| `miracle_ai` | L2 | Anomaly detection (Z-score + Isolation Forest + PCA ensemble), PHM, model lifecycle | `anomaly_detector.py`, `phm_predictor.py`, `model_manager.py`, `tool_wear_estimator.py`, `chatter_detector.py` | `AnomalyDetector`, `PHMPredictor`, `ModelManager` |
| `miracle_scada` | L3 | Alarm management with escalation, alert correlation, KPI calculation, Prometheus export | `alarm_manager.py`, `alert_correlator.py`, `kpi_calculator.py`, `prometheus_exporter.py`, `historian.py` | `AlarmManager`, `AlertCorrelator`, `KpiCalculator` |
| `miracle_mes` | L3 | Digital thread with hash-chain integrity, job scheduling, fleet management, OEE | `digital_thread.py`, `job_scheduler.py`, `fleet_manager.py`, `oee_calculator.py`, `resource_manager.py` | `DigitalThread`, `JobScheduler`, `FleetManager` |
| `miracle_security` | L4 | Intrusion detection, RBAC access control, audit logging, G-code signing, TPM integration | `intrusion_detection.py`, `access_enforcer.py`, `audit_logger.py`, `gcode_signer.py`, `tpm_interface.py` | `IntrusionDetection`, `AccessEnforcer`, `GCodeSigner` |
| `miracle_resiliency` | L4 | Recovery orchestration, chaos testing, heartbeat aggregation, failover coordination | `recovery_orchestrator.py`, `chaos_injector.py`, `fault_executor.py`, `heartbeat_aggregator.py`, `lifecycle_client.py` | `RecoveryOrchestrator`, `ChaosInjector`, `HeartbeatAggregator` |
| `miracle_cognitive` | L5 | Knowledge graph, causal reasoning, GOAP/HTN planning, multi-agent task allocation, self-x autonomy | `knowledge/knowledge_graph.py`, `knowledge/causal_inference.py`, `knowledge/reasoning_engine.py`, `interface/explanation_generator.py`, `multi_agent/task_allocator.py` | `KnowledgeGraph`, `CausalInference`, `ReasoningEngine`, `TaskAllocator` |
| `miracle_unity_bridge` | -- | ROS-TCP-Endpoint bridge node; translates between TCP frames and ROS2 topics | `unity_endpoint.py` | `UnityEndpoint` |
| `miracle_bringup` | -- | Launch files for full-system, simulation, physical, and layer-specific orchestration | `miracle_full_system.launch.py`, `miracle_simulation.launch.py`, `miracle_physical.launch.py`, `lifecycle_autostart.py` | -- |
| `miracle_gazebo` | -- | Gazebo Fortress simulation world and launch configuration | `cnc_world.launch.py` | -- |
| `miracle_msgs` | -- | Custom ROS2 message and service definitions (38 msgs, 16 srvs) | `msg/*.msg`, `srv/*.srv` | -- |
| `miracle_microros` | -- | micro-ROS agent configuration for embedded CNC controllers | -- | -- |

---

## 3. Unity Project Structure

All scripts live under `unity_twin/Assets/Scripts/`.

| Directory | Responsibility | Key Files |
|-----------|---------------|-----------|
| `Core/` | ROS2 bridge, message routing, simulation clock, data recording, replay, performance monitoring, config | `MiracleBridge.cs`, `MessageDispatcher.cs`, `SimulationClock.cs`, `PerformanceMonitor.cs`, `DataRecorder.cs`, `ReplayController.cs`, `MiracleConfig.cs`, `SensorDataBridge.cs` |
| `Core/Events/` | ScriptableObject event channels for decoupled communication | `GameEventSO.cs`, `MachineStateEventSO.cs`, `AnomalyAlertEventSO.cs`, `CuttingStateEventSO.cs`, `ToolWearEventSO.cs`, `SecurityAlertEventSO.cs` |
| `Cutting/` | Voxel-based material removal, G-code parsing/execution, force models, thermal/wear/roughness simulation | `VoxelWorkpiece.cs`, `VoxelGridData.cs`, `MarchingCubesRenderer.cs`, `GCodeParser.cs`, `GCodeInterpreter.cs`, `CuttingForceEngine.cs`, `ThermalModel.cs`, `ToolWearModel.cs`, `SurfaceRoughnessModel.cs`, `GCodeValidator.cs`, `ChipFormationModel.cs` |
| `CNC/` | Machine-specific controllers, kinematics, spindle visualization, workpiece/vise management | `ICNCController.cs`, `BantamExplorerController.cs`, `CoastRunnerCR1Controller.cs`, `CNCMachineProfileSO.cs`, `CNCMachineSelector.cs`, `SpindleVisualizer.cs`, `WorkpieceManager.cs` |
| `Robots/` | Robot arm controllers for machine tending (Niryo Ned2, xArm 6), IK, trajectory, multi-agent coordination | `RobotController.cs`, `InverseKinematics.cs`, `TrajectoryInterpolator.cs`, `GripperController.cs`, `MultiAgentCoordinator.cs`, `RobotTendingSequence.cs` |
| `UI/` | Dashboard panels, charts, runtime file browser, G-code editor, simulation controls | `ForceChart.cs`, `FleetOverviewPanel.cs`, `AlertTimelinePanel.cs`, `DecisionSupportPanel.cs`, `ThermalChart.cs`, `ToolWearChart.cs`, `PowerChart.cs`, `RuntimeFileBrowser.cs`, `GCodeEditor.cs`, `SimulationControlPanel.cs`, `EStopButton.cs` |
| `Visualization/` | 3D overlays: heat maps, force arrows, surface roughness, chip particles, toolpath preview | `HeatMapOverlay.cs`, `ForceArrowRenderer.cs`, `SurfaceRoughnessOverlay.cs`, `ChipParticleController.cs`, `ToolpathPreview.cs`, `WearIndicator.cs` |
| `Audio/` | Procedural audio driven by cutting parameters and spindle RPM | `CuttingSoundController.cs`, `SpindleSoundController.cs` |
| `RosMessages/` | C# message/service definitions mirroring `miracle_msgs` | `Msg/*.cs`, `Srv/*.cs` |
| `Editor/` | Unity Editor utilities: scene builders, asset creators, dashboard wiring | `SceneBuilder.cs`, `SceneSetup.cs`, `DashboardWiring.cs`, `CNCProfileCreator.cs` |
| `Testing/` | Standalone test driver for offline CNC validation | `LocalCNCTestDriver.cs` |

---

## 4. Data Flow

### 4.1 G-code Execution

```
G-code file --> GCodeParser --> GCodeInterpreter --> Lookahead buffer
  --> CNC axis commands --> VoxelWorkpiece (boolean subtraction)
  --> MarchingCubesRenderer --> GPU mesh
```

The parser tokenizes G-code into blocks. The interpreter resolves coordinates, feeds, and tool changes. The lookahead buffer smooths velocity transitions across consecutive blocks. Voxel subtraction is performed on a 3D grid, and marching cubes generates the visible mesh each frame.

### 4.2 Cutting Forces

```
Block params (depth, width, feed, RPM, material)
  --> CuttingForceEngine (Altintas mechanistic model)
  --> Fx, Fy, Fz components
  --> ForceChart (UI) + ForceArrowRenderer (3D)
  --> AnomalyDetector (ROS2, via bridge)
```

Force coefficients come from `MaterialDatabase` and `ToolDefinition` ScriptableObjects. The force engine runs per-frame during active cutting and publishes via `CuttingStateEventSO`.

### 4.3 Prediction Pipeline

```
GCodeBlock stream --> CuttingSimProxy (ROS2)
  --> PredictionRunner (runs Altintas + thermal + wear models server-side)
  --> Predicted forces, temperatures, wear rates
  --> AnomalyMarkers (flagged blocks sent back to Unity)
  --> AdaptiveController (adjusts feed/speed overrides)
```

This pipeline runs ahead of real-time execution, scanning upcoming blocks for stability limit violations, thermal overload, or excessive tool wear before the machine reaches those blocks.

### 4.4 Alert Pipeline

```
Sensor data --> AnomalyDetector (Z-score + Isolation Forest + PCA ensemble)
  --> AnomalyAlert msg --> AlertCorrelator (pattern library matching)
  --> CorrelatedAlert msg --> AlarmManager (escalation engine)
  --> Escalation tiers --> NotificationDispatcher
  --> Unity AlertTimelinePanel + NotificationToastManager
```

The ensemble detector uses three independent algorithms; an anomaly is raised when at least two agree. The correlator groups temporally and spatially related alerts to reduce noise. The escalation engine applies configurable timeout-based tier promotion.

### 4.5 Digital Thread

```
Manufacturing events --> DigitalThread node
  --> SHA256 hash chain (each entry hashes previous entry)
  --> Energy tracking (per-block kWh accumulation)
  --> Prediction tracking (predicted vs. actual comparison)
  --> Genealogy tracking (material batch --> part record linkage)
  --> DigitalThreadEntry msg --> Audit log (immutable)
```

The hash chain ensures tamper-evident recording. Any break in the chain is detectable by recomputing hashes from the genesis entry.

---

## 5. Key Design Patterns

### ScriptableObject Events

Unity components communicate through `ScriptableObject`-based event channels (the `Core/Events/` directory). A publisher calls `Raise()` on an event asset; all listeners registered on that asset receive the callback. This decouples publishers from subscribers and avoids `FindObjectOfType` or singleton patterns.

```csharp
// Publisher
[SerializeField] private CuttingStateEventSO onCuttingUpdate;
onCuttingUpdate.Raise(newState);

// Subscriber
onCuttingUpdate.RegisterListener(HandleCuttingUpdate);
```

### ROS2 Lifecycle Nodes

All ROS2 nodes extend `MiracleLifecycleNode`, which wraps the standard ROS2 lifecycle state machine and mixes in `HeartbeatMixin`:

```python
class MyNode(MiracleLifecycleNode):
    def on_configure(self, state):
        # Load parameters, create publishers/subscribers
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        # Start timers, begin processing
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        # Pause processing
        return TransitionCallbackReturn.SUCCESS
```

Lifecycle transitions are managed by `miracle_bringup/lifecycle_autostart.py`, which brings nodes through `configure -> activate` in dependency order.

### sys.modules.setdefault for Test Isolation

ROS2 Python tests use `sys.modules.setdefault` to inject mock submodules before importing the module under test. This avoids importing `rclpy` or hardware-dependent code in CI:

```python
sys.modules.setdefault("rclpy", MagicMock())
sys.modules.setdefault("rclpy.lifecycle", MagicMock())
# Now import the module under test
from miracle_cnc.gcode_executor import GCodeExecutor
```

Never mock a top-level package that is already partially imported. Always mock leaf submodules first.

### Dataclass-Heavy Design

All structured data transfer in ROS2 Python code uses `@dataclass` classes. This provides type hints, default values, and `__eq__`/`__repr__` for free, and avoids raw dicts.

### Hash-Chained Audit

The `DigitalThread` node maintains a SHA256 chain where each `DigitalThreadEntry` includes the hash of the previous entry. This provides tamper-evident manufacturing records without requiring an external blockchain.

---

## 6. Configuration

| Source | Location | Purpose |
|--------|----------|---------|
| `miracle_defaults.yaml` | `miracle_ws/config/miracle_defaults.yaml` | Central ROS2 node parameters; supports `${ENV_VAR}` overrides |
| SROS2 Governance | `miracle_ws/config/sros2_governance.xml` | DDS security governance policies |
| SROS2 Permissions | `miracle_ws/config/sros2_permissions.xml` | Per-node DDS topic access permissions |
| ScriptableObjects | `unity_twin/Assets/` (various `.asset` files) | Unity-side machine profiles (`CNCMachineProfileSO`), tool definitions (`ToolDefinition`), material databases, event channels |
| `MiracleConfig.cs` | `unity_twin/Assets/Scripts/Core/MiracleConfig.cs` | Unity runtime configuration (bridge host/port, feature flags) |
| Launch files | `miracle_ws/src/miracle_bringup/launch/` | System composition; which nodes to start, parameter file paths, remappings |
| Docker `.env` | Project root (deployment) | Container-specific overrides (ports, volumes, log levels) |

Parameters in `miracle_defaults.yaml` follow a namespace convention: `/<package_name>/<node_name>/<parameter>`. Any parameter can be overridden at launch time via command-line or environment variables.

---

## 7. Extending the System

### Adding a New ROS2 Node

1. Identify the appropriate layer package (e.g., `miracle_twin` for L2 functionality).
2. Create a new Python module in that package directory.
3. Extend `MiracleLifecycleNode`:

```python
from miracle_core.lifecycle_node_base import MiracleLifecycleNode

class MyNewNode(MiracleLifecycleNode):
    def __init__(self):
        super().__init__("my_new_node")

    def on_configure(self, state):
        # Set up publishers, subscribers, parameters
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        return TransitionCallbackReturn.SUCCESS
```

4. Add a console_scripts entry point in the package's `setup.py`.
5. Add the node to the appropriate launch file in `miracle_bringup`.
6. Add default parameters to `miracle_defaults.yaml`.

### Adding a New Unity UI Panel

1. Create a new C# script in `unity_twin/Assets/Scripts/UI/`.
2. Create matching UXML (layout) and USS (style) files in `Assets/UI/`.
3. Subscribe to the relevant `ScriptableObject` event channel(s) from `Core/Events/`.
4. Wire the panel into the dashboard via `Editor/DashboardWiring.cs` or by adding it to the scene hierarchy.
5. If the panel needs ROS2 data, register a handler in `MessageDispatcher` for the appropriate message type.

### Adding a New Test File (ROS2 Python)

1. Create the test file under the package's `test/` directory.
2. Use `sys.modules.setdefault` to mock ROS2 and hardware dependencies **before** importing the module under test:

```python
import sys
from unittest.mock import MagicMock

# Mock submodules -- never mock a package that is already partially loaded
sys.modules.setdefault("rclpy", MagicMock())
sys.modules.setdefault("rclpy.lifecycle", MagicMock())
sys.modules.setdefault("rclpy.node", MagicMock())
sys.modules.setdefault("miracle_msgs.msg", MagicMock())

from miracle_twin.my_module import MyClass

def test_my_feature():
    obj = MyClass()
    assert obj.some_method() == expected_value
```

3. Run tests from the workspace root:

```bash
cd miracle_ws
python -m pytest --rootdir=. src/miracle_twin/test/test_my_module.py -v
```

### Adding a New Message Type

1. Create the `.msg` file in `miracle_ws/src/miracle_msgs/msg/`:

```
# MyNewMessage.msg
std_msgs/Header header
float64 value
string description
```

2. Register it in `miracle_ws/src/miracle_msgs/CMakeLists.txt` under `rosidl_generate_interfaces`.
3. Rebuild: `colcon build --packages-select miracle_msgs`.
4. Create the matching C# message class in `unity_twin/Assets/Scripts/RosMessages/Msg/` following the existing pattern (class name must match, namespace `RosMessageTypes.Miracle`).
5. Register the message type in `MessageDispatcher.cs` if it needs to be routed to Unity subscribers.

---

## Appendix: Message Catalog (Selected)

### Topics (miracle_msgs/msg)

| Message | Used By | Purpose |
|---------|---------|---------|
| `MachineState` | cnc, twin, UI | Axis positions, spindle RPM, coolant state |
| `SensorData` / `FusedSensorData` | cnc, ai | Raw and Kalman-filtered sensor readings |
| `GCodeBlock` | cnc, twin | Current G-code block under execution |
| `CuttingState` | twin, UI | Forces, temperatures, MRR, chip load |
| `AnomalyAlert` / `CorrelatedAlert` | ai, scada, UI | Single and correlated anomaly detections |
| `ToolWearEstimate` | ai, twin | Predicted remaining tool life |
| `PHMPrediction` | ai, mes | Prognostics and health prediction |
| `SystemKPIs` | scada, UI | OEE, availability, quality metrics |
| `DigitalThreadEntry` | mes | Hash-chained manufacturing event record |
| `SecurityAlert` | security, UI | IDS and access violation alerts |
| `Heartbeat` | core, resiliency | Node liveness signal |
| `TaskAnnouncement` / `TaskAward` | cognitive | Multi-agent task allocation protocol |

### Services (miracle_msgs/srv)

| Service | Purpose |
|---------|---------|
| `TriggerEStop` | Emergency stop command |
| `ValidateGCode` | Server-side G-code validation |
| `GetFleetStatus` | Query fleet-wide machine status |
| `RunPrediction` | On-demand prediction for a G-code block |
| `InjectFault` | Chaos testing fault injection |
| `SPARQLQuery` | Knowledge graph query |
| `OptimizeParameters` | Request parameter optimization from cognitive layer |
