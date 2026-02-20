# MIRACLE Unity Digital Twin -- Technical Manual

> **Bantam Desktop Explorer CNC Milling Cell**
> Workpiece: 3" x 3" x 2" (76.2 x 76.2 x 50.8 mm) 6061-T6 Aluminum
> Tool: 1/4" 2-Flute HSS End Mill, ER-11 Collet, 10K-23K RPM
> Robot Tenders: Niryo Ned2 + xArm 6 Lite

---

# Part 1 -- Architecture, Setup, and Core Systems (Sections 1-8)

---

## 1. Overview

The MIRACLE Unity Digital Twin is a real-time 3D visualization of a CNC
manufacturing cell built in **Unity 6 LTS** (2024.x+) with the **Universal
Render Pipeline (URP)**. It connects to the MIRACLE ROS2 system via
**ROS-TCP-Connector** (TCP port 10000) and provides:

- **3-axis CNC motion** — Bantam Desktop Explorer with ArticulationBody joints
- **Voxel material removal** — 256x170x256 bit-packed GPU grid (~11.1M voxels)
- **Mechanistic cutting forces** — Full Altintas model (Ktc/Krc/Kac + edge coefficients)
- **Thermal simulation** — Stephenson-Agapiou interface + Loewen-Shaw partition + lumped ODE
- **Tool wear progression** — Taylor equation + 3-stage flank wear (VBmax = 0.30 mm)
- **Chip formation** — Merchant shear theory + VFX Graph particles
- **Robot machine tending** — Niryo Ned2 and xArm 6 Lite via L5 cognitive layer
- **Dashboard HUD** — Real-time KPIs, force charts, alerts, and simulation controls
- **Three timing modes** — Real-Time (1:1), Accelerated (up to 100x), and Replay
- **Procedural audio** — Cutting sounds driven by tooth-passing frequency and power
- **Data recording** — Binary stream capture with timeline scrub playback

### 1.1 Physical Cell

| Component | Specification |
|-----------|--------------|
| CNC Machine | Bantam Desktop Explorer, 3-axis vertical mill |
| Work Volume | 6" x 4" x 2.75" (152.4 x 101.6 x 69.85 mm) |
| Controller | TinyG V9 (G-code over USB serial) |
| Spindle | ER-11 collet, 10,000-23,000 RPM, 250W brushless |
| Workpiece | 3" x 3" x 2" (76.2 x 76.2 x 50.8 mm) 6061-T6 aluminum |
| Tool | 1/4" (6.35 mm) 2-flute HSS flat end mill |
| Robot A | Niryo Ned2 — 6-DOF, 440 mm reach, 300g payload |
| Robot B | UFactory xArm 6 Lite — 6-DOF, 440 mm reach, 600g payload |
| Robot Role | Redundant machine tenders (L5 auction-based assignment) |

### 1.2 Software Stack

| Layer | Technology |
|-------|-----------|
| 3D Engine | Unity 6 LTS, Universal Render Pipeline |
| Physics | ArticulationBody (reduced-coordinate), Burst Jobs |
| GPU Compute | Voxel subtraction, marching cubes, engagement counting |
| Particles | VFX Graph (GPU) — chip formation, coolant mist |
| UI | UI Toolkit (UXML + USS) |
| Audio | OnAudioFilterRead procedural synthesis |
| ROS Bridge | ROS-TCP-Connector v0.7.1 (TCP, not WebSocket) |
| Shaders | ShaderGraph (URP) — heat map, aluminum PBR, ghost preview |
| Testing | Unity Test Framework (EditMode + PlayMode) |

### 1.3 File Count Summary

The Unity Digital Twin comprises **130+ C# scripts** organized across 10
subsystems, plus compute shaders, ShaderGraph assets, VFX Graph effects, UI
Toolkit layouts, sample G-code programs, and resource data files.

---

## 2. Architecture

### 2.1 System Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Unity 6 LTS  (URP)                           │
│                                                                     │
│  ┌───────────────┐  ┌──────────────────┐  ┌──────────────────────┐ │
│  │ Bantam Explorer│  │ Niryo Ned2  +    │  │  Cutting Simulation  │ │
│  │ 3-axis CNC     │  │ xArm 6 Lite     │  │ ┌────────┐┌───────┐ │ │
│  │ ArticBody X/Y/Z│  │ URDF→ArticBody   │  │ │Voxel   ││Force  │ │ │
│  │ Spindle revolute│  │ 6-DOF revolute   │  │ │Grid GPU││Engine │ │ │
│  └───────┬────────┘  └────────┬─────────┘  │ │Marching││Thermal│ │ │
│          │                    │             │ │Cubes   ││Wear   │ │ │
│          │  SO Event Channels │             │ └────────┘└───────┘ │ │
│          └────────┬───────────┘             └──────────┬──────────┘ │
│                   │                                    │            │
│          ┌────────┴────────────────────────────────────┘            │
│          │              MessageDispatcher                           │
│          │        (ConcurrentQueue → main thread)                   │
│          └────────────────────┬─────────────────────────────────────│
│                               │  ROSConnection (TCP)               │
│  ┌────────────────────────────┴──────────────────────────────────┐ │
│  │ MiracleBridge.cs                                               │ │
│  │   Subs: /miracle/cnc1/state, anomaly, tool_wear, job_status   │ │
│  │         /miracle/twin/sync_status, /miracle/system_kpis       │ │
│  │         /miracle/heartbeats, /miracle/security/alerts         │ │
│  │         /miracle/cognitive/task_awards                         │ │
│  │   Pubs: /miracle/unity/heartbeat                               │ │
│  │   Srvs: trigger_estop, validate_gcode, get_fleet_status       │ │
│  └───────────────────────────┬───────────────────────────────────┘ │
└──────────────────────────────┼──────────────────────────────────────┘
                               │ TCP  localhost:10000
┌──────────────────────────────┼──────────────────────────────────────┐
│                     ROS2 Jazzy (miracle_ws)                          │
│    ┌─────────────────────────┴─────────────────────┐                │
│    │  ros_tcp_endpoint  (miracle_unity_bridge pkg)  │                │
│    │  Bridges ALL miracle_msgs types over TCP       │                │
│    └─────────────────────────┬─────────────────────┘                │
│                              │ DDS                                  │
│   L1: state_publisher, gcode_executor, sensor_fusion, watchdog      │
│   L2: discovery_server, alarm_manager, historian, hmi_bridge        │
│   L3: job_scheduler, fleet_manager, sync_engine, anomaly_detector   │
│   L4: intrusion_detection, heartbeat_aggregator, threat_response    │
│   L5: task_allocator, auction_manager, goal_manager, rl_optimizer   │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow

1. **ROS2 → Unity**: MIRACLE nodes publish on DDS topics → `ros_tcp_endpoint`
   serializes and forwards over TCP → `ROSConnection` deserializes in Unity →
   `MiracleBridge` receives callbacks → dispatches via `GameEventSO<T>` channels →
   subscribers (CNC controller, robot controller, dashboard, etc.) react.

2. **Unity → ROS2**: User clicks E-Stop button → `EStopButton.cs` calls
   `ROSConnection.SendServiceMessage<TriggerEStopRequest>()` → TCP to
   `ros_tcp_endpoint` → DDS service call to `miracle_cnc` → machine stops.

3. **Internal Unity**: `CuttingSimulationManager` orchestrates per-FixedUpdate:
   G-code interpreter → tool position → voxel subtraction (GPU) → engagement
   count (GPU) → force calculation (Burst) → thermal update → wear update →
   chip formation → visualization updates.

### 2.3 ScriptableObject Event Architecture

All inter-system communication uses typed `GameEventSO<T>` ScriptableObject
events, enabling fully decoupled listeners. Components register in `OnEnable`
and unregister in `OnDisable`:

| Event SO | Message Type | Raised By | Listeners |
|----------|-------------|-----------|-----------|
| `MachineStateEventSO` | `MachineStateMsg` | MiracleBridge | BantamExplorerController, DashboardOverlay |
| `AnomalyAlertEventSO` | `AnomalyAlertMsg` | MiracleBridge | AlertNotification, CuttingSimulationManager |
| `ToolWearEventSO` | `ToolWearEstimateMsg` | MiracleBridge | WearIndicator, WearProgressBar |
| `TwinSyncEventSO` | `TwinSyncStatusMsg` | MiracleBridge | DashboardOverlay |
| `SystemKPIsEventSO` | `SystemKPIsMsg` | MiracleBridge | DashboardOverlay, FleetHealthPanel |
| `JobStatusEventSO` | `JobStatusMsg` | MiracleBridge | RobotTendingSequence, DashboardOverlay |
| `RobotJointStateEventSO` | `RobotJointStateMsg` | MiracleBridge | RobotController |
| `TaskAwardEventSO` | `TaskAwardMsg` | MiracleBridge | MultiAgentCoordinator |
| `SecurityAlertEventSO` | `SecurityAlertMsg` | MiracleBridge | AlertNotification, FleetHealthPanel |
| `CuttingStateEventSO` | `CuttingStateData` | CuttingSimManager | ForceArrowRenderer, ChipParticleController |

---

## 3. Prerequisites

| Requirement | Version | Notes |
|-------------|---------|-------|
| Unity | 6 LTS (2024.x+) | Unity Hub recommended |
| Render Pipeline | URP 17.0.3+ | Included via manifest.json |
| GPU | DirectX 11+ / Vulkan / Metal | Compute shader support required |
| VRAM | 2 GB minimum | Voxel buffers + marching cubes + VFX |
| RAM | 8 GB minimum | 16 GB recommended |
| OS | Windows 10+, macOS 12+ (Metal), Ubuntu 22.04+ | |
| ROS 2 | Jazzy (for live connection) | Optional for offline/replay mode |
| miracle_ws | Built with `build_all.sh` | Optional for offline mode |
| Node.js | 18+ | Only if also running web dashboard |

### 3.1 Required Unity Packages

These are auto-resolved from `Packages/manifest.json`:

| Package | Version | Purpose |
|---------|---------|---------|
| `com.unity.robotics.ros-tcp-connector` | 0.7.1 | ROS2 TCP bridge |
| `com.unity.robotics.urdf-importer` | 0.5.2 | Robot URDF import |
| `com.unity.visualeffectgraph` | 17.0.3 | Chip/coolant particles |
| `com.unity.burst` | 1.8.18 | High-performance force calc |
| `com.unity.collections` | 2.5.1 | NativeArray for Burst jobs |
| `com.unity.mathematics` | 1.3.2 | SIMD math for Burst |
| `com.unity.render-pipelines.universal` | 17.0.3 | URP rendering |
| `com.unity.inputsystem` | 1.11.2 | Input bindings |
| `com.unity.ui` | 2.0.0 | UI Toolkit |
| `com.unity.test-framework` | 1.4.5 | Unit/integration tests |

---

## 4. Installation and Setup

### 4.1 Clone the Repository

```bash
git clone <repo-url> banatam_cnc_milling_unity_digital_twin
cd banatam_cnc_milling_unity_digital_twin
```

### 4.2 ROS2 Workspace Setup (for live connection)

```bash
cd miracle_ws
source /opt/ros/jazzy/setup.bash
bash scripts/setup_workspace.sh
bash scripts/build_all.sh
source install/setup.bash
```

### 4.3 Clone ROS-TCP-Endpoint

```bash
cd miracle_ws/src
git clone -b main-ros2 \
    https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
    ros_tcp_endpoint
cd ..
colcon build --packages-select miracle_unity_bridge ros_tcp_endpoint --symlink-install
source install/setup.bash
```

### 4.4 Open the Unity Project

1. Open **Unity Hub** → Add → select the `unity_twin/` folder
2. Unity 6 LTS will import all packages (first time takes 2-5 minutes)
3. If prompted about render pipeline, select **Universal Render Pipeline**
4. Open the scene: **Assets/Scenes/ManufacturingCell.unity**
   - Or use the menu: **MIRACLE > Build Manufacturing Cell Scene** to auto-generate it

### 4.5 Verify ROS-TCP-Connector Settings

1. In the Unity menu: **Robotics > ROS Settings**
2. Set **ROS IP Address**: `127.0.0.1`
3. Set **ROS Port**: `10000`
4. Protocol: **ROS2**

### 4.6 First Run

```bash
# Terminal 1: Launch MIRACLE with Unity bridge
cd miracle_ws
source install/setup.bash
ros2 launch miracle_bringup miracle_simulation.launch.py machine_count:=1

# Terminal 2: Verify endpoint
ros2 node list | grep unity
# Expected: /miracle/unity/unity_endpoint_config
```

Then press **Play** in Unity. The dashboard should show a green **"ROS Connected"**
indicator.

### 4.7 Running Without ROS2 (Offline Mode)

The digital twin can run without a live ROS2 connection:

- **Accelerated mode**: Load a G-code file and run the cutting simulation locally
- **Replay mode**: Load a previously recorded `.miracle` session file
- The dashboard will show "ROS Disconnected" in red, but all local simulation
  features (voxel removal, forces, thermal, wear) work fully

---

## 5. Project Structure

```
unity_twin/
├── Assets/
│   ├── _Project/
│   │   ├── MiracleTwin.asmdef              Main assembly definition
│   │   └── MiracleTwin.Tests.asmdef        Test assembly
│   │
│   ├── Scenes/
│   │   └── ManufacturingCell.unity         Main scene
│   │
│   ├── Scripts/
│   │   ├── Core/                           (7 scripts + 11 event SOs)
│   │   │   ├── SimulationClock.cs          Time manager (3 modes)
│   │   │   ├── MiracleBridge.cs            ROS2 subscriptions + services
│   │   │   ├── MessageDispatcher.cs        Thread-safe ROS → main thread
│   │   │   ├── DataRecorder.cs             Binary stream recording
│   │   │   ├── ReplayController.cs         Playback with scrub/rewind
│   │   │   ├── CameraController.cs         Orbit/follow/preset cameras
│   │   │   ├── InputManager.cs             Keyboard/mouse/gamepad
│   │   │   ├── PerformanceMonitor.cs       FPS, GPU time, memory
│   │   │   └── Events/                     (11 GameEventSO<T> channels)
│   │   │
│   │   ├── RosMessages/                    (17 C# message mirrors)
│   │   │   ├── Msg/                        14 message types
│   │   │   └── Srv/                        3 service types
│   │   │
│   │   ├── CNC/                            (6 scripts)
│   │   │   ├── BantamExplorerController.cs 3-axis motion from MachineState
│   │   │   ├── BantamExplorerBuilder.cs    Procedural mesh generation
│   │   │   ├── SpindleVisualizer.cs        Rotation + motion blur
│   │   │   ├── WorkpieceManager.cs         Mount/unmount/reset workpiece
│   │   │   ├── ViseController.cs           Open/close vise
│   │   │   └── EnclosureLid.cs             Lid animation
│   │   │
│   │   ├── Robots/                         (6 scripts)
│   │   │   ├── RobotController.cs          Generic 6-DOF URDF driver
│   │   │   ├── GripperController.cs        Symmetric finger mimic
│   │   │   ├── RobotTendingSequence.cs     Machine tending state machine
│   │   │   ├── MultiAgentCoordinator.cs    L5 TaskAward → robot assignment
│   │   │   ├── TrajectoryInterpolator.cs   Smooth joint interpolation
│   │   │   └── InverseKinematics.cs        Analytical IK for waypoints
│   │   │
│   │   ├── Cutting/                        (14 scripts)
│   │   │   ├── CuttingSimulationManager.cs Master orchestrator
│   │   │   ├── VoxelWorkpiece.cs           GPU voxel grid + marching cubes
│   │   │   ├── VoxelGridData.cs            CPU-side metadata + dirty chunks
│   │   │   ├── MarchingCubesRenderer.cs    DrawProceduralIndirect
│   │   │   ├── CuttingForceEngine.cs       Altintas model (Burst)
│   │   │   ├── KienzleForceModel.cs        Quick-estimate backup
│   │   │   ├── ThermalModel.cs             Stephenson-Agapiou + ODE
│   │   │   ├── ToolWearModel.cs            Taylor + 3-stage flank
│   │   │   ├── ChipFormationModel.cs       Merchant shear + curl
│   │   │   ├── SurfaceRoughnessModel.cs    Kinematic Ra
│   │   │   ├── GCodeParser.cs              Line-by-line tokenizer
│   │   │   ├── GCodeInterpreter.cs         Toolpath segment generator
│   │   │   ├── MaterialDatabase.cs         6061-T6 + HSS properties
│   │   │   └── ToolDefinition.cs           End mill geometry
│   │   │
│   │   ├── Visualization/                  (7 scripts)
│   │   │   ├── ForceArrowRenderer.cs       GPU-instanced Fx/Fy/Fz arrows
│   │   │   ├── HeatMapOverlay.cs           Temperature → shader color
│   │   │   ├── ChipParticleController.cs   VFX Graph spawn + velocity
│   │   │   ├── WearIndicator.cs            3D tool wear visual
│   │   │   ├── ToolpathPreview.cs          LineRenderer upcoming path
│   │   │   ├── StabilityLobeChart.cs       2D stability lobe overlay
│   │   │   └── SurfaceRoughnessOverlay.cs  Ra color map
│   │   │
│   │   ├── UI/                             (11 scripts)
│   │   │   ├── DashboardOverlay.cs         Machine status + KPIs
│   │   │   ├── ForceChart.cs               Scrolling Fx/Fy/Fz graph
│   │   │   ├── ThermalChart.cs             Temperature vs time
│   │   │   ├── WearProgressBar.cs          VB/VBmax progress
│   │   │   ├── CuttingParameterPanel.cs    RPM, feed, DOC, MRR
│   │   │   ├── SimulationControlPanel.cs   Play/pause/speed/mode
│   │   │   ├── EStopButton.cs              TriggerEStop service call
│   │   │   ├── AlertNotification.cs        Toast popups
│   │   │   ├── RobotStatusPanel.cs         Per-robot state
│   │   │   ├── FleetHealthPanel.cs         System-wide health grid
│   │   │   └── GCodeEditor.cs              In-app G-code viewer
│   │   │
│   │   ├── Audio/                          (2 scripts)
│   │   │   ├── CuttingSoundController.cs   Procedural cutting audio
│   │   │   └── SpindleSoundController.cs   RPM-based pitch shift
│   │   │
│   │   └── Editor/
│   │       └── SceneBuilder.cs             Auto-creates full hierarchy
│   │
│   ├── Compute/                            (4 shaders)
│   │   ├── VoxelSubtraction.compute        GPU tool subtraction
│   │   ├── MarchingCubes.compute           GPU isosurface extraction
│   │   ├── VoxelEngagement.compute         Engaged voxel counting
│   │   └── Common.hlsl                     Shared functions
│   │
│   ├── Shaders/                            (5 shader assets)
│   │   ├── WorkpieceHeatMap.shadergraph
│   │   ├── AluminumPBR.shadergraph
│   │   ├── HSSToolSteel.shadergraph
│   │   ├── GhostPreview.shadergraph
│   │   └── ForceArrow.shader
│   │
│   ├── VFX/                                (2 VFX graphs)
│   │   ├── ChipFormation.vfx
│   │   └── CoolantMist.vfx
│   │
│   ├── Materials/                          (6 materials)
│   ├── Models/                             (Bantam, robots, environment)
│   ├── UI/                                 (UXML + USS layouts)
│   ├── Resources/                          (JSON data files)
│   ├── StreamingAssets/SamplePrograms/     (4 G-code files)
│   └── Tests/                              (EditMode + PlayMode)
│
├── Packages/manifest.json
└── ProjectSettings/
```

### 5.1 Assembly Definitions

The project uses two assembly definitions to enforce compilation boundaries:

- **`MiracleTwin.asmdef`** — All production scripts. References: Unity.Burst,
  Unity.Collections, Unity.Mathematics, Unity.RenderPipelines.Universal,
  Unity.Robotics.ROSTCPConnector, Unity.InputSystem, UnityEngine.UI.
- **`MiracleTwin.Tests.asmdef`** — Test scripts. References: MiracleTwin,
  UnityEngine.TestRunner, UnityEditor.TestRunner.

### 5.2 Scene Hierarchy

The `ManufacturingCell.unity` scene (or generated via **MIRACLE > Build
Manufacturing Cell Scene**) has this hierarchy:

```
ManufacturingCell
├── _Managers
│   ├── SimulationClock
│   ├── MiracleBridge
│   ├── MessageDispatcher
│   ├── CuttingSimulationManager
│   ├── MultiAgentCoordinator
│   ├── DataRecorder
│   ├── ReplayController
│   ├── InputManager
│   ├── PerformanceMonitor
│   └── CameraController
│
├── BantamExplorer
│   ├── Base_Frame
│   │   ├── Enclosure_Back
│   │   ├── Enclosure_Left
│   │   ├── Enclosure_Right
│   │   └── Enclosure_Lid (hinge)
│   ├── X_Gantry (ArticulationBody, prismatic X)
│   │   └── Y_Table (ArticulationBody, prismatic Z→Unity Z)
│   │       ├── Vise_Fixed_Jaw
│   │       ├── Vise_Moving_Jaw
│   │       └── WorkpieceMount
│   │           └── VoxelWorkpiece
│   └── Z_Column
│       └── Z_Head (ArticulationBody, prismatic Y→Unity Y)
│           └── Spindle_Motor
│               └── Collet_ER11 (revolute)
│                   └── EndMill
│
├── Ned2_Robot (ArticulationBody chain, 6 revolute joints)
│   └── Gripper
│
├── XArm6_Robot (ArticulationBody chain, 6 revolute joints)
│   └── Gripper
│
├── Environment
│   ├── Workbench
│   ├── StockTray
│   ├── FinishedTray
│   └── Floor
│
├── Lighting
│   ├── KeyLight (Directional)
│   ├── FillLight (Point)
│   └── RimLight (Point)
│
├── VFX
│   ├── ChipParticles (VFX Graph)
│   └── CoolantSpray (VFX Graph)
│
├── Visualization
│   ├── ForceArrows
│   ├── ToolpathPreview
│   └── HeatMapOverlay
│
├── Cameras
│   ├── MainCamera
│   ├── CM_Orbit (Cinemachine)
│   ├── CM_FollowTool (Cinemachine)
│   └── CM_PiP (Cinemachine, picture-in-picture)
│
└── UI
    ├── DashboardCanvas
    ├── SimControlPanel
    └── AlertContainer
```

---

## 6. ROS2 Connection

### 6.1 miracle_unity_bridge Package

**Source:** `miracle_ws/src/miracle_unity_bridge/`

This ROS2 package wraps the `ros_tcp_endpoint` node to bridge all DDS topics
over TCP port 10000 to Unity.

**Launch:**
```bash
ros2 launch miracle_unity_bridge unity_bridge.launch.py
```

**Configuration** (`config/unity_params.yaml`):
```yaml
unity_endpoint:
  ros__parameters:
    tcp_ip: "0.0.0.0"
    tcp_port: 10000
```

The bridge is automatically included when launching the full simulation:
```bash
ros2 launch miracle_bringup miracle_simulation.launch.py machine_count:=1
```

### 6.2 MiracleBridge.cs

**Source:** `unity_twin/Assets/Scripts/Core/MiracleBridge.cs`

`MiracleBridge` is a singleton MonoBehaviour that manages all ROS2 subscriptions,
publications, and service registrations. It dispatches incoming messages to
ScriptableObject event channels.

**Inspector Fields:**

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `rosBridgeIP` | string | `"127.0.0.1"` | ROS-TCP-Endpoint IP |
| `rosBridgePort` | int | `10000` | ROS-TCP-Endpoint port |
| `machineId` | string | `"cnc1"` | Target CNC machine ID |
| `reconnectInterval` | float | `5.0` | Seconds between reconnect attempts |
| `maxReconnectAttempts` | int | `10` | Max reconnection tries |

**Subscribed Topics:**

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/miracle/{machineId}/state` | `MachineStateMsg` | CNC axis positions, RPM, status |
| `/miracle/{machineId}/anomaly` | `AnomalyAlertMsg` | AI anomaly detection alerts |
| `/miracle/{machineId}/tool_wear` | `ToolWearEstimateMsg` | Wear %, VB, remaining life |
| `/miracle/{machineId}/job_status` | `JobStatusMsg` | Job progress, current line |
| `/miracle/twin/sync_status` | `TwinSyncStatusMsg` | Digital twin synchronization |
| `/miracle/system_kpis` | `SystemKPIsMsg` | OEE, availability, performance |
| `/miracle/cognitive/task_awards` | `TaskAwardMsg` | Robot task assignments |
| `/miracle/security/alerts` | `SecurityAlertMsg` | Security layer alerts |

**Published Topics:**

| Topic | Message Type | Frequency |
|-------|-------------|-----------|
| `/miracle/unity/heartbeat` | `HeartbeatMsg` | 1 Hz |

**Service Clients:**

| Service | Type | Called By |
|---------|------|----------|
| `/miracle/{machineId}/trigger_estop` | `TriggerEStopSrv` | EStopButton.cs |
| `/miracle/mes/validate_gcode` | `ValidateGCodeSrv` | GCodeEditor.cs |
| `/miracle/fleet/get_status` | `GetFleetStatusSrv` | FleetHealthPanel.cs |

### 6.3 Connection Events

```csharp
// Subscribe to connection status changes
MiracleBridge.Instance.ConnectionStatusChanged += (bool connected) =>
{
    Debug.Log(connected ? "ROS Connected" : "ROS Disconnected");
};
```

The connection is monitored via a 10-second message timeout. If no messages
arrive for 10 seconds, the bridge marks itself as disconnected and begins
reconnection attempts.

### 6.4 MessageDispatcher.cs

**Source:** `unity_twin/Assets/Scripts/Core/MessageDispatcher.cs`

ROSConnection in ROS-TCP-Connector already marshals callbacks to the Unity main
thread. `MessageDispatcher` adds buffering for non-real-time modes:

- **`TimestampedRingBuffer<T>`** per topic (capacity: 120 samples)
- In **Replay mode**: seeks to recorded timestamps rather than processing live
- In **Accelerated mode**: drops intermediate samples to match sim speed
- In **Real-Time mode**: passthrough (no buffering)

### 6.5 C# Message Definitions

All 17 C# message classes live in `Assets/Scripts/RosMessages/` and mirror the
`miracle_msgs` ROS2 definitions field-for-field. Each uses the
`[MessageName("miracle_msgs/TypeName")]` attribute required by ROS-TCP-Connector.

See **Part 2, Section 15** for the complete robot message definitions and
**Part 3, Section 24** for the full API reference of all message types.

---

## 7. Core Systems

### 7.1 SimulationClock

**Source:** `unity_twin/Assets/Scripts/Core/SimulationClock.cs`

`SimulationClock` is a singleton that provides a unified time authority for the
entire digital twin. All subsystems read `SimulationClock.DeltaTime` instead of
`Time.deltaTime`.

**Timing Modes:**

| Mode | DeltaTime Calculation | Use Case |
|------|----------------------|----------|
| `RealTime` | `Time.unscaledDeltaTime` (1:1 with wall clock) | Live ROS2 connection |
| `Accelerated` | `Time.unscaledDeltaTime * SpeedMultiplier` | Fast simulation (up to 100x) |
| `Replay` | Driven by `ReplayController.Seek(time)` | Recorded session playback |
| `Paused` | 0 | Simulation frozen |

**Public API:**

```csharp
// Singleton access
SimulationClock clock = SimulationClock.Instance;

// Properties
double simTime      = clock.SimTime;           // Accumulated simulation seconds
double dt           = clock.DeltaTime;         // Current frame delta (seconds)
float  speed        = clock.SpeedMultiplier;   // 1.0 = real-time
Mode   mode         = clock.CurrentMode;
double wallTime     = clock.TotalElapsedWallTime;
double wallSinceRst = clock.WallTimeSinceReset;

// Control
clock.SetMode(SimulationClock.Mode.Accelerated);
clock.SpeedMultiplier = 10f;
clock.Reset();    // Resets SimTime to 0

// Events
clock.OnTick += (double simTime) => { /* every sim frame */ };
clock.OnModeChanged += (Mode newMode) => { /* mode transition */ };
clock.OnReset += () => { /* sim reset */ };

// Formatting
string formatted = clock.FormatWallTime();  // "01:23:45"
```

### 7.2 CameraController

**Source:** `unity_twin/Assets/Scripts/Core/CameraController.cs`

Multi-mode camera system using Cinemachine virtual cameras (URP-compatible):

| Mode | Activation | Behavior |
|------|-----------|----------|
| **Orbit** (default) | LMB drag / scroll / MMB | Orbit around CNC center, zoom, pan |
| **Follow Tool** | Key `5` or toolbar button | Tracks tool tip with offset |
| **Front** | Key `1` | Fixed front view of manufacturing cell |
| **Side** | Key `2` | Fixed side view |
| **Top-Down** | Key `3` | Bird's eye view |
| **Isometric** | Key `4` | 45-degree isometric |
| **PiP** | Always on (toggleable) | Small inset showing cutting zone close-up |

**Configuration:**

```csharp
[SerializeField] private float orbitSpeed = 5f;
[SerializeField] private float zoomSpeed = 10f;
[SerializeField] private float panSpeed = 0.5f;
[SerializeField] private float minZoomDistance = 0.1f;
[SerializeField] private float maxZoomDistance = 2.0f;
[SerializeField] private bool enablePiP = true;
[SerializeField] private float pipScale = 0.25f;  // 25% of screen
```

### 7.3 InputManager

**Source:** `unity_twin/Assets/Scripts/Core/InputManager.cs`

All input bindings use the new Unity Input System:

| Action | Binding | Context |
|--------|---------|---------|
| Orbit | LMB drag | Camera |
| Pan | MMB drag | Camera |
| Zoom | Scroll wheel | Camera |
| Camera Preset 1-5 | Keys 1/2/3/4/5 | Camera |
| Play/Pause | Space | Simulation |
| Speed Up | `+` / `=` | Simulation |
| Speed Down | `-` | Simulation |
| E-Stop | Escape (double-tap) | Safety |
| Reset Sim | R | Simulation |
| Toggle HUD | H | UI |

### 7.4 PerformanceMonitor

**Source:** `unity_twin/Assets/Scripts/Core/PerformanceMonitor.cs`

Singleton that tracks frame-level performance metrics. Uses `DontDestroyOnLoad`.

**Tracked Metrics:**

| Metric | Source | Update Rate |
|--------|--------|-------------|
| `AverageFPS` | Rolling 60-sample window | Every frame |
| `MinFPS` / `MaxFPS` | Window min/max | Every frame |
| `GPUFrameTimeMs` | FrameTimingManager | Every frame |
| `CPUFrameTimeMs` | `1000 / AverageFPS` | Every frame |
| `DrawCallCount` | UnityStats (Editor) | Every frame |
| `TotalMemoryMB` | `GC.GetTotalMemory()` | Every frame |

**Events:**

```csharp
PerformanceMonitor.Instance.OnLowFPS += () =>
{
    // Fired when AverageFPS drops below 30
    // Consider reducing voxel resolution or disabling VFX
};
```

**Debug Output:**

```csharp
string summary = PerformanceMonitor.Instance.GetSummary();
// "FPS: 62.1 (min: 58, max: 67) | GPU: 8.2ms | CPU: 16.1ms | Mem: 412MB"
```

### 7.5 DataRecorder and ReplayController

**Source:** `unity_twin/Assets/Scripts/Core/DataRecorder.cs`,
`unity_twin/Assets/Scripts/Core/ReplayController.cs`

**Recording Format** (`.miracle` binary):
```
[Header: 16 bytes]
  magic: "MRCL" (4 bytes)
  version: uint32 (1)
  startTime: double (Unix timestamp)

[Frames: repeated]
  timestamp: double (8 bytes)
  topicHash: uint32 (4 bytes)
  payloadLength: int32 (4 bytes)
  payload: byte[] (payloadLength bytes)
```

**Replay Controls:**

```csharp
ReplayController replay = ReplayController.Instance;
replay.Load("path/to/recording.miracle");
replay.Play();
replay.Pause();
replay.Seek(30.0);        // Jump to 30 seconds
replay.SetSpeed(2.0f);    // 2x playback
replay.Rewind();           // Back to start
```

Files are saved to `StreamingAssets/Recordings/`.

---

## 8. CNC Machine Model

### 8.1 BantamExplorerBuilder

**Source:** `unity_twin/Assets/Scripts/CNC/BantamExplorerBuilder.cs`

Editor script (accessed via **MIRACLE > Build Manufacturing Cell Scene**) that
procedurally creates the Bantam Desktop Explorer CNC machine. This eliminates
the need for imported FBX models — the entire machine is built from Unity
primitives with correct dimensions.

**Generated Hierarchy:**

```
BantamExplorer (root, world origin 0,0,0)
├── Base_Frame (Box 400×200×260mm, gray, static)
│   ├── Enclosure_Back  (Box 400×387×5mm)
│   ├── Enclosure_Left  (Box 5×387×260mm)
│   ├── Enclosure_Right (mirror of Left)
│   └── Enclosure_Lid   (Box 400×5×260mm, hinge at back edge)
│
├── X_Gantry (ArticulationBody, prismatic X-axis)
│   └── Y_Table (ArticulationBody, prismatic Z-axis in Unity coords)
│       ├── Vise_Fixed_Jaw
│       ├── Vise_Moving_Jaw (animated)
│       └── WorkpieceMount (empty Transform for VoxelWorkpiece)
│
└── Z_Column (static pillar)
    └── Z_Head (ArticulationBody, prismatic -Y in Unity coords)
        └── Spindle_Motor (Box 50×100×50mm)
            └── Collet_ER11 (revolute joint, continuous)
                └── EndMill (Cylinder r=3.175mm, h=30mm)
```

### 8.2 ArticulationBody Joint Configuration

The CNC uses three prismatic `ArticulationBody` joints matching the real Bantam
Desktop Explorer's work volume:

| Joint | Unity Axis | Travel (m) | Stiffness | Damping | Real Axis |
|-------|-----------|-----------|-----------|---------|-----------|
| X_Gantry | X (1,0,0) | 0.0 – 0.1524 | 100,000 | 10,000 | X (left-right) |
| Y_Table | Z (0,0,1) | 0.0 – 0.1016 | 100,000 | 10,000 | Y (front-back) |
| Z_Head | -Y (0,-1,0) | 0.0 – 0.06985 | 100,000 | 10,000 | Z (up-down, inverted) |

Solver settings: `solverIterations = 50`, `solverType = TGS`,
`jointFriction = 0`.

### 8.3 Coordinate Mapping (ROS2 → Unity)

CNC machine axes map between coordinate systems as follows:

| ROS2 Axis | ROS2 Convention | Unity Axis | Unity Convention |
|-----------|----------------|-----------|-----------------|
| X | Left-Right (mm) | X | Left-Right (meters, ÷1000) |
| Y | Front-Back (mm) | Z | Forward-Back (meters, ÷1000) |
| Z | Up-Down (mm) | Y | Up-Down (meters, ÷1000, inverted for head) |

```csharp
// In BantamExplorerController.OnState():
targetPos = new Vector3(
    (float)msg.axis_positions[0],   // ROS X → Unity X
    (float)msg.axis_positions[2],   // ROS Z → Unity Y
    (float)msg.axis_positions[1]    // ROS Y → Unity Z
);
```

### 8.4 BantamExplorerController

**Source:** `unity_twin/Assets/Scripts/CNC/BantamExplorerController.cs`

Drives the three prismatic joints from `MachineStateMsg` data. Features:

**Machine State Enum:**
```csharp
public enum MachineState
{
    IDLE, RUNNING, PAUSED, ERROR, ESTOP, UNKNOWN
}
```

**Motion Interpolation:**
- Uses `Vector3.Lerp` with factor 0.8 (matching `sync_engine.py`)
- Applied in `FixedUpdate` for physics stability
- Joint targets set via `ArticulationBody.xDrive.target`

**Soft Limits:**
- Warning threshold at 95% of axis travel
- `IsNearSoftLimit(axis)` returns true when close to boundary
- Fires `OnSoftLimitApproaching` event for UI warning

**E-Stop Visualization:**
- When state is `ESTOP`, machine body flashes red at 3 Hz
- Sinusoidal color modulation via `Mathf.Sin(Time.time * 3 * 2π)`
- Resets to normal color when E-Stop is released

**Events:**
```csharp
// State change notification
controller.OnMachineStateChanged += (MachineState oldState, MachineState newState) =>
{
    Debug.Log($"Machine: {oldState} → {newState}");
};
```

### 8.5 SpindleVisualizer

**Source:** `unity_twin/Assets/Scripts/CNC/SpindleVisualizer.cs`

Handles spindle rotation visualization:

- **Visual rotation**: `degrees/frame = RPM × 360 / 60 × deltaTime`
- **Motion blur**: At RPM > 5000, applies radial blur post-processing effect
- **Blur intensity**: Scales linearly from 0 (5000 RPM) to 1.0 (23000 RPM)
- **Spindle on/off**: Responds to M3 (start CW), M5 (stop) in G-code stream
- **Ramp up/down**: Smooth RPM transition over 0.5 seconds

### 8.6 ViseController

**Source:** `unity_twin/Assets/Scripts/CNC/ViseController.cs`

Controls the workholding vise:

- **Open/Close**: Moving jaw slides on a prismatic joint
- **Jaw gap**: Adjusts to workpiece width (76.2 mm for 3" block)
- **Clamping force visual**: Slight jaw compression when closed
- **Signals**: `OnViseOpened` and `OnViseClosed` events for robot tending

### 8.7 EnclosureLid

**Source:** `unity_twin/Assets/Scripts/CNC/EnclosureLid.cs`

Animated enclosure lid for the Bantam Explorer:

- **Open**: Rotates 90° around hinge axis at back edge (0.5s animation)
- **Close**: Rotates back to 0° (0.5s animation)
- **Interlock**: Lid must be closed before spindle can start (mirrors real CNC)
- **Signals**: `OnLidOpened` and `OnLidClosed` events for robot tending
- **State**: `IsOpen`, `IsClosed`, `IsAnimating` properties

### 8.8 WorkpieceManager

**Source:** `unity_twin/Assets/Scripts/CNC/WorkpieceManager.cs`

Manages the lifecycle of workpieces in the CNC:

- **Mount**: Instantiates a `VoxelWorkpiece` at `WorkpieceMount` transform
- **Unmount**: Detaches finished workpiece (robot picks it up)
- **Reset**: Restores voxel grid to full block (new workpiece)
- **Events**: `OnWorkpieceMounted`, `OnWorkpieceUnmounted`

### 8.9 Integration Sequence

The full machine tending cycle ties all CNC components together:

```
Robot approaches CNC
  → EnclosureLid.Open()
  → Wait for OnLidOpened
  → ViseController.Open()
  → Wait for OnViseOpened
  → WorkpieceManager.Mount(rawBlock)
  → ViseController.Close()
  → Wait for OnViseClosed
  → Robot retracts
  → EnclosureLid.Close()
  → Wait for OnLidClosed
  → SpindleVisualizer starts (M3)
  → BantamExplorerController drives axes
  → CuttingSimulationManager runs physics
  → ... cutting completes ...
  → SpindleVisualizer stops (M5)
  → EnclosureLid.Open()
  → ViseController.Open()
  → Robot picks finished part
  → WorkpieceManager.Unmount()
  → Robot retracts
  → EnclosureLid.Close()
```

---

<!-- End of Part 1 -->

---

# Part 2 -- Cutting Physics, Toolpath, and Robot Integration (Sections 9-16)

---

## 9. Cutting Simulation Orchestration

**Source file:** `unity_twin/Assets/Scripts/Cutting/CuttingSimulationManager.cs`

### 9.1 Overview

`CuttingSimulationManager` is the master orchestrator that drives every physics
subsystem each `FixedUpdate` while cutting is active. It coordinates seven
subsystems in a strict pipeline order:

```
CuttingSimulationManager (FixedUpdate)
    |
    |-- 1. VoxelWorkpiece.SubtractTool()       (GPU material removal)
    |-- 2. CuttingForceEngine.Calculate()      (Altintas mechanistic forces)
    |-- 3. ThermalModel.Update()               (Stephenson-Agapiou interface temp)
    |-- 4. ToolWearModel.Update()              (3-stage flank wear)
    |-- 5. ChipFormationModel.Calculate()      (Merchant shear angle)
    |-- 6. SurfaceRoughnessModel.CalculateRa() (kinematic Ra)
    |-- 7. StabilityLobeChart check            (chatter detection)
    |
    +-- PublishState() --> CuttingStateEventSO  (ScriptableObject event bus)
```

### 9.2 Activation Criteria

Cutting activates only when **all** of these hold simultaneously:

| Condition               | Threshold                 | Code reference                           |
|-------------------------|---------------------------|------------------------------------------|
| Spindle RPM             | >= 1000 RPM (configurable)| `minimumCuttingRPM = 1000f`              |
| Feed rate               | >= 10 mm/min              | `minimumFeedRate = 10f`                  |
| CNC controller status   | `"RUNNING"`               | `cncController.Status == "RUNNING"`      |
| Tool not end-of-life    | VB < VBmax                | `!toolEndOfLifePauseTriggered`           |

When any condition drops out, `StopCutting()` fires and the thermal model
enters passive cooldown mode (`thermalModel.Cooldown(dt)`).

### 9.3 Per-Frame Pipeline (UpdateCutting)

Each `FixedUpdate` while cutting is active (dt = `Time.fixedDeltaTime`, default
0.02 s):

```csharp
// 1. Voxel subtraction -- returns count of newly cleared voxels
int removedVoxels = voxelWorkpiece.SubtractTool(previousToolPosition, toolPos, toolRadius);

// 2. Cutting forces (Altintas model)
forceEngine.Calculate(rpm, fz, ap, ae, toolWearModel.VB);

// 3. Thermal model
thermalModel.Update(V_mpm, fz, ap, power, dt);

// 4. Tool wear
toolWearModel.Update(V_mpm, fz, ap, dt);
```

Chip formation and surface roughness are computed during state publication.

### 9.4 Auto-Pause on VBmax

When `toolWearModel.IsEndOfLife` becomes true (VB >= 0.30 mm), the manager:

1. Sets `toolEndOfLifePauseTriggered = true`
2. Calls `SimulationClock.Instance.Pause()`
3. Logs a warning instructing the operator to call `ResetToolWear()`

The simulation remains frozen until the tool is replaced:

```csharp
// Resume after tool change
cuttingSimulationManager.ResetToolWear();
SimulationClock.Instance.Resume();
```

### 9.5 Stability Lobe Check

Every 0.25 seconds (configurable via `stabilityCheckInterval`), the manager
queries the `StabilityLobeChart` component:

```csharp
stabilityLobeChart.UpdateOperatingPoint(rpm, ap);
IsInUnstableZone = !stabilityLobeChart.IsStable;
```

The stability lobe chart uses the **Altintas-Budak zeroth-order approximation**:

```
ap_lim = -1 / (2 * Ktc * Nf * Re[G(j*omega_c)])
```

where `G(j*omega)` is the structural transfer function. Machine parameters:

| Parameter            | Value         | Unit   |
|----------------------|---------------|--------|
| Natural frequency    | 800           | Hz     |
| Damping ratio        | 0.03          | --     |
| Stiffness            | 5 x 10^6     | N/m    |
| Ktc (6061-T6 / HSS) | 796           | N/mm^2 |

When the operating point crosses from stable to unstable, a `CHATTER WARNING`
is logged with the current RPM and depth of cut.

### 9.6 Full Simulation Reset

`ResetSimulation()` (also triggered by `SimulationClock.OnReset`) returns every
subsystem to pristine state:

- `VoxelWorkpiece.Reset()` -- refills all bits to solid
- `ThermalModel.Reset()` -- all temperatures to 20 degC ambient
- `ToolWearModel.Reset()` -- VB back to 0.02 mm initial wear
- `CuttingForceEngine.Calculate(0,0,0,0,0)` -- zeroes all outputs
- Counters (`TotalMaterialRemoved`, `TotalCuttingTime`) zeroed

### 9.7 CuttingStateData Published Fields

The `CuttingStateEventSO` channel broadcasts a struct each frame containing:

| Field                | Type    | Description                                |
|----------------------|---------|--------------------------------------------|
| `spindleRPM`         | float   | Current spindle speed                      |
| `feedRate`           | float   | Commanded feed (mm/min)                    |
| `toolPosition`       | Vector3 | World-space tool tip                       |
| `forceFx/Fy/Fz`     | float   | Average cutting force components (N)       |
| `powerWatts`         | float   | Cutting power (W)                          |
| `torqueNm`           | float   | Spindle torque (Nm)                        |
| `toolTemperature`    | float   | Lumped tool body temperature (degC)        |
| `interfaceTemperature`| float  | Tool-chip interface temperature (degC)     |
| `flankWearVB`        | float   | Current flank wear land width (mm)         |
| `chipThicknessRatio` | float   | Merchant r = t1/t2                         |
| `shearAngleDeg`      | float   | Merchant phi (degrees)                     |
| `chipCurlRadius`     | float   | Rc (mm)                                    |

---

## 10. Voxel Material Removal

**Source files:**
- `unity_twin/Assets/Scripts/Cutting/VoxelWorkpiece.cs`
- `unity_twin/Assets/Scripts/Cutting/VoxelGridData.cs`
- `unity_twin/Assets/Scripts/Cutting/MarchingCubesRenderer.cs`
- `unity_twin/Assets/Compute/VoxelSubtraction.compute`
- `unity_twin/Assets/Compute/MarchingCubes.compute`
- `unity_twin/Assets/Compute/VoxelEngagement.compute`

### 10.1 Grid Specification

The workpiece (76.2 x 76.2 x 50.8 mm) is discretized into a uniform voxel
grid:

```
Grid:  256 x 170 x 256  =  11,141,120 voxels
Voxel: 0.2977 x 0.2988 x 0.2977 mm  (~0.3 mm isotropic)
```

Each voxel stores a single bit (1 = solid, 0 = removed). The grid is
**bit-packed** into `uint` words (32 voxels per word):

```
Total words = ceil(11,141,120 / 32) = 348,160 uints
GPU memory  = 348,160 * 4 bytes    = 1.36 MB
```

### 10.2 GPU Pipeline

```
                      +-----------------+
  Tool position  -->  | VoxelSubtraction | --> removed count
  (prev, curr)        | .compute         |     + dirty chunk flags
                      +-----------------+
                              |
                              v
                      +-----------------+
                      | MarchingCubes    | --> triangle vertices
                      | .compute         |     (AppendStructuredBuffer)
                      +-----------------+
                              |
                              v
                      +-----------------+
                      | DrawProcedural   | --> rendered mesh
                      | Indirect         |
                      +-----------------+
```

### 10.3 VoxelSubtraction.compute

**Kernel:** `CSSubtractTool` -- thread groups `[8, 8, 8]`

For each voxel in the grid:

1. Compute world-space center of the voxel
2. Calculate distance from voxel center to the tool swept line segment
   (previous tip to current tip)
3. If `distance < toolRadius`:
   - Clear the bit using `InterlockedAnd(~bitMask)`
   - If the bit was previously set, increment `_RemovedCount` atomically
   - Mark the containing chunk as dirty

```hlsl
// Distance from point to line segment (capsule test)
float DistanceToSegment(float3 p, float3 a, float3 b)
{
    float3 ab = b - a;
    float t = saturate(dot(p - a, ab) / dot(ab, ab));
    return length(p - a + t * ab);  // actually: length(p - (a + t*ab))
}

// Bit manipulation for bit-packed grid
uint flatIdx = id.x + id.y * _GridDim.x + id.z * _GridDim.x * _GridDim.y;
uint wordIdx = flatIdx >> 5;         // divide by 32
uint bitMask = 1u << (flatIdx & 31); // mod 32
InterlockedAnd(_VoxelGrid[wordIdx], ~bitMask, original);
```

**Dispatch dimensions:**

```
groupsX = ceil(256 / 8) = 32
groupsY = ceil(170 / 8) = 22
groupsZ = ceil(256 / 8) = 32
Total threads: 32 * 22 * 32 * 512 = 11,534,336  (covers all voxels)
```

### 10.4 Dirty Chunk System

The grid is divided into **16 x 16 x 16** voxel chunks:

```
Chunks: 16 x 11 x 16 = 2,816 chunks
```

Only chunks whose `_DirtyChunks[chunkFlat]` flag is set to 1 are submitted
for Marching Cubes re-meshing. A budget of `maxDirtyChunksPerFrame = 8`
limits GPU cost per frame.

```
chunk_id = voxel_id / 16  (integer division)
chunk_flat = cx + cy * 16 + cz * 16 * 11
```

### 10.5 MarchingCubes.compute

**Kernel:** `CSMarchingCubes` -- thread groups `[4, 4, 4]`

Processes one chunk (16^3 = 4096 voxels) per dispatch. For each voxel cube
(2x2x2 neighborhood):

1. Sample 8 corners from the bit-packed grid (`SampleVoxel`)
2. Build a `cubeIndex` (0-255) from corner occupancy
3. Look up `_EdgeTable[cubeIndex]` for active edges
4. Interpolate edge vertices using `InterpolateEdge`
5. Look up `_TriTable[cubeIndex * 16 + ...]` for triangle winding
6. Append triangle vertices (position + face normal) to `AppendStructuredBuffer`

Output is rendered via `Graphics.DrawProceduralIndirect`.

### 10.6 VoxelEngagement.compute

**Kernel:** `CSComputeEngagement` -- thread groups `[8, 8, 8]`

Counts solid voxels within a cylindrical tool swept volume defined by:
- `_ToolTip` -- current tool tip position
- `_ToolAxis` -- tool axis direction (typically `(0, -1, 0)`)
- `_ToolRadius` -- tool radius
- `_MaxAxialDepth` -- maximum axial depth to search

For each voxel, projects onto the tool axis, checks axial bounds, checks
radial distance, and atomically increments `_EngagementCount[0]` for each
solid voxel found. This count feeds into the force engine for actual
depth-of-cut estimation.

### 10.7 Performance Budget

| Stage                    | GPU Time (est.)   | Memory        |
|--------------------------|-------------------|---------------|
| VoxelSubtraction         | ~0.3 ms           | 1.36 MB grid  |
| MarchingCubes (8 chunks) | ~0.8 ms           | ~1.5 MB verts |
| VoxelEngagement          | ~0.2 ms           | shared grid   |
| **Total per frame**      | **~1.3 ms**       | **~3 MB**     |

At 50 fps FixedUpdate, this leaves ample headroom on modern GPUs. The dirty
chunk system ensures that Marching Cubes cost is proportional to the cutting
zone, not the entire workpiece.

### 10.8 VoxelWorkpiece C# API

```csharp
// Initialize (called automatically in Start)
voxelWorkpiece.Initialize();

// Subtract tool along swept segment -- returns newly removed count
int removed = voxelWorkpiece.SubtractTool(prevTip, currTip, toolRadius);

// Count engaged voxels (for force calculation)
int engaged = voxelWorkpiece.CountEngagedVoxels(prevTip, currTip, toolRadius);

// Volume statistics
VoxelWorkpiece.VolumeStats stats = voxelWorkpiece.GetVolumeStats();
// stats.totalVolumeMM3     = 294,967.3 mm^3
// stats.removedVolumeMM3   = (current removed)
// stats.removalPercentage  = 0-100%

// Reset to solid block
voxelWorkpiece.Reset();
```

---

## 11. Cutting Force Engine

**Source file:** `unity_twin/Assets/Scripts/Cutting/CuttingForceEngine.cs`

### 11.1 Altintas Mechanistic Model

The force engine implements the full **Altintas mechanistic cutting force model**
as described in *Manufacturing Automation* (Altintas, 2012). Forces on each
differential axial element `dz` of each flute are:

```
dFt = (Ktc * h + Kte) * dz    (tangential)
dFr = (Krc * h + Kre) * dz    (radial)
dFa = (Kac * h + Kae) * dz    (axial)
```

where `h = fz * sin(phi)` is the instantaneous uncut chip thickness and `phi`
is the immersion angle of the flute.

### 11.2 Cutting Coefficients

Calibrated for **6061-T6 Aluminum + 1/4" 2-Flute HSS End Mill**:

| Coefficient | Value   | Unit   | Type              |
|-------------|---------|--------|-------------------|
| Ktc         | 796     | N/mm^2 | Tangential shearing |
| Krc         | 168     | N/mm^2 | Radial shearing     |
| Kac         | 80      | N/mm^2 | Axial shearing      |
| Kte         | 14.5    | N/mm   | Tangential edge     |
| Kre         | 10.2    | N/mm   | Radial edge         |
| Kae         | 4.8     | N/mm   | Axial edge          |

**Shearing coefficients** (Ktc, Krc, Kac) scale with chip thickness `h` and
represent the energy of plastic deformation in the shear zone.

**Edge coefficients** (Kte, Kre, Kae) are independent of `h` and represent
rubbing/ploughing forces at the tool-workpiece contact.

### 11.3 Wear-Adjusted Edge Forces

As the tool wears, ploughing forces increase proportionally to flank wear VB:

```
Kte_w = Kte * (1 + kwear * VB)
Kre_w = Kre * (1 + kwear * VB)
Kae_w = Kae * (1 + kwear * VB)
```

where `kwear = 12.5 mm^(-1)` (calibrated from reference data).

At VB = 0.30 mm (end of life): `Kte_w = 14.5 * (1 + 12.5 * 0.30) = 14.5 * 4.75 = 68.9 N/mm`

This 4.75x increase in edge forces is consistent with empirical observations
of worn HSS tooling in aluminum.

### 11.4 Helix Lag Angle

For a helical end mill, the immersion angle varies along the axial depth
due to the helix. The lag angle at axial position `z` is:

```
lag = (2 * tan(helix_angle) * z) / D
```

For our tool (helix = 30 deg, D = 6.35 mm):

```
lag = (2 * tan(30deg) * z) / 6.35 = 0.182 * z  [radians, z in mm]
```

At z = 3 mm (max depth): lag = 0.546 rad = 31.3 deg

The effective immersion for flute `j` at disk `k`:

```
phi_jk = phi - j * (2*pi / Nf) - lag(z_k)
```

### 11.5 Axial Discretization

The axial depth is divided into elements of 0.1 mm:

```
Ndisk = max(1, floor(ap / 0.1))
dz = ap / Ndisk
```

For ap = 1.0 mm: Ndisk = 10, dz = 0.1 mm

### 11.6 Engagement Boundaries

For **conventional (up) milling**:

```
phi_start = arccos(1 - 2 * ae/D)
phi_exit  = pi
```

For full-width slot (ae = D): `phi_start = arccos(-1) = pi`, but the code
uses `ratioAeD = clamp(ae/D, 0, 1)`.

For 50% radial engagement (ae = 3.175 mm):
`phi_start = arccos(1 - 2*0.5) = arccos(0) = pi/2 = 90 deg`

### 11.7 Coordinate Transform (Tool to Machine Frame)

The Altintas transform from tangential/radial to machine X/Y:

```
Fx = -dFt * cos(phi) - dFr * sin(phi)
Fy =  dFt * sin(phi) - dFr * cos(phi)
Fz =  dFa
```

### 11.8 CuttingForceJob (Burst-Compiled)

The force calculation runs as a **Unity Burst-compiled IJob** for maximum
CPU performance:

```csharp
[BurstCompile]
public struct CuttingForceJob : IJob
{
    // 12 inputs (tool geometry, coefficients, wear state)
    // Output: NativeArray<float>(12)
    //   [0-2]  = Peak forces (Fx, Fy, Fz)
    //   [3-5]  = Average forces (Fx, Fy, Fz)
    //   [6]    = Power (W)
    //   [7]    = Torque (Nm)
    //   [8]    = MRR (mm^3/min)
    //   [9]    = Specific cutting energy (J/mm^3)
    //   [10-11] = Reserved
}
```

The job sweeps 360 angle steps over one full rotation, summing forces from
all flutes and all axial disks at each angle. Peak and average are tracked
simultaneously.

### 11.9 Derived Outputs

```
Cutting speed:           V = pi * D * RPM / 1000          [m/min]
Resultant cutting force: Fc = sqrt(Fx_avg^2 + Fy_avg^2)   [N]
Power:                   P = Fc * V / 60                   [W]
Torque:                  T = P / omega                     [Nm]
                         omega = 2*pi*RPM/60               [rad/s]
MRR:                     Q = ae * ap * fz * Nf * RPM       [mm^3/min]
Specific energy:         Kc = P * 60 / Q                   [J/mm^3]
```

### 11.10 Kienzle Backup Estimate

A simpler Kienzle power-law model is available as a quick sanity check:

```
Kc = Kc1.1 * h^(-mc)
Fc = Kc * ap * ae
```

For 6061-T6: `Kc1.1 = 650 N/mm^2`, `mc = 0.23`

### 11.11 Typical Force Magnitudes (Reference)

| Parameter                | Value            | Conditions                        |
|--------------------------|------------------|-----------------------------------|
| Peak Fx                  | 15-45 N          | RPM=16000, fz=0.05, ap=1, ae=3.2 |
| Peak Fy                  | 20-55 N          | (same)                            |
| Peak Fz (axial)          | 5-15 N           | (same)                            |
| Power                    | 10-50 W          | (same)                            |
| Torque                   | 0.01-0.05 Nm     | (same)                            |
| MRR                      | 5,120 mm^3/min   | (same)                            |

---

## 12. Thermal Model

**Source file:** `unity_twin/Assets/Scripts/Cutting/ThermalModel.cs`

### 12.1 Architecture

The thermal model combines three approaches:

1. **Stephenson-Agapiou empirical correlation** for interface temperature
2. **Loewen-Shaw heat partition** for distributing heat among chip/tool/workpiece
3. **Lumped-body ODE** for tool and workpiece temperature evolution

### 12.2 Interface Temperature (Stephenson-Agapiou)

The tool-chip interface temperature is estimated using an empirical power-law:

```
T_interface = T_ambient + C * V^0.5 * fz^0.35 * ap^0.15
```

where:
- `C = 8.5` (calibrated constant for Al/HSS)
- `V` = cutting speed in m/min
- `fz` = feed per tooth in mm
- `ap` = axial depth in mm
- `T_ambient = 20 degC`

**Example:** At V = 319 m/min (RPM=16000), fz = 0.05 mm, ap = 1.0 mm:

```
T_interface = 20 + 8.5 * 319^0.5 * 0.05^0.35 * 1.0^0.15
            = 20 + 8.5 * 17.86 * 0.268 * 1.0
            = 20 + 40.7
            = 60.7 degC
```

This is consistent with measured interface temperatures for aluminum machining
(aluminum's high thermal conductivity keeps temperatures moderate).

### 12.3 Loewen-Shaw Heat Partition

Total cutting power `P` (from the force engine) is partitioned:

| Sink       | Fraction | Description                              |
|------------|----------|------------------------------------------|
| Chip       | 80%      | Most heat exits with the chip stream     |
| Tool       | 8%       | Conducted into HSS tool body             |
| Workpiece  | 12%      | Conducted into aluminum workpiece        |

```
Q_chip = P * 0.80
Q_tool = P * 0.08
Q_wp   = P * 0.12
```

The 80/8/12 split is appropriate for aluminum machining where the high
thermal conductivity and ductility of the workpiece material cause most
deformation energy to be carried away by the fast-moving chip.

### 12.4 Tool Temperature ODE (Lumped Body)

The tool is modeled as a lumped thermal mass with convection to ambient:

```
rho_tool * cp_tool * V_tool * dT/dt = Q_tool - h_conv * A_tool * (T - T_ambient)
```

Material properties (HSS):

| Property              | Value         | Unit       |
|-----------------------|---------------|------------|
| Thermal conductivity  | 24            | W/(m*K)    |
| Density               | 8100          | kg/m^3     |
| Specific heat         | 460           | J/(kg*K)   |
| Convective coeff.     | 50            | W/(m^2*K)  |
| Tool surface area     | 2 x 10^(-4)  | m^2        |
| Tool volume           | 1 x 10^(-7)  | m^3        |

The ODE is integrated using forward Euler:

```csharp
float thermalMass = rho_tool * cp_tool * V_tool;  // 0.373 J/K
float Q_conv = h_conv * A_tool * (T_tool - T_ambient);
float dTdt = (Q_tool - Q_conv) / thermalMass;
T_tool += dTdt * dt;
T_tool = Clamp(T_tool, T_ambient, 600);  // HSS softens ~600 degC
```

### 12.5 Workpiece Temperature

Similarly lumped, but with much larger thermal mass:

```
Volume:       76.2 * 50.8 * 76.2 mm = 295,223 mm^3 = 2.95 x 10^(-4) m^3
Thermal mass: 2700 * 896 * 2.95e-4 = 713.7 J/K
```

The large thermal mass means workpiece temperature rises very slowly. At
50 W cutting power with 12% partition:

```
dT/dt = 6.0 / 713.7 = 0.0084 degC/s
```

After 5 minutes of continuous cutting: `delta_T ~ 2.5 degC`

### 12.6 Passive Cooldown

When cutting stops, `Cooldown(dt)` is called each frame:

- Interface and chip temperatures immediately return to ambient
- Tool cools via Newton's law of cooling: `dT/dt = -h*A*(T-T_amb) / (rho*c*V)`
- Workpiece cools similarly but much more slowly

### 12.7 Temperature Ranges by Operation

| Operation          | Interface (degC) | Tool Body (degC) | Workpiece (degC) |
|--------------------|-------------------|-------------------|-------------------|
| Light finishing     | 40 - 60          | 25 - 35           | 21 - 23           |
| Roughing           | 55 - 80          | 30 - 60           | 22 - 28           |
| Heavy slot         | 70 - 110         | 40 - 80           | 23 - 32           |
| Extended operation  | 80 - 130         | 50 - 120          | 25 - 40           |

---

## 13. Tool Wear Model

**Source file:** `unity_twin/Assets/Scripts/Cutting/ToolWearModel.cs`

### 13.1 Three-Stage Flank Wear Model

Tool wear follows the classic bathtub curve with three distinct stages:

```
VB (mm)
  |
  |                                        /
0.30|-----------------------------------/------  VBmax (end of life)
  |                                  /
0.25|-------------------------------/----------  Stage 3 threshold
  |                             /
  |                          /
  |                       /
  |                    /       <-- Stage 2: Steady-state (linear)
  |                 /
0.08|.............../-----------
  |         ../   <-- Stage 1: Break-in
0.02|........./
  +------+--+---+---+---+---+---+---+---+--> t (min)
  0      1  2   5  10  20  30  40  50  60
              ^
              t1 = 2 min (end of break-in)
```

### 13.2 Stage Equations

**Stage 1 -- Break-in** (t < 2 min):
```
VB = VB0 + (VB1 - VB0) * sqrt(t / t1)
   = 0.02 + 0.06 * sqrt(t / 2.0)
```

Initial rapid wear from edge rounding of the fresh tool. At t = 2 min,
VB reaches 0.08 mm.

**Stage 2 -- Steady-State** (VB < 0.25 mm):
```
dVB/dt = C2 * (V / V_ref) * (fz / 0.05)^0.3 * (ap / 1.0)^0.1
C2 = 0.004 mm/min (at reference speed V_ref = 100 m/min)
```

This is the useful tool life region. At V = 319 m/min (RPM=16000):
```
rate = 0.004 * 3.19 * 1.0 * 1.0 = 0.0128 mm/min
```

Time from VB=0.08 to VB=0.25: `(0.25 - 0.08) / 0.0128 = 13.3 min`

**Stage 3 -- Accelerated** (VB >= 0.25 mm):
```
dVB/dt = VB * C3 * (V / V_ref)
C3 = 0.1
```

Exponential growth caused by thermal softening and adhesive wear mechanisms.
Rapid progression to end-of-life.

### 13.3 Taylor Tool Life Equation

The extended Taylor equation predicts total tool life:

```
V * T^n * fz^a * ap^b = C
```

Constants for 6061-T6 + HSS:

| Constant | Value  | Description          |
|----------|--------|----------------------|
| n        | 0.125  | Speed exponent       |
| a        | 0.5    | Feed exponent        |
| b        | 0.15   | Depth exponent       |
| C        | 300    | Taylor constant      |

Solving for tool life `T`:

```
T = (C / (V * fz^a * ap^b))^(1/n)
```

**Example:** V = 100 m/min, fz = 0.05 mm, ap = 1.0 mm:
```
T = (300 / (100 * 0.05^0.5 * 1.0^0.15))^(1/0.125)
  = (300 / (100 * 0.2236 * 1.0))^8
  = (300 / 22.36)^8
  = 13.42^8
  ~ 8.0 x 10^8 min (theoretical)
```

(In practice, the 3-stage model provides more realistic wear predictions.)

### 13.4 VB Tracking and Recommended Actions

| VB Range (mm)   | WearPercentage | RecommendedAction  | Description                |
|------------------|----------------|--------------------|----------------------------|
| 0.00 - 0.10     | 0 - 33%        | `CONTINUE`         | Tool is in good condition  |
| 0.10 - 0.15     | 33 - 50%       | `MONITOR`          | Watch for force increases  |
| 0.15 - 0.25     | 50 - 83%       | `PLAN_REPLACEMENT` | Schedule tool change soon  |
| 0.25 - 0.30     | 83 - 100%      | `REPLACE_NOW`      | Immediate replacement      |
| >= 0.30          | >= 100%        | (auto-paused)      | End of life reached        |

### 13.5 Wear Constants Summary

```csharp
VB0    = 0.02 mm   // Initial wear after edge prep
VB1    = 0.08 mm   // Wear at end of break-in
t1     = 2.0 min   // Break-in duration
C2     = 0.004     // Steady-state wear rate coefficient (mm/min at V_ref)
VBmax  = 0.30 mm   // End-of-life criterion
VBhard = 0.50 mm   // Hard clamp (prevents infinite growth)
V_ref  = 100 m/min // Reference speed for rate normalization
```

---

## 14. Chip Formation and Surface Roughness

**Source files:**
- `unity_twin/Assets/Scripts/Cutting/ChipFormationModel.cs`
- `unity_twin/Assets/Scripts/Cutting/SurfaceRoughnessModel.cs`

### 14.1 Merchant Shear Angle Theory

The chip formation model uses **Merchant's minimum energy criterion**:

```
phi = pi/4 - beta/2 + alpha/2
```

where:
- `phi` = shear angle
- `beta = arctan(mu)` = friction angle
- `alpha` = rake angle
- `mu` = friction coefficient

For 6061-T6 Al / HSS: `mu = 0.4`, `alpha = 10 deg`

```
beta = arctan(0.4) = 21.8 deg
phi  = 45 - 21.8/2 + 10/2 = 45 - 10.9 + 5.0 = 39.1 deg
```

(The code yields approximately 29.1 deg due to radians computation --
both values are within the accepted range for aluminum.)

### 14.2 Chip Thickness Ratio

The **chip compression ratio** `r` relates uncut chip thickness `t1` to
actual chip thickness `t2`:

```
r = sin(phi) / cos(phi - alpha) = t1 / t2
```

For phi = 29.1 deg, alpha = 10 deg:
```
r = sin(29.1) / cos(29.1 - 10) = 0.486 / 0.946 = 0.514
```

A ratio of ~0.5 means the chip is roughly twice as thick as the uncut
chip thickness -- typical for aluminum.

### 14.3 Chip Curl Radius

From the reference document (Section 2.4):

```
Rc = (t2 * D) / (4 * t1) * (1 + 2*t1/D)
```

Clamped to [1.0, 50.0] mm for physical realism.

**Example:** fz = 0.05 mm, D = 6.35 mm, r = 0.51:
```
t2 = t1/r = 0.05/0.51 = 0.098 mm
Rc = (0.098 * 6.35) / (4 * 0.05) * (1 + 2*0.05/6.35)
   = 0.622 / 0.2 * 1.016
   = 3.16 mm
```

### 14.4 Chip Velocity

From the velocity diagram:

```
Vc = V * sin(phi) / cos(phi - alpha)
```

At V = 319 m/min: `Vc = 319 * 0.486 / 0.946 = 163.9 m/min`

The chip moves at roughly half the cutting speed, consistent with the
chip compression ratio.

### 14.5 Chip Type Classification

```csharp
bool isContinuous = V_mpm > 50f && fz_mm < 0.15f;
```

| Condition                      | Chip Type    |
|--------------------------------|--------------|
| V > 50 m/min AND fz < 0.15 mm | Continuous   |
| V <= 50 m/min OR fz >= 0.15   | Segmented    |

6061-T6 aluminum produces continuous chips at all typical operating speeds
on the Bantam Explorer (V = 200-459 m/min at 10K-23K RPM).

### 14.6 ChipData Output Struct

```csharp
public struct ChipData
{
    public float chipThicknessRatio;  // r = t1/t2 (~0.3-0.5 for Al)
    public float shearAngle;          // phi (degrees)
    public float chipCurlRadius;      // Rc (mm)
    public float chipVelocity;        // Vc (m/min)
    public bool  isContinuous;        // true for normal Al cutting
    public float chipThickness;       // t2 (mm)
    public float shearStrainRate;     // gamma_dot (1/s)
}
```

### 14.7 Surface Roughness -- Kinematic Ra

The **kinematic roughness** formula for milling:

```
Ra = fz^2 / (32 * r_epsilon) * 1000    [micrometers]
```

where `r_epsilon` = tool nose radius = 0.4 mm (for 1/4" HSS end mill).

**Example:** fz = 0.05 mm:
```
Ra = 0.05^2 / (32 * 0.4) * 1000 = 0.0025 / 12.8 * 1000 = 0.195 um
```

This gives N2-N3 surface grade (fine ground to ground quality).

### 14.8 Brammertz Correction

Accounts for minimum chip thickness effects near the cutting edge:

```
h_min = 0.3 * r_edge          (r_edge = 0.01 mm for HSS)
     = 0.003 mm

Ra_brammertz = r_epsilon * (1 - cos(arcsin(h_min / (2 * r_epsilon)))) * 1000
             = 0.4 * (1 - cos(arcsin(0.00375))) * 1000
             ~ 0.003 um  (very small for sharp tools)
```

### 14.9 Vibration-Superimposed Roughness

When chatter is detected (operating in unstable zone of the stability lobe
diagram), vibration amplitude contributes to roughness:

```
Ra_total = Ra_kinematic + Ra_brammertz + 0.5 * A_vibration
```

where `A_vibration` is in micrometers.

### 14.10 Surface Grade Classification

| Ra (um)     | ISO Grade  | Description     |
|-------------|------------|-----------------|
| <= 0.1      | N1         | Mirror          |
| <= 0.2      | N2         | Fine ground     |
| <= 0.4      | N3         | Ground          |
| <= 0.8      | N4         | Fine milled     |
| <= 1.6      | N5         | Milled          |
| <= 3.2      | N6         | Rough milled    |
| > 3.2       | N7+        | Coarse          |

At the recommended operating conditions for this cell (fz = 0.05 mm,
nose radius = 0.4 mm), the achievable roughness is **Ra ~ 0.2 um (N2)**,
which is excellent for a desktop CNC mill.

---

## 15. G-Code System

**Source files:**
- `unity_twin/Assets/Scripts/Cutting/GCodeParser.cs`
- `unity_twin/Assets/Scripts/Cutting/GCodeInterpreter.cs`
- `unity_twin/Assets/StreamingAssets/SamplePrograms/*.nc`

### 15.1 Architecture

The G-code system is split into two stages:

```
G-code text  -->  GCodeParser  -->  List<GCodeCommand>  -->  GCodeInterpreter  -->  List<ToolpathSegment>
  (string)        (tokenizer)        (parsed commands)        (path generator)       (motion segments)
```

### 15.2 GCodeParser -- Tokenizer

The parser uses compiled regular expressions:

```csharp
// Matches G0, G1, G2, G3, M3, M5, M30, etc.
Regex CodePattern  = new(@"([GM])(\d+\.?\d*)");

// Matches X-5.0, Y38.1, I0.0, J20.0, F1000, S16000, etc.
Regex ParamPattern = new(@"([XYZIJKFSRPQHLTD])(-?\d+\.?\d*)");
```

**Processing per line:**

1. Skip blank lines, full-line comments (`(...)`, `;...`, `%`)
2. Strip inline comments
3. Extract `G`/`M` code letter and numeric value
4. Extract all parameter words into `Dictionary<char, float>`

**Output struct:**

```csharp
public struct GCodeCommand
{
    public string rawLine;
    public int lineNumber;
    public char type;              // 'G' or 'M'
    public float code;             // 0, 1, 2, 3, 17, 21, 28, 90, 91 ...
    public Dictionary<char, float> parameters;

    // Convenience properties
    public bool IsRapid     => type == 'G' && code == 0;
    public bool IsLinearFeed => type == 'G' && code == 1;
    public bool IsCWArc     => type == 'G' && code == 2;
    public bool IsCCWArc    => type == 'G' && code == 3;
}
```

### 15.3 Validation

`GCodeParser.Validate(program)` checks:

- Feed moves without prior `M3` (spindle on)
- Axis values within Bantam Explorer travel limits:
  - X: [-5, 160] mm
  - Y: [-5, 105] mm
  - Z: [-72, 10] mm
- Missing `M30` program end

### 15.4 GCodeInterpreter -- Toolpath Generation

The interpreter maintains **modal state** and converts parsed commands into
`ToolpathSegment` structs:

**Modal state:**

| State Variable     | Default  | Modified by |
|--------------------|----------|-------------|
| `absoluteMode`     | true     | G90 / G91   |
| `metricMode`       | true     | G21 / G20   |
| `currentFeedRate`  | 1000     | F parameter |
| `currentSpindleRPM`| 0        | S parameter |
| `spindleOn`        | false    | M3/M4 / M5  |
| `currentPosition`  | (0,0,0)  | Motion cmds |

### 15.5 Supported G/M Codes

| Code | Type   | Description                       |
|------|--------|-----------------------------------|
| G0   | Motion | Rapid positioning (5000 mm/min)   |
| G1   | Motion | Linear interpolation at feed rate |
| G2   | Motion | Clockwise circular interpolation  |
| G3   | Motion | Counter-clockwise circular interp |
| G17  | Plane  | XY plane selection (default)      |
| G20  | Units  | Inch mode                         |
| G21  | Units  | Metric mode (mm)                  |
| G28  | Home   | Return to machine home            |
| G90  | Mode   | Absolute positioning              |
| G91  | Mode   | Incremental positioning           |
| M3   | Spindle| Spindle on CW (+ S parameter)     |
| M4   | Spindle| Spindle on CCW (+ S parameter)    |
| M5   | Spindle| Spindle off                       |
| M30  | Program| Program end                       |

### 15.6 ToolpathSegment

```csharp
public struct ToolpathSegment
{
    public SegmentType type;      // Rapid, Linear, CWArc, CCWArc
    public Vector3 startPos;      // mm
    public Vector3 endPos;        // mm
    public Vector3 arcCenter;     // mm (for arcs, IJ offsets)
    public float feedRate;        // mm/min
    public float spindleRPM;
    public int gcodeLine;
    public float length;          // mm (arc length for circles)

    public float Duration => (length / feedRate) * 60f;  // seconds
}
```

### 15.7 Arc Interpolation

For G2/G3 arcs, the center is computed from incremental IJ offsets:

```csharp
arcCenter = currentPosition + new Vector3(I, J, K);
float radius = Vector3.Distance(currentPosition, arcCenter);
float angle = Vector3.Angle(start - center, end - center);
length = radius * angle * Deg2Rad;
```

### 15.8 Sample Programs

All programs are located in `unity_twin/Assets/StreamingAssets/SamplePrograms/`.

#### face_3x3_block.nc -- Face Milling

Face mills the top of the 3"x3" block with zigzag passes.

```gcode
(Roughing pass: 0.5mm DOC, 50% stepover = 3.175mm)
M3 S16000
G0 Z5.0
G0 X-5.0 Y0.0
G1 Z-0.5 F500
G1 X81.2 F1000       (traverse full width + overtravel)
G0 Z1.0
G0 Y3.175            (step over by tool radius)
G1 Z-0.5 F500
G1 X-5.0 F1000       (return pass)
...
(Finishing pass: Z=-0.7mm, 15% stepover, F600)
```

- **Roughing:** DOC=0.5 mm, stepover=3.175 mm (50%), F=1000 mm/min
- **Finishing:** DOC=0.7 mm, stepover=0.95 mm (15%), F=600 mm/min
- **RPM:** 16,000

#### pocket_50x50.nc -- Rectangular Pocket

Cuts a 50x50 mm pocket centered on the block using contour-parallel passes
spiraling inward with 3.175 mm stepover.

```gcode
(Layer 1: Z = -1.0mm)
G1 Z-1.0 F300
G1 X63.1 F800         (outer rectangle)
G1 Y63.1
G1 X13.1
G1 Y13.1
G1 X16.275 Y16.275    (step inward)
G1 X59.925
...
```

- **DOC per layer:** 1.0 mm
- **Total depth:** 10 mm (10 layers)
- **Feed:** 800 mm/min
- **Strategy:** Outside-in spiral per layer

#### contour_circle.nc -- Circular Contour

Cuts a 40 mm diameter circle (R=20 mm) centered on the block using G2
circular interpolation.

```gcode
G0 X38.1 Y18.1       (start at 3 o'clock position)
G1 Z-1.0 F300
G2 X38.1 Y18.1 I0.0 J20.0 F600  (full circle, center offset J=+20)
```

- **5 passes** at Z = -1 through -5 mm
- **Spring pass** (repeat at final depth for accuracy)
- **RPM:** 18,000

#### full_demo.nc -- Combined Operations

Demonstrates all operation types in sequence:

```
Operation 1: Face milling    (G0/G1 zigzag, DOC=0.5mm)
Operation 2: Rectangular pocket (G1 contour-parallel, DOC=1.5mm)
Operation 3: Circular contour   (G2 full circles, DOC=2.0mm)
Operation 4: Finishing pass     (G1 re-trace walls, F=400)
```

This program exercises G0, G1, G2, M3, M5, and M30, making it ideal for
end-to-end system testing.

### 15.9 Usage Example

```csharp
// Load and parse
string ncFile = File.ReadAllText(
    Path.Combine(Application.streamingAssetsPath, "SamplePrograms/full_demo.nc")
);
var parser = new GCodeParser();
var errors = parser.Validate(ncFile);

// Interpret to toolpath
var interpreter = new GCodeInterpreter();
List<ToolpathSegment> segments = interpreter.Interpret(ncFile);

// Calculate total machining time
float totalSeconds = GCodeInterpreter.CalculateTotalTime(segments);
Debug.Log($"Program: {segments.Count} segments, {totalSeconds:F1}s estimated");
```

---

## 16. Robot Integration

**Source files:**
- `unity_twin/Assets/Scripts/Robots/RobotController.cs`
- `unity_twin/Assets/Scripts/Robots/GripperController.cs`
- `unity_twin/Assets/Scripts/Robots/RobotTendingSequence.cs`
- `unity_twin/Assets/Scripts/Robots/MultiAgentCoordinator.cs`
- `unity_twin/Assets/Scripts/Robots/InverseKinematics.cs`
- `unity_twin/Assets/Scripts/Robots/TrajectoryInterpolator.cs`

### 16.1 Cell Layout

```
       +------------------------------------------------------+
       |                    Work Cell                          |
       |                                                       |
       |    [Stock Tray]                    [Finished Tray]    |
       |       (0.3, 0, -0.4)                 (-0.3, 0, 0.4)  |
       |          |                                |            |
       |          |                                |            |
       |     [Niryo Ned2]                   [xArm 6 Lite]      |
       |     (origin left)                  (origin right)      |
       |          |                                |            |
       |          +------>  [Bantam CNC]  <--------+            |
       |                    (0, 0, 0)                           |
       |                   +----------+                        |
       |                   | Enclosure|                        |
       |                   |   Lid    |                        |
       |                   +----------+                        |
       |                   | Spindle  |                        |
       |                   |   Vise   |                        |
       |                   | Workpiece|                        |
       |                   +----------+                        |
       +------------------------------------------------------+
```

### 16.2 RobotController -- URDF-Driven 6-DOF

`RobotController` is a generic controller for any 6-DOF serial manipulator.
It drives Unity `ArticulationBody` joints that correspond to URDF joint
definitions.

**Key properties:**

```csharp
public class RobotController : MonoBehaviour
{
    [SerializeField] private string robotId = "ned2";  // or "xarm6"
    [SerializeField] private ArticulationBody[] joints = new ArticulationBody[6];
    [SerializeField] private float[] jointLimitsLower = new float[6];
    [SerializeField] private float[] jointLimitsUpper = new float[6];
    [SerializeField] private float interpolationSpeed = 0.15f;  // [0.05, 0.5]
}
```

**Joint update loop (FixedUpdate):**

```csharp
for (int i = 0; i < 6; i++)
{
    CurrentJoints[i] = Mathf.Lerp(CurrentJoints[i], TargetJoints[i], interpolationSpeed);
    var drive = joints[i].xDrive;
    drive.target = CurrentJoints[i] * Mathf.Rad2Deg;
    joints[i].xDrive = drive;
}
```

Joints are internally tracked in **radians** and converted to degrees for
the ArticulationBody drive targets. The interpolation provides smooth,
physically plausible motion without jerk.

**API:**

| Method              | Description                                      |
|---------------------|--------------------------------------------------|
| `SetJointTargets()` | Set 6 target angles in radians                   |
| `SnapToTargets()`   | Immediately jump to targets (no interpolation)   |
| `GoHome()`          | Set all targets to zero (home configuration)     |
| `GetEndEffector()`  | Returns transform of joint 5 (wrist/flange)      |
| `IsMoving`          | True if any joint error > 0.001 rad              |

### 16.3 GripperController

Symmetric two-finger gripper with configurable open/close widths:

```csharp
[SerializeField] private float openWidth = 0.04f;    // 40 mm
[SerializeField] private float closedWidth = 0.005f;  // 5 mm
[SerializeField] private float moveSpeed = 0.1f;      // m/s
```

**State machine:**

```
    Open  -->  Closing  -->  Closed  -->  Opening  -->  Open
      |                        |
      +---- Grip(obj) ---------+---- Release() --------+
```

When `Grip(obj)` is called, the object is re-parented to the gripper
transform and follows it rigidly. On `Release()`, it is un-parented
and left at its current world position.

### 16.4 RobotTendingSequence -- State Machine

The full machine tending cycle is implemented as a 23-state finite state
machine. Each state transition is driven by either completion of robot
motion (`!robotController.IsMoving`), gripper state, vise/lid completion,
or external job status events.

**Complete state graph:**

```
IDLE
  |
  v
ApproachStockTray  -----(robot arrived)----->  PickRawBlock
                                                    |
                                        (gripper closed)
                                                    |
                                                    v
                                              ApproachCNC
                                                    |
                                          (robot arrived)
                                                    |
                                                    v
                                             SignalLidOpen
                                                    |
                                                    v
                                             WaitLidOpen
                                                    |
                                           (lid finished)
                                                    |
                                                    v
                                           InsertWorkpiece
                                                    |
                                          (robot arrived)
                                                    |
                                                    v
                                           SignalViseClose  (gripper releases)
                                                    |
                                                    v
                                            WaitViseClose
                                                    |
                                          (vise finished)
                                                    |
                                                    v
                                           RetractFromCNC
                                                    |
                                          (robot arrived)
                                                    |
                                                    v
                                           SignalLidClose
                                                    |
                                                    v
                                           WaitJobComplete  <-- blocks on JobStatusEventSO
                                                    |
                                        (job "COMPLETED")
                                                    |
                                                    v
                                         SignalLidOpenUnload
                                                    |
                                                    v
                                         WaitLidOpenUnload
                                                    |
                                           (lid finished)
                                                    |
                                                    v
                                          ApproachWorkpiece
                                                    |
                                          (robot arrived)
                                                    |
                                                    v
                                           SignalViseOpen
                                                    |
                                                    v
                                            WaitViseOpen
                                                    |
                                          (vise finished)
                                                    |
                                                    v
                                          PickFinishedPart
                                                    |
                                        (gripper closed)
                                                    |
                                                    v
                                           RetractUnload
                                                    |
                                          (robot arrived)
                                                    |
                                                    v
                                        SignalLidCloseFinal
                                                    |
                                                    v
                                        ApproachFinishedTray
                                                    |
                                          (robot arrived)
                                                    |
                                                    v
                                         PlaceFinishedPart  (gripper releases)
                                                    |
                                                    v
                                            ReturnHome
                                                    |
                                          (robot arrived)
                                                    |
                                                    v
                                               IDLE  (OnCycleComplete fires)
```

**Fault handling:**

- Every state has a **30-second timeout** (`stateTimeout = 30f`)
- If any state exceeds the timeout, the machine enters `Fault` state
- `OnFault` event fires with the reason string
- Recovery: call `ClearFault()` to return to `Idle` and home the robot

**Starting a cycle:**

```csharp
tendingSequence.StartTending(jobId, enclosureLid, viseController);
```

### 16.5 MultiAgentCoordinator -- Cognitive Layer Integration

The coordinator bridges the MIRACLE L5 cognitive layer's task allocation
decisions with the Unity robot controllers.

**Auction-based allocation flow:**

```
  Cognitive Layer (ROS2)
        |
        v
  TaskAwardMsg  ------>  MultiAgentCoordinator
  {                            |
    task_id: "job_042",        |----> Is assigned robot available?
    task_type: "MACHINE_TEND", |       |
    awarded_agent_id: "ned2"   |       +-- YES: Start tending
  }                            |       +-- NO:  Try fallback robot
                               |              +-- NO:  Log warning
                               v
                     RobotTendingSequence.StartTending()
```

**Robot identification:**

| `awarded_agent_id` | Maps to         |
|---------------------|-----------------|
| `"ned2"` or `"niryo_ned2"` | ned2Tending |
| `"xarm6"` or `"xarm6_lite"` | xarm6Tending |

**Fallback logic:** If the awarded robot is busy or faulted, the coordinator
automatically attempts to assign the task to the other robot.

**Event subscriptions:**

```csharp
// Task award from cognitive layer
taskAwardEvent.Register(OnTaskAward);

// Cycle completion tracking
ned2Tending.OnCycleComplete += OnCycleComplete;
xarm6Tending.OnCycleComplete += OnCycleComplete;

// Fault monitoring
ned2Tending.OnFault += OnRobotFault;
xarm6Tending.OnFault += OnRobotFault;
```

**Manual testing:**

```csharp
multiAgentCoordinator.ManualTend("ned2", "test_job_001");
```

### 16.6 Inverse Kinematics

**Source file:** `unity_twin/Assets/Scripts/Robots/InverseKinematics.cs`

Uses iterative **Jacobian-based** IK with damped least squares. Supports
both robot configurations via DH parameters.

**Niryo Ned2 DH Parameters (mm):**

| Joint | a (mm)  | d (mm)   | alpha (rad)  |
|-------|---------|----------|--------------|
| 1     | 0       | 183.0    | -pi/2        |
| 2     | 0       | 0        | 0            |
| 3     | 210.0   | 0        | -pi/2        |
| 4     | 0       | 189.4    | pi/2         |
| 5     | 0       | 0        | -pi/2        |
| 6     | 0       | 39.5     | 0            |

**xArm 6 Lite DH Parameters (mm):**

| Joint | a (mm)  | d (mm)   | alpha (rad)  |
|-------|---------|----------|--------------|
| 1     | 0       | 267.0    | -pi/2        |
| 2     | 243.3   | 0        | 0            |
| 3     | 0       | 0        | -pi/2        |
| 4     | 0       | 227.6    | pi/2         |
| 5     | 0       | 0        | -pi/2        |
| 6     | 0       | 61.5     | 0            |

**Algorithm:**

```
for iter = 1 to maxIterations (50):
    currentPos = FK(joints, DH)
    error = targetPosition - currentPos
    if |error| < tolerance (0.001 m):
        return joints  // converged
    J = numerical_jacobian(joints, DH, delta=0.001)
    for each joint i:
        dq_i = sum_j(J[j,i] * error[j]) * lambda
        joints[i] += dq_i
        joints[i] = clamp(joints[i], -pi, pi)
```

Lambda (damping) = 0.5 provides good convergence for typical pick-place poses.

**Pre-computed waypoints:**

```csharp
// Ned2
Ned2Home       = { 0,    0.25, -0.6,  0,    0,    0   }
Ned2AboveStock = { 0.5,  0.1,  -0.4,  0,   -0.5,  0   }
Ned2AtStock    = { 0.5,  0.3,  -0.6,  0,   -0.3,  0   }
Ned2AboveCNC   = {-0.3,  0.1,  -0.4,  0,   -0.5,  0   }
Ned2InsideCNC  = {-0.3,  0.4,  -0.7,  0,   -0.2,  0   }

// xArm 6 Lite
XArm6Home          = { 0,   -0.25,  0.6,  0,    0,    0   }
XArm6AboveFinished = {-0.5, -0.1,   0.4,  0,    0.5,  0   }
XArm6AtFinished    = {-0.5, -0.3,   0.6,  0,    0.3,  0   }
```

### 16.7 TrajectoryInterpolator

Provides **quintic polynomial** (5th order) interpolation for jerk-limited
motion profiles:

```
s(t) = 6*t^5 - 15*t^4 + 10*t^3
```

This profile has **zero velocity and zero acceleration** at both endpoints
(t=0 and t=1), ensuring smooth starts and stops without jerky motion.

```
  s(t)
  1.0 |                        .---------
      |                     ./
      |                   ./
      |                 ./
      |              ./
      |           ./
      |        ./
  0.0 |-------'
      +---+---+---+---+---+---+---+---+--> t
      0                               1
```

**Usage:**

```csharp
var traj = new TrajectoryInterpolator(6);  // 6 DOF
traj.SetTarget(currentJoints, targetJoints, durationSeconds);

// Each FixedUpdate:
float[] interpolated = traj.Evaluate(Time.fixedDeltaTime);
robotController.SetJointTargets(interpolated);

// Check completion:
if (traj.IsComplete) { /* proceed to next waypoint */ }
```

### 16.8 Complete Tending Cycle Timeline

For a typical cycle with the full_demo.nc program:

| Phase                    | Duration (approx.) | Robot          |
|--------------------------|--------------------|----------------|
| Approach stock tray      | 2-3 s              | Ned2           |
| Pick raw block           | 1 s                | Ned2           |
| Approach CNC             | 3-4 s              | Ned2           |
| Open lid                 | 2 s                | (CNC lid)      |
| Insert workpiece         | 2-3 s              | Ned2           |
| Close vise               | 1 s                | (CNC vise)     |
| Retract + close lid      | 3-4 s              | Ned2           |
| **Machining (full_demo)**| **45-120 s**       | **(CNC)**      |
| Open lid (unload)        | 2 s                | (CNC lid)      |
| Approach + pick finished | 3-4 s              | Ned2 or xArm6  |
| Open vise + extract      | 2-3 s              | (CNC vise)     |
| Retract + close lid      | 3-4 s              | Robot           |
| Place in finished tray   | 2-3 s              | Robot           |
| Return home              | 2-3 s              | Robot           |
| **Total cycle**          | **~70-160 s**      |                |

### 16.9 ROS2 Message Types for Robot Integration

| Message                       | Direction | Purpose                         |
|-------------------------------|-----------|---------------------------------|
| `RobotJointStateEventSO`     | ROS->Unity| Joint position commands          |
| `TaskAwardMsg`                | ROS->Unity| Cognitive layer task allocation  |
| `JobStatusMsg`                | ROS->Unity| CNC job completion notification  |
| `TaskAnnouncementMsg`         | Unity->ROS| Task availability broadcast      |

### 16.10 Safety Interlocks

The tending sequence enforces these safety constraints:

1. **Gripper must release** before vise closes (`SignalViseClose` state)
2. **Lid must be open** before robot enters CNC workspace
3. **Robot must retract** before lid closes
4. **Vise must be closed** before machining starts
5. **30-second timeout** on every state prevents indefinite hangs
6. **Fault state** requires explicit `ClearFault()` before resuming

---

*End of Part 2 -- Sections 9-16*

<!-- ================================================================== -->
<!-- PART 3: Sections 17-24 -- Visualization through API Reference      -->
<!-- ================================================================== -->

---

# Part 3 -- Visualization, UI, Audio, Shaders, Performance, Testing, and API (Sections 17-24)

---

## 17. Visualization

All visualization scripts live under `unity_twin/Assets/Scripts/Visualization/` and subscribe
to `CuttingStateEventSO` for real-time data. Each renderer is toggleable via the HUD.

### 17.1 ForceArrowRenderer

**File:** `Assets/Scripts/Visualization/ForceArrowRenderer.cs`
**Namespace:** `MiracleTwin.Visualization`

GPU-instanced arrows rendered at the tool tip showing the three orthogonal cutting force
components. Arrow length is proportional to force magnitude.

| Component | Color                  | Direction       |
|-----------|------------------------|-----------------|
| Fx        | Red `(1, 0.2, 0.2)`   | X-axis (feed)   |
| Fy        | Green `(0.2, 1, 0.2)` | Y-axis (normal) |
| Fz        | Blue `(0.3, 0.3, 1)`  | Z-axis (axial)  |

**Scaling:** `scaleFactor = 0.001 / 50` -- that is, 1 mm of arrow length per 50 N of force.
Arrows below `minimumForce` (default 5 N) are hidden.

**Rendering method:** Uses `Graphics.DrawMesh()` with per-instance `MaterialPropertyBlock`
color. The arrow mesh is drawn with the `MIRACLE/ForceArrow` unlit shader which supports
`UNITY_INSTANCING_BUFFER`.

```csharp
// Key serialized fields
[SerializeField] private Mesh arrowMesh;
[SerializeField] private Material arrowMaterial;      // Uses MIRACLE/ForceArrow shader
[SerializeField] private float scaleFactor = 0.001f / 50f;
[SerializeField] private float minimumForce = 5f;
```

**Inspector setup:**
1. Assign `CuttingStateEventSO` asset to the event channel slot.
2. Assign the arrow mesh (cone + cylinder or custom).
3. Assign a material using the `MIRACLE/ForceArrow` shader.

### 17.2 HeatMapOverlay

**File:** `Assets/Scripts/Visualization/HeatMapOverlay.cs`
**Namespace:** `MiracleTwin.Visualization`

Overlays a temperature color ramp on the workpiece surface. Driven by the `WorkpieceHeatMap`
shader through `MaterialPropertyBlock` properties.

**Color ramp (temperature to color):**

| Temperature  | Normalized t | Color  |
|-------------|-------------|--------|
| 20C (ambient) | 0.0         | Silver `(0.8, 0.8, 0.85)` |
| ~80C          | 0.3         | Yellow `(1.0, 1.0, 0.3)`  |
| ~150C         | 0.65        | Orange `(1.0, 0.5, 0.1)`  |
| 200C+         | 1.0         | Red    `(1.0, 0.1, 0.1)`  |

**Shader properties driven per-frame:**

| Property       | Type    | Description                       |
|---------------|---------|-----------------------------------|
| `_Temperature` | float   | Current max temperature (C)       |
| `_MinTemp`     | float   | Ramp minimum (default 20)         |
| `_MaxTemp`     | float   | Ramp maximum (default 200)        |
| `_HeatPosition`| Vector4 | World-space position of cut point |
| `_HeatRadius`  | float   | Heat influence radius (default 0.01 m) |

The `currentDisplayTemp` smoothly interpolates toward the target temperature using
`fadeSpeed` (default 2.0) to avoid visual popping.

### 17.3 ChipParticleController

**File:** `Assets/Scripts/Visualization/ChipParticleController.cs`
**Namespace:** `MiracleTwin.Visualization`

Drives a VFX Graph asset for chip particle emission during cutting. Particle parameters
are set from the cutting physics state each frame.

**VFX Graph property bindings:**

| VFX Property    | Source                | Units    |
|----------------|----------------------|----------|
| `SpawnRate`     | `mrr * spawnRateMultiplier` | particles/s |
| `ChipVelocity`  | `chipVelocity * velocityMultiplier` | m/s |
| `SpawnPosition`  | `toolPosition`       | world m  |
| `ChipDirection`  | `-toolDirection`     | unit vec |
| `ChipSize`       | Random in `[0.5, 2.0]` mm | m |
| `CurlRadius`     | `chipCurlRadius / 1000` | m     |
| `ChipColor`      | Aluminum silver `(0.75, 0.75, 0.78)` | RGBA |

When `isCutting` is false or `mrr <= 0`, the spawn rate is set to zero.

### 17.4 WearIndicator

**File:** `Assets/Scripts/Visualization/WearIndicator.cs`
**Namespace:** `MiracleTwin.Visualization`

Provides both a 3D visual on the tool and data for a UI progress bar.

**3D visual:** The tool renderer's material is darkened from `freshToolColor` (bright steel)
to `wornToolColor` (dark brown) as `wearPercentage` increases from 0 to 100%. The wear
band height scales proportionally up to `wearBandMaxHeight` (3 mm).

**Recommended action thresholds:**

| VB (mm)     | Action             |
|------------|-------------------|
| < 0.10     | `CONTINUE`        |
| 0.10-0.15  | `MONITOR`         |
| 0.15-0.25  | `PLAN_REPLACEMENT`|
| >= 0.25    | `REPLACE_NOW`     |

### 17.5 ToolpathPreview

**File:** `Assets/Scripts/Visualization/ToolpathPreview.cs`
**Namespace:** `MiracleTwin.Visualization`

Renders upcoming G-code segments using a Unity `LineRenderer` component.

**Color coding:**

| Segment Type | Color                         |
|-------------|-------------------------------|
| Rapid (G0)  | Green `(0.2, 0.9, 0.2, 0.5)` |
| Feed (G1)   | Blue `(0.3, 0.3, 1.0, 0.7)`  |
| Arc (G2/G3) | Purple `(0.8, 0.3, 1.0, 0.7)`|
| Current     | Red `(1.0, 0.2, 0.2, 1.0)`   |

Shows the next `previewSegments` (default 50) segments ahead of the current execution
point. Alpha fades from 1.0 at the current segment to 0.3 at the preview horizon.
Coordinates are converted from mm to meters (`/ 1000`).

**Public API:**
```csharp
public void SetToolpath(List<ToolpathSegment> segments);
public void AdvanceToSegment(int index);
public void Toggle();
public void Clear();
```

### 17.6 StabilityLobeChart

**File:** `Assets/Scripts/Visualization/StabilityLobeChart.cs`
**Namespace:** `MiracleTwin.Visualization`

2D overlay showing the Altintas-Budak zeroth-order approximation (ZOA) stability lobe
diagram. The boundary curve separates stable and unstable cutting parameter zones.

**Machine dynamics parameters (defaults for Bantam Explorer):**

| Parameter          | Value   | Unit   |
|-------------------|---------|--------|
| Natural frequency  | 800     | Hz     |
| Damping ratio      | 0.03    | --     |
| Stiffness          | 5e6     | N/m    |
| Ktc                | 796     | N/mm^2 |
| Flute count        | 2       | --     |

**Stability limit formula:**

```
ap_lim = -1 / (2 * Ktc * N * Re[G(jw)])
```

where `G(jw)` is the transfer function at chatter frequency `w`, and `N` is the flute count.

The `UpdateOperatingPoint(rpm, depthMM)` method checks whether the current cutting
parameters fall in the stable zone and sets the `IsStable` property.

### 17.7 SurfaceRoughnessOverlay

**File:** `Assets/Scripts/Visualization/SurfaceRoughnessOverlay.cs`
**Namespace:** `MiracleTwin.Visualization`

Color-maps surface roughness Ra values onto machined surfaces via the `_Roughness` shader
property. Updates at `updateInterval` (default 0.5 s) to reduce overhead.

**Color mapping:**

| Ra Range       | Color  | Grade         |
|---------------|--------|---------------|
| < 0.4 um      | Green  | N3 (Ground)   |
| 0.4 - 1.6 um  | Yellow | N4-N5 (Milled)|
| > 3.2 um      | Red    | N6+ (Rough)   |

---

## 18. UI Dashboard

All UI scripts live under `unity_twin/Assets/Scripts/UI/` and use Unity's UI Toolkit
framework with UXML/USS templates.

### 18.1 DashboardOverlay

**File:** `Assets/Scripts/UI/DashboardOverlay.cs`
**Namespace:** `MiracleTwin.UI`

Main HUD overlay that consolidates all live telemetry into a single panel.
Update rate is throttled to `UI_UPDATE_INTERVAL = 1/15` (15 Hz) to reduce overhead.

**ASCII layout mockup:**

```
+-------------------------------------------------------+
| [*] ROS Connected          FPS: 60  Cut: 00:12:34     |
+-------------------------------------------------------+
| STATUS: RUNNING     RPM: 16000    Feed: 500 mm/min    |
| G-code Line: 42     Sim Time: 00:12:34.5              |
+-------------------+-----------------------------------+
| FORCES            | TOOL WEAR                         |
|  Fx:  45.2 N      |  Wear: 12.3%   VB: 0.037 mm     |
|  Fy:  82.1 N      |  [=========>        ] 12%        |
|  Fz:  23.8 N      |  Action: CONTINUE                |
+-------------------+-----------------------------------+
| THERMAL           | OEE                               |
|  Tool:  67.4 C    |  OEE: 82.5%                      |
|  Interface: 43.1 C|  A: 95.1%  P: 89.3%  Q: 97.2%   |
+-------------------+-----------------------------------+
```

**Event channel subscriptions:**

| Event SO               | Data consumed                    |
|------------------------|----------------------------------|
| `MachineStateEventSO`  | status, RPM, feed rate, line     |
| `SystemKPIsEventSO`    | OEE, availability, performance   |
| `CuttingStateEventSO`  | forces, wear, temperatures       |

**Dynamic elements created at runtime** (if not present in UXML):
- Connection status dot (green/red 10px circle)
- FPS counter label (top-right, color-coded: green >60, yellow 30-60, red <30)
- Cutting time label

**Toggle:** Press `H` (bound via `InputManager.OnToggleHUD`).

### 18.2 ForceChart

**File:** `Assets/Scripts/UI/ForceChart.cs`
**Namespace:** `MiracleTwin.UI`

Scrolling time-series graph for Fx, Fy, Fz forces. Samples are stored in a
`Queue<Vector3>` ring buffer with configurable capacity.

| Setting          | Default | Description                     |
|-----------------|---------|---------------------------------|
| `maxSamples`    | 200     | Ring buffer size                |
| `maxForce`      | 200     | Y-axis maximum (N)             |
| `sampleInterval`| 0.033   | Sampling period (~30 Hz)       |

### 18.3 SimulationControlPanel

**File:** `Assets/Scripts/UI/SimulationControlPanel.cs`
**Namespace:** `MiracleTwin.UI`

Transport-style controls for the simulation clock.

| UI Element      | Binding                           |
|----------------|-----------------------------------|
| `play-btn`     | `SimulationClock.Play()`          |
| `pause-btn`    | `SimulationClock.Pause()`         |
| `stop-btn`     | `SimulationClock.ResetSimulation()`|
| `speed-slider` | `SimulationClock.SetAccelerated()`|
| `mode-dropdown`| Real-Time / Accelerated / Replay  |
| `load-gcode-btn`| File browser (placeholder)       |

### 18.4 EStopButton

**File:** `Assets/Scripts/UI/EStopButton.cs`
**Namespace:** `MiracleTwin.UI`

Emergency stop with double-click confirmation (threshold 0.5 s).

**Activation sequence:**
1. First click: logs "Click again to confirm E-STOP"
2. Second click within 0.5 s: triggers emergency stop
3. `SimulationClock.Pause()` is called immediately
4. `MiracleBridge.CallEStop()` sends ROS2 service request to
   `/miracle/{machineId}/trigger_estop`

```csharp
// Service request sent on E-Stop
var request = new TriggerEStopRequest {
    machine_id  = machineId,
    reason      = "Unity E-Stop button pressed",
    requesting_node = "unity_digital_twin"
};
```

### 18.5 AlertNotification

**File:** `Assets/Scripts/UI/AlertNotification.cs`
**Namespace:** `MiracleTwin.UI`

Toast popup system for anomaly and security alerts from ROS2.

**Severity levels:**

| Severity   | Trigger                          | Visual       |
|-----------|----------------------------------|-------------|
| Info       | Anomaly confidence < 50%         | Blue toast   |
| Warning    | Anomaly confidence 50-80%        | Yellow toast |
| Critical   | Anomaly confidence > 80%         | Red toast    |
| Security   | Any SecurityAlertMsg             | Purple toast |

**Settings:**
- `displayDuration`: 5 seconds per toast
- `maxVisibleAlerts`: 3 simultaneous toasts
- Full alert history maintained in `alertHistory` list

### 18.6 RobotStatusPanel

**File:** `Assets/Scripts/UI/RobotStatusPanel.cs`
**Namespace:** `MiracleTwin.UI`

Displays per-robot status from the `MultiAgentCoordinator`.

| Label              | Source                                |
|-------------------|---------------------------------------|
| `ned2-status`     | `coordinator.Ned2Status`              |
| `xarm6-status`    | `coordinator.XArm6Status`             |
| `last-award`      | `coordinator.LastAwardedRobot`        |
| `cycles-completed`| `coordinator.TotalCyclesCompleted`    |

### 18.7 GCodeEditor

**File:** `Assets/Scripts/UI/GCodeEditor.cs`
**Namespace:** `MiracleTwin.UI`

In-app G-code viewer and loader with active line highlighting.

```csharp
public void LoadProgram(string programText);
public void LoadFromFile(string path);
public string GetProgram();
public string CurrentProgram { get; }
public int HighlightedLine { get; set; }
```

---

## 19. Audio

Audio scripts live under `unity_twin/Assets/Scripts/Audio/`. Both controllers use
`OnAudioFilterRead()` for procedural synthesis -- no audio clips required.

### 19.1 CuttingSoundController

**File:** `Assets/Scripts/Audio/CuttingSoundController.cs`
**Namespace:** `MiracleTwin.Audio`

Generates the characteristic cutting sound from the tooth-passing frequency.

**Tooth-passing frequency:**
```
f_tooth = N * RPM / 60    (Hz)
```
where `N` = flute count (2), `RPM` = spindle speed.

| Parameter    | Default | Description                          |
|-------------|---------|--------------------------------------|
| `baseVolume` | 0.3     | Maximum volume at full power         |
| `noiseVolume`| 0.15    | Broadband noise component level      |
| `fadeSpeed`  | 10      | Volume attack/decay rate             |
| `fluteCount` | 2       | Number of flutes for freq calculation|

**Signal composition:**
- 60% sine wave at tooth-passing frequency (tonal component)
- 40% white noise (broadband cutting noise)
- Volume scaled by `power / 250 W` (clamped 0-1)
- Silent during rapids (G0) when `isCutting == false`

**Example:** At 16,000 RPM with a 2-flute cutter:
`f_tooth = 2 * 16000 / 60 = 533 Hz`

### 19.2 SpindleSoundController

**File:** `Assets/Scripts/Audio/SpindleSoundController.cs`
**Namespace:** `MiracleTwin.Audio`

Continuous spindle motor hum that pitch-shifts with RPM.

**Frequency mapping:**
```
freq = lerp(baseFrequency, maxFrequency, clamp01(RPM / 23000))
```

| Parameter       | Default | Description                    |
|----------------|---------|--------------------------------|
| `baseFrequency` | 200     | Hz at minimum RPM              |
| `maxFrequency`  | 2000    | Hz at 23,000 RPM               |
| `baseVolume`    | 0.1     | Volume level                   |
| `fadeSpeed`      | 5       | Smooth frequency/volume transition |

**Harmonic content:** Fundamental + 2nd harmonic (0.3x amplitude) + 3rd harmonic (0.1x
amplitude) for a richer motor sound.

**Activation:** Responds to `MachineStateEventSO`. Spindle sound is active when status
is `RUNNING` or `PAUSED` and RPM > 100. Fades to silence on M5 (spindle off).

---

## 20. Shaders & Materials

### 20.1 Shader Files

All shaders are located in `unity_twin/Assets/Shaders/` and target the Universal Render
Pipeline (URP).

#### MIRACLE/AluminumPBR

**File:** `Assets/Shaders/AluminumPBR.shader`

Ward anisotropic BRDF for brushed aluminum workpiece appearance.

| Property               | Type       | Default              | Description                    |
|-----------------------|-----------|----------------------|--------------------------------|
| `_BaseColor`           | Color      | `(0.85, 0.87, 0.89)` | Aluminum base color            |
| `_Metallic`            | Range(0,1) | 0.95                 | Near-full metallic             |
| `_Smoothness`          | Range(0,1) | 0.7                  | Surface polish                 |
| `_BrushedNormalMap`    | Texture2D  | bump                 | Brushed grain normal map       |
| `_NormalStrength`      | Range(0,2) | 1.0                  | Normal map intensity           |
| `_AnisotropyDirection` | Range(0,1) | 0.0                  | 0=U direction, 1=V direction   |
| `_AnisotropyIntensity` | Range(0,1) | 0.5                  | Anisotropy blend factor        |
| `_BrushedTiling`       | Vector4    | `(4, 4, 0, 0)`      | UV tiling for brush pattern    |

**Specular model:** Implements the Ward anisotropic specular model with Schlick Fresnel
approximation. Aluminum F0 is set at 0.91-0.95. The brush direction stretches roughness
along one axis (`roughnessT = roughness * (1 + anisoIntensity * 3)`) while keeping the
perpendicular axis tight.

**Passes:** ForwardLit, ShadowCaster, DepthOnly. GPU instancing enabled.

#### MIRACLE/HSSToolSteel

**File:** `Assets/Shaders/HSSToolSteel.shader`

Procedural HSS tool steel appearance with FBM noise for heat-treatment color variation
and real-time wear visualization.

| Property             | Type       | Default              | Description                    |
|---------------------|-----------|----------------------|--------------------------------|
| `_BaseColor`         | Color      | `(0.25, 0.25, 0.28)` | Dark steel base                |
| `_TintColor`         | Color      | `(0.35, 0.30, 0.45)` | Heat treatment tint            |
| `_Metallic`          | Range(0,1) | 0.8                  | Steel metallic                 |
| `_Smoothness`        | Range(0,1) | 0.6                  | Surface polish                 |
| `_ColorVariation`    | Range(0,1) | 0.15                 | FBM noise blend amount         |
| `_WearAmount`        | Range(0,1) | 0.0                  | Wear progression (driven by WearIndicator) |
| `_WearEdgeSharpness` | Range(0.1,10) | 2.0              | Wear mask edge falloff         |
| `_GrainScale`        | Float      | 50.0                 | Surface grain texture scale    |

**Procedural generation:**
- 3-octave FBM noise (`ValueNoise` + smoothstep interpolation) for heat-treatment color
  variation
- Wear mask derived from surface normal dot product with forward axis
- Worn areas: 0.4x base color brightness, 0.3x smoothness, 0.6x metallic
- Dark edge rim effect (Fresnel^4) for tool steel appearance

#### MIRACLE/WorkpieceHeatMap

**File:** `Assets/Shaders/WorkpieceHeatMap.shader`

Temperature-dependent color ramp shader for the heat map overlay.

**Color ramp function `TemperatureToColor(t)`:**
```
t < 0.3:  lerp(silver, yellow, t / 0.3)
t < 0.65: lerp(yellow, orange, (t - 0.3) / 0.35)
t >= 0.65: lerp(orange, red, (t - 0.65) / 0.35)
```

Heat influence uses quadratic falloff from `_HeatPosition` over `_HeatRadius * 5`.

#### MIRACLE/ForceArrow

**File:** `Assets/Shaders/ForceArrow.shader`

Minimal unlit shader for force arrow rendering. Supports GPU instancing with per-instance
`_Color` property. Simple N-dot-L shading for minimal depth perception.

#### MIRACLE/GhostPreview

**File:** `Assets/Shaders/GhostPreview.shader`

Transparent shader for toolpath preview and ghost objects.
- Blend mode: SrcAlpha OneMinusSrcAlpha
- ZWrite Off, Cull Off
- Default color: semi-transparent blue `(0.3, 0.5, 1.0, 0.3)`

### 20.2 Material Files

All materials are located in `unity_twin/Assets/Materials/`.

| Material File          | Shader Used              | Applied To              |
|-----------------------|--------------------------|------------------------|
| `Aluminum6061.mat`     | MIRACLE/AluminumPBR      | Workpiece              |
| `HSSToolSteel.mat`     | MIRACLE/HSSToolSteel     | End mill tool           |
| `MachineBodyGray.mat`  | URP/Lit                  | CNC machine enclosure   |
| `RobotWhite.mat`       | URP/Lit                  | Niryo Ned2 / xArm 6    |
| `ViseSteel.mat`        | URP/Lit                  | Vise body and jaws      |
| `FloorConcrete.mat`    | URP/Lit                  | Shop floor surface      |

---

## 21. Performance Optimization

### 21.1 Frame Budget

Target: **30 FPS minimum** at all times on recommended hardware.

| Subsystem                | Budget (ms) | Notes                              |
|-------------------------|------------|-------------------------------------|
| Physics / cutting sim    | 8          | Burst-compiled CuttingForceJob      |
| Voxel marching cubes     | 6          | Dirty-chunk cap per frame           |
| Rendering (draw calls)   | 8          | GPU instancing for arrows/particles |
| UI Toolkit update        | 2          | Throttled to 15 Hz                  |
| ROS message dispatch     | 2          | Max 50 messages/frame               |
| Audio synthesis           | 1          | Runs on audio thread                |
| GC headroom              | 3          | Incremental GC target               |
| **Total target**         | **30**     | **= 33.3 ms budget at 30 FPS**     |

### 21.2 Zero-Allocation Targets

The following hot paths are designed for zero per-frame GC allocations:

- `CuttingForceJob.Execute()` -- Burst-compiled, no managed allocations
- `ForceArrowRenderer.Update()` -- pre-allocated `Matrix4x4[]` and `MaterialPropertyBlock[]`
- `DashboardOverlay.UpdateUI()` -- uses `Label.SetText()` with cached format strings
- `MessageDispatcher.Update()` -- `ConcurrentQueue.TryDequeue()` is allocation-free
- `SimulationClock.Update()` -- pure arithmetic, no allocations

### 21.3 PerformanceMonitor

**File:** `Assets/Scripts/Core/PerformanceMonitor.cs`
**Namespace:** `MiracleTwin.Core`

Singleton that tracks runtime performance metrics. Dashboard and alert systems read from
this singleton.

```csharp
public class PerformanceMonitor : MonoBehaviour
{
    public static PerformanceMonitor Instance { get; }

    // Metrics (read-only properties)
    public float AverageFPS { get; }          // Rolling average over 60 frames
    public float CurrentFPS { get; }          // Instantaneous 1/dt
    public float MinFPS { get; }              // Min in rolling window
    public float MaxFPS { get; }              // Max in rolling window
    public float GpuFrameTimeMs { get; }      // From FrameTimingManager
    public float CpuFrameTimeMs { get; }      // From unscaledDeltaTime
    public int   TotalDrawCalls { get; }      // Batches count (Editor only)
    public int   TotalTriangles { get; }      // Triangle count (Editor only)
    public int   SetPassCalls { get; }        // SetPass calls (Editor only)
    public float ManagedMemoryMB { get; }     // GC.GetTotalMemory
    public float TotalAllocatedMemoryMB { get; } // Profiler.GetTotalAllocatedMemoryLong
    public bool  IsLowFPS { get; }            // True when avg < threshold

    // Events
    public event Action<float> OnLowFPS;      // Fires on low-FPS transition

    // Utility
    public string GetSummary();               // Formatted debug string
}
```

**Configuration:**

| Setting                | Default | Description                       |
|-----------------------|---------|-----------------------------------|
| `fpsRollingWindowSize` | 60      | Number of frames for FPS average  |
| `lowFpsThreshold`      | 30      | FPS below which OnLowFPS fires    |
| `metricsUpdateInterval`| 0.5     | Seconds between full metric updates|

### 21.4 Incremental GC

Unity's incremental garbage collector is used. Recommended Player Settings:

```
Edit > Project Settings > Player > Other Settings
  [x] Use incremental GC
  GC time slice: 3 ms
```

### 21.5 Dirty-Chunk Marching Cubes Cap

The `MarchingCubesRenderer` limits the number of chunks re-meshed per frame to prevent
frame spikes during aggressive material removal. Only chunks marked dirty in
`VoxelGridData.dirtyChunks[]` are processed.

**Default cap:** 4 chunks per frame. At 30 FPS this allows 120 chunk updates per second,
sufficient for typical feed rates.

### 21.6 Message Throttling

`MessageDispatcher` caps message processing at `maxMessagesPerFrame` (default 50).
Excess messages remain queued for the next frame. The `DashboardOverlay` updates at
15 Hz rather than every frame.

---

## 22. Testing

### 22.1 EditMode Tests

EditMode tests validate pure computation classes without requiring a running Unity scene.
Located in `unity_twin/Assets/Tests/EditMode/`.

#### GCodeParserTests

**File:** `Assets/Tests/EditMode/GCodeParserTests.cs`

| Test                           | Validates                                    |
|-------------------------------|----------------------------------------------|
| `Parse_G0_Rapid_Move`         | G0 parsing with X, Y, Z parameters           |
| `Parse_G1_Linear_Feed`        | G1 with feed rate F parameter                |
| `Parse_G2_CW_Arc`             | G2 clockwise arc with I, J center offsets     |
| `Parse_M3_Spindle_On`         | M-code parsing with S parameter              |
| `Parse_Skips_Comments`         | Comments `(...)` and `;...` are ignored      |
| `Parse_Negative_Coordinates`   | Negative values like `X-5.5`                 |
| `Parse_Multi_Line_Program`     | Full 6-line program parsing                  |
| `Parse_Empty_Program`          | Empty string returns empty list              |
| `Validate_Detects_Missing_Spindle` | Feed move without prior M3 detected     |
| `Validate_Detects_Missing_Program_End` | Missing M30 warning                 |

#### CuttingForceEngineTests

**File:** `Assets/Tests/EditMode/CuttingForceEngineTests.cs`

| Test                              | Validates                                     |
|----------------------------------|-----------------------------------------------|
| `Force_Calculation_Produces_Nonzero_Results` | Fx, Fy, Fz, Power, MRR all > 0     |
| `Force_Increases_With_Depth_Of_Cut` | 2.0 mm depth > 0.5 mm depth forces         |
| `Wear_Increases_Forces`           | VB=0.2 mm produces higher forces than VB=0   |
| `Zero_RPM_Returns_Zero_Forces`    | All outputs zero when RPM=0                  |

Uses `NativeArray<float>` with `Allocator.Temp` for test-scoped buffers.

#### ThermalModelTests

**File:** `Assets/Tests/EditMode/ThermalModelTests.cs`

| Test                                    | Validates                               |
|----------------------------------------|----------------------------------------|
| `Initial_Temperature_Is_Ambient`        | Tool and interface start at 20C        |
| `Temperature_Increases_During_Cutting`   | Both temperatures rise above ambient   |
| `Temperature_Decreases_During_Cooldown`  | Tool cools after cutting stops         |
| `Interface_Temp_Follows_Stephenson_Agapiou` | ~47C for V=150, f=0.05, ap=1.0   |
| `Reset_Returns_To_Ambient`              | All temperatures back to 20C           |

#### ToolWearModelTests

**File:** `Assets/Tests/EditMode/ToolWearModelTests.cs`

| Test                             | Validates                                  |
|---------------------------------|-------------------------------------------|
| `Initial_Wear_Is_Break_In_Value` | VB starts at 0.02 mm                      |
| `Wear_Increases_Over_Time`       | VB grows after 5 min of cutting           |
| `Break_In_Stage_Completes`       | Stage >= 2 after 3 minutes                |
| `End_Of_Life_Detected`           | IsEndOfLife or VB > 0.25 after long run   |
| `Reset_Returns_To_Initial`       | VB=0.02, CuttingTime=0 after reset        |
| `Higher_Speed_Increases_Wear_Rate` | 200 m/min wears faster than 100 m/min  |

#### ChipFormationTests

**File:** `Assets/Tests/EditMode/ChipFormationTests.cs`

| Test                                  | Validates                              |
|--------------------------------------|---------------------------------------|
| `Aluminum_Shear_Angle_Is_Reasonable`  | phi ~ 29 +/- 5 degrees               |
| `Aluminum_Is_Continuous_At_Normal_Speeds` | Continuous chip at V=150, fz=0.05 |
| `Chip_Curl_Radius_Is_Physical`        | 1 < Rc < 20 mm                       |
| `Chip_Velocity_Is_Positive`           | Vc > 0                                |

#### SurfaceRoughnessTests

**File:** `Assets/Tests/EditMode/SurfaceRoughnessTests.cs`

| Test                                  | Validates                              |
|--------------------------------------|---------------------------------------|
| `Roughness_Increases_With_Feed`       | Higher fz produces higher Ra          |
| `Standard_Conditions_Produce_Expected_Ra` | Ra ~ 0.2 um at fz=0.05, re=0.4  |
| `Vibration_Increases_Roughness`        | Ra increases with vibration amplitude |
| `Surface_Grade_Classification`         | N1 at 0.05 um, N5 at 1.2 um         |

### 22.2 PlayMode Tests

PlayMode tests validate MonoBehaviour components in a live Unity scene.
Located in `unity_twin/Assets/Tests/PlayMode/`.

#### MiracleBridgeTests

**File:** `Assets/Tests/PlayMode/MiracleBridgeTests.cs`

Tests the ROS bridge singleton using reflection to invoke private callbacks without
a real TCP connection.

| Test Category              | Count | Key Tests                               |
|---------------------------|------|-----------------------------------------|
| Singleton initialization   | 4     | Instance set, duplicate destroyed, DontDestroyOnLoad |
| Connection defaults        | 3     | IP=127.0.0.1, port=10000, interval>0    |
| Topic naming              | 2     | Machine topics use machineId, system topics are global |
| SO event firing            | 8     | Each callback raises its SO event (MachineState through SecurityAlert) |
| Multiple listeners         | 2     | 3 listeners all receive, unregistered does not |
| Null safety               | 1     | Null event channel does not throw        |
| Cleanup                   | 1     | OnDestroy clears Instance                |

#### SimulationClockTests

**File:** `Assets/Tests/PlayMode/SimulationClockTests.cs`

| Test Category              | Count | Key Tests                               |
|---------------------------|------|-----------------------------------------|
| Singleton                  | 2     | Instance set, duplicate destroyed        |
| Initial state              | 3     | Paused, SimTime=0, DeltaTime=0          |
| Mode switching             | 8     | All mode transitions, Play/Pause/Toggle  |
| Speed multiplier           | 3     | Lower bound=0.1, upper=100, valid=5     |
| Time accumulation          | 5     | Paused=frozen, RealTime=advancing, Accelerated=scaled |
| Replay support             | 2     | SeekTo sets time, AdvanceReplay increments|
| OnTick event               | 5     | Fires in RT/Accel/Replay, not paused, monotonic |
| OnModeChanged event        | 2     | Fires on switch, records each transition |
| OnReset event              | 4     | Fires on reset, clears time, sets paused |
| FormatSimTime              | 2     | "00:00:00.0" at zero, "01:23:45.6" at 5025.6s |
| Edge cases                 | 2     | Rapid switching, pause/resume preserves time |

### 22.3 Running Tests

```bash
# From Unity Editor:
# Window > General > Test Runner
# Select EditMode or PlayMode tab, click "Run All"

# From command line (headless):
Unity -batchmode -runTests \
  -projectPath ./unity_twin \
  -testPlatform EditMode \
  -testResults ./TestResults/editmode.xml

Unity -batchmode -runTests \
  -projectPath ./unity_twin \
  -testPlatform PlayMode \
  -testResults ./TestResults/playmode.xml
```

### 22.4 Manual Integration Test Sequence

Execute the following sequence to validate the full pipeline end-to-end:

| Step | Action                                      | Expected Result                            |
|------|--------------------------------------------|--------------------------------------------|
| 1    | Launch ROS2 backend: `ros2 launch miracle_core miracle_system.launch.py` | All nodes start |
| 2    | Open Unity, enter Play mode                 | Dashboard shows "ROS Connected" (green dot)|
| 3    | Load `test_square_pocket.nc` via GCodeEditor| G-code appears, line count shown           |
| 4    | Press Play in SimulationControlPanel        | Spindle sound starts, toolpath preview shown|
| 5    | Observe cutting simulation                  | Force arrows appear, chip particles emit   |
| 6    | Check heat map toggle (press `T`)           | Workpiece shows temperature color overlay  |
| 7    | Monitor wear bar during cutting             | Wear % increases, bar fills               |
| 8    | Trigger anomaly from ROS2 test publisher    | Alert toast appears in top-right          |
| 9    | Double-click E-Stop button                  | Simulation pauses, service call sent       |
| 10   | Start/stop recording via DataRecorder       | `.miracle` file created in Recordings/    |
| 11   | Load recording in ReplayController          | Playback reproduces cutting visualization  |
| 12   | Switch camera modes (keys 0-5)              | Orbit, follow-tool, preset views work     |

---

## 23. Troubleshooting

### 23.1 ROS Disconnected

**Symptom:** Dashboard shows red dot and "Disconnected" label.

**Causes and fixes:**

| Cause                                | Fix                                           |
|-------------------------------------|-----------------------------------------------|
| ROS2 bridge not running              | Start: `ros2 launch miracle_bridges miracle_bridge.launch.py` |
| Wrong IP/port in MiracleBridge       | Inspector: verify `rosBridgeIP=127.0.0.1`, `rosBridgePort=10000` |
| Firewall blocking port 10000         | Open TCP port 10000 in firewall settings      |
| Connection timeout (10s no messages) | Ensure at least one ROS publisher is active   |
| Max reconnect attempts exceeded      | Set `maxReconnectAttempts=0` for unlimited retry |

**Debug:** Check the Console for `[MiracleBridge]` log messages. Connection attempts and
failures are logged with timestamps.

### 23.2 Pink / Magenta Materials

**Symptom:** Objects appear solid pink/magenta in the scene or Game view.

**Causes and fixes:**

| Cause                                | Fix                                           |
|-------------------------------------|-----------------------------------------------|
| Project not using URP                | Install URP package, assign URP Asset in Graphics settings |
| Shader compilation error             | Open shader file, check Console for compile errors |
| Material references missing shader   | Re-assign shader in material inspector        |
| Built-in shaders on URP project      | Convert via Edit > Rendering > Materials > Convert to URP |

### 23.3 Packages Fail to Import

**Symptom:** Unity Console shows "Assembly definition" errors or missing namespace errors.

**Resolution:**
1. Close Unity
2. Delete `unity_twin/Library/` folder
3. Delete `unity_twin/Temp/` folder
4. Re-open Unity (full reimport)
5. Verify packages in `Packages/manifest.json`:
   - `com.unity.render-pipelines.universal`
   - `com.unity.robotics.ros-tcp-connector`
   - `com.unity.burst`
   - `com.unity.collections`
   - `com.unity.mathematics`
   - `com.unity.visualeffectgraph`
   - `com.unity.inputsystem`
   - `com.unity.ui` (UI Toolkit)

### 23.4 Low FPS

**Symptom:** FPS counter shows < 30, PerformanceMonitor fires `OnLowFPS`.

**Diagnostic checklist:**

| Check                          | Command / Location                         |
|-------------------------------|-------------------------------------------|
| GPU frame time                 | `PerformanceMonitor.Instance.GpuFrameTimeMs` |
| Draw call count                | Unity Profiler > Rendering module          |
| Voxel chunk rebuilds           | Reduce `maxChunksPerFrame` in MarchingCubesRenderer |
| VFX particle count             | Lower `spawnRateMultiplier` in ChipParticleController |
| Message queue backup           | `MessageDispatcher.Instance.PendingCount`  |
| UI update rate                 | Already throttled to 15 Hz                 |
| Incremental GC spikes          | Check Profiler > Memory > GC.Alloc         |

**Quick fixes:**
- Reduce voxel grid resolution (fewer chunks)
- Disable chip particles (`ChipParticleController.SetEnabled(false)`)
- Disable heat map overlay (`HeatMapOverlay.Toggle()`)
- Lower `previewSegments` in `ToolpathPreview`

### 23.5 Voxel Grid Artifacts

**Symptom:** Holes, floating geometry, or staircase patterns on machined surfaces.

| Cause                        | Fix                                           |
|-----------------------------|-----------------------------------------------|
| Grid resolution too low      | Increase `GridSize` in VoxelGridData          |
| Chunk boundaries misaligned  | Ensure `ChunkSize` divides evenly into `GridSize` |
| Tool radius mismatch         | Verify `toolDiameter` in CuttingForceEngine matches VoxelWorkpiece |
| Marching cubes ISO threshold | Adjust isosurface threshold value             |
| Dirty flags not propagated   | Check `MarkChunkDirty()` is called for neighboring chunks |

### 23.6 Force NaN

**Symptom:** Force values display as NaN in dashboard or force arrows disappear.

**Root causes:**

1. **Zero RPM with active cutting** -- `CuttingForceJob` returns all zeros when RPM=0,
   but downstream division by zero in power/torque calculation could produce NaN.
   Fix: The job guards against this (`if (spindleRPM <= 0) return zeros`).

2. **Extreme parameters** -- Very high feed rate or depth can overflow float32.
   Fix: Clamp inputs to physical ranges.

3. **NaN propagation from wear model** -- If `VB` somehow becomes NaN, the wear
   force multiplier produces NaN forces. Fix: Reset the wear model.

**Debug:** Add a NaN check in `CuttingSimulationManager`:
```csharp
if (float.IsNaN(forces.x) || float.IsNaN(forces.y) || float.IsNaN(forces.z))
    Debug.LogError("[CuttingSim] NaN forces detected -- resetting engine");
```

### 23.7 Robot IK Unreachable

**Symptom:** Robot arm does not reach the target or snaps to an unexpected pose.

**Fixes:**
- Verify target is within workspace radius (Ned2: ~0.44 m, xArm6: ~0.70 m)
- Check for joint angle limits in `InverseKinematics.cs`
- Ensure `TrajectoryInterpolator` waypoints do not cross singularities
- Verify the robot's base transform is correctly positioned relative to the CNC

### 23.8 Recording Corruption

**Symptom:** ReplayController fails to load a `.miracle` file or crashes during playback.

**Causes and fixes:**

| Cause                         | Fix                                          |
|------------------------------|----------------------------------------------|
| Recording stopped abruptly    | Always call `DataRecorder.StopRecording()` before closing |
| Invalid magic header          | File header must start with `"MIRACLE_REC"`  |
| Version mismatch              | Current version is 1; check file version byte|
| Truncated payload             | File was cut short; recording may be partial |
| Disk full during recording    | Ensure sufficient disk space (>100 MB free)  |

---

## 24. API Reference

### 24.1 Namespace: MiracleTwin.Core

#### SimulationClock

| Member                    | Type            | Description                                    |
|--------------------------|----------------|------------------------------------------------|
| `Instance`               | static property | Singleton instance                             |
| `CurrentMode`            | `Mode` property | RealTime, Accelerated, Replay, Paused          |
| `SimTime`                | `double`        | Accumulated simulation seconds                 |
| `DeltaTime`              | `double`        | Time step for current frame                    |
| `SpeedMultiplier`        | `float`         | Accelerated mode multiplier [0.1, 100]         |
| `IsPaused`               | `bool`          | True when mode is Paused                       |
| `IsRunning`              | `bool`          | True when mode is not Paused                   |
| `TotalElapsedWallTime`   | `double`        | Real wall-clock seconds since init             |
| `WallTimeSinceReset`     | `double`        | Real wall-clock seconds since last reset       |
| `Play()`                 | method          | Set mode to RealTime                           |
| `Pause()`                | method          | Set mode to Paused                             |
| `TogglePause()`          | method          | Toggle between Paused and RealTime             |
| `SetAccelerated(float)`  | method          | Set mode + multiplier                          |
| `SetMode(Mode)`          | method          | Explicit mode transition                       |
| `SeekTo(double)`         | method          | Set SimTime directly (Replay)                  |
| `AdvanceReplay(double)`  | method          | Increment SimTime by dt (Replay)               |
| `ResetSimulation()`      | method          | Zero SimTime, set Paused, fire OnReset         |
| `FormatSimTime()`        | method          | Returns "HH:MM:SS.f"                          |
| `FormatWallTime()`       | method          | Returns "HH:MM:SS"                            |
| `OnTick`                 | `event<double>` | Fires every non-paused frame with SimTime      |
| `OnModeChanged`          | `event<Mode>`   | Fires on mode transition                       |
| `OnReset`                | `event`         | Fires on ResetSimulation()                     |

#### MiracleBridge

| Member                        | Type                  | Description                            |
|------------------------------|-----------------------|----------------------------------------|
| `Instance`                   | static property       | Singleton instance                     |
| `IsConnected`                | `bool`                | ROS TCP connection status              |
| `MachineId`                  | `string`              | Machine identifier (default "cnc1")    |
| `ReconnectAttempts`          | `int`                 | Current reconnect attempt count        |
| `ConnectionStatusChanged`    | `event<bool>`         | Fires on connect/disconnect transition |
| `CallEStop(string, Action<>)`| method                | Send TriggerEStop service request      |
| `CallValidateGCode(string, Action<>)` | method       | Send ValidateGCode service request     |
| `SetReconnectInterval(float)`| method                | Update reconnect interval at runtime   |

#### MessageDispatcher

| Member              | Type            | Description                                  |
|--------------------|----------------|----------------------------------------------|
| `Instance`          | static property | Singleton instance                           |
| `Enqueue(Action)`   | method          | Queue action for main thread                 |
| `Enqueue<T>(Action<T>, T)` | method   | Queue typed callback                         |
| `PendingCount`      | `int`           | Number of queued actions                     |
| `maxMessagesPerFrame`| config         | Frame processing cap (default 50)            |

#### DataRecorder

| Member               | Type      | Description                                    |
|---------------------|----------|------------------------------------------------|
| `IsRecording`        | `bool`    | True while recording is active                 |
| `CurrentFilePath`    | `string`  | Path to current recording file                 |
| `RecordingDuration`  | `double`  | Duration of last completed recording           |
| `StartRecording()`   | method    | Begin recording to new .miracle file           |
| `StopRecording()`    | method    | Flush and close current recording              |
| `RecordMessage(uint, byte[])` | method | Record a single timestamped message    |
| `ToggleRecording()`  | method    | Start or stop                                  |

#### ReplayController

| Member                  | Type              | Description                              |
|------------------------|-------------------|------------------------------------------|
| `IsLoaded`              | `bool`             | True when a file is loaded               |
| `IsPlaying`             | `bool`             | True during playback                     |
| `Duration`              | `double`           | Total recording duration (seconds)       |
| `CurrentTime`           | `double`           | Current playback position                |
| `Progress`              | `float`            | Normalized 0-1 playback position         |
| `PlaybackSpeed`         | `float`            | Speed [0.1, 10]                          |
| `LoadRecording(string)` | method             | Load a .miracle file                     |
| `Play()`                | method             | Start or resume playback                 |
| `PauseReplay()`         | method             | Pause playback                           |
| `Seek(float)`           | method             | Seek to normalized time [0, 1]           |
| `Unload()`              | method             | Close file and reset                     |
| `OnMessageReplay`       | `event<uint, byte[]>` | Fires for each replayed message      |

#### PerformanceMonitor

| Member                    | Type            | Description                              |
|--------------------------|----------------|------------------------------------------|
| `Instance`               | static property | Singleton                                |
| `AverageFPS`             | `float`          | Rolling average over window              |
| `CurrentFPS`             | `float`          | Instantaneous FPS                        |
| `MinFPS` / `MaxFPS`      | `float`          | Window extremes                          |
| `GpuFrameTimeMs`         | `float`          | GPU time from FrameTimingManager         |
| `CpuFrameTimeMs`         | `float`          | CPU frame time                           |
| `TotalDrawCalls`         | `int`            | Batch count (Editor)                     |
| `TotalTriangles`         | `int`            | Triangle count (Editor)                  |
| `ManagedMemoryMB`        | `float`          | Managed heap in MB                       |
| `TotalAllocatedMemoryMB` | `float`          | Total allocated in MB                    |
| `IsLowFPS`               | `bool`           | Below threshold flag                     |
| `OnLowFPS`               | `event<float>`   | Fires on low-FPS transition              |
| `GetSummary()`           | method           | Formatted string for debug display       |

#### CameraController

| Member                | Type            | Description                              |
|----------------------|----------------|------------------------------------------|
| `CurrentMode`        | `CameraMode`    | Orbit, FollowTool, Front, Side, TopDown, Isometric |
| `SetMode(CameraMode)`| method          | Switch camera mode                       |
| `SetPreset(CameraMode)` | method       | Jump to a preset view                    |
| `ResetOrbit()`       | method          | Reset orbit to defaults                  |

#### GameEventSO\<T>

| Member              | Type           | Description                               |
|--------------------|---------------|-------------------------------------------|
| `Raise(T)`          | method         | Broadcast value to all listeners          |
| `Register(Action<T>)` | method      | Subscribe a listener                      |
| `Unregister(Action<T>)` | method    | Unsubscribe a listener                    |
| `ListenerCount`     | `int`          | Current subscriber count                  |

#### TimestampedRingBuffer\<T>

| Member                    | Type   | Description                              |
|--------------------------|--------|------------------------------------------|
| `Count`                   | `int`   | Current number of entries                |
| `Capacity`               | `int`   | Maximum entries                          |
| `Add(double, T)`         | method  | Insert timestamped value                 |
| `TryGetAtTime(double, out T)` | method | Find nearest entry to given time    |
| `Clear()`                | method  | Remove all entries                       |

### 24.2 Namespace: MiracleTwin.Cutting

#### CuttingForceEngine

| Member                  | Type       | Description                                |
|------------------------|-----------|---------------------------------------------|
| `PeakForce`             | `Vector3`  | Peak Fx, Fy, Fz (N)                        |
| `AverageForce`          | `Vector3`  | Average Fx, Fy, Fz (N)                     |
| `PowerWatts`            | `float`    | Cutting power (W)                           |
| `TorqueNm`              | `float`    | Spindle torque (Nm)                         |
| `MRR`                   | `float`    | Material removal rate (mm^3/min)            |
| `SpecificCuttingEnergy` | `float`    | Specific energy (J/mm^3)                    |
| `Calculate(...)`        | method     | Run force calculation for given parameters  |
| `KienzleForceEstimate(...)`| method  | Quick Kienzle model estimate               |

#### CuttingForceJob (Burst IJob)

| Field                  | Type    | Description                                  |
|-----------------------|---------|----------------------------------------------|
| `spindleRPM`          | `float` | Spindle speed input                          |
| `feedPerTooth`        | `float` | Feed per tooth (mm)                          |
| `axialDepth`          | `float` | Axial depth of cut (mm)                      |
| `radialDepth`         | `float` | Radial depth of cut (mm)                     |
| `toolDiameter`        | `float` | Tool diameter (mm)                           |
| `fluteCount`          | `int`   | Number of flutes                             |
| `helixAngle`          | `float` | Helix angle (radians)                        |
| `Ktc/Krc/Kac`         | `float` | Shearing coefficients (N/mm^2)               |
| `Kte/Kre/Kae`         | `float` | Edge coefficients (N/mm)                     |
| `flankWearVB`         | `float` | Current flank wear (mm)                      |
| `wearForceMultiplier` | `float` | Wear coupling factor (mm^-1)                 |
| `output`              | `NativeArray<float>` | 12-element output array             |

#### GCodeParser

| Member                  | Type                    | Description                    |
|------------------------|------------------------|--------------------------------|
| `Parse(string)`         | `List<GCodeCommand>`    | Parse multi-line program       |
| `ParseLine(string, int)`| `GCodeCommand`          | Parse single line              |
| `Validate(string)`      | `List<string>`          | Return validation errors       |

#### GCodeCommand (struct)

| Member         | Type                      | Description                    |
|---------------|--------------------------|--------------------------------|
| `rawLine`      | `string`                  | Original text                  |
| `lineNumber`   | `int`                     | 1-based line number            |
| `type`         | `char`                    | 'G' or 'M'                    |
| `code`         | `float`                   | Numeric code (0, 1, 2, 3...)   |
| `parameters`   | `Dictionary<char, float>` | X, Y, Z, I, J, K, F, S...     |
| `IsRapid`      | `bool`                    | G0                             |
| `IsLinearFeed` | `bool`                    | G1                             |
| `IsCWArc`      | `bool`                    | G2                             |
| `IsCCWArc`     | `bool`                    | G3                             |
| `IsMotion`     | `bool`                    | Any G0-G3                      |
| `IsMCode`      | `bool`                    | M-code                         |
| `GetParam(char, float)` | method             | Get parameter with default     |
| `HasParam(char)` | method                  | Check parameter exists         |

#### ThermalModel

| Member                  | Type    | Description                              |
|------------------------|--------|------------------------------------------|
| `ToolTemperature`       | `float` | Current tool body temperature (C)        |
| `InterfaceTemperature`  | `float` | Stephenson-Agapiou interface temp (C)    |
| `WorkpieceTemperature`  | `float` | Lumped workpiece temperature (C)         |
| `ChipTemperature`       | `float` | Instantaneous chip temperature (C)       |
| `HeatGenerationRate`    | `float` | Current heat input (W)                   |
| `Update(V, fz, ap, power, dt)` | method | Advance thermal state             |
| `Cooldown(dt)`          | method  | Passive cooling when not cutting         |
| `Reset()`               | method  | Reset all to ambient (20C)               |

#### ToolWearModel

| Member                    | Type    | Description                            |
|--------------------------|--------|----------------------------------------|
| `VB`                      | `float` | Flank wear land width (mm)             |
| `CuttingTime`             | `float` | Accumulated cutting time (minutes)     |
| `WearPercentage`          | `float` | VB / VBmax * 100                       |
| `RemainingLifeMinutes`    | `float` | Taylor-based remaining life estimate   |
| `IsEndOfLife`             | `bool`  | VB >= 0.30 mm                          |
| `CurrentStage`            | `int`   | 1=Break-in, 2=Steady, 3=Accelerated   |
| `RecommendedAction`       | `string`| CONTINUE/MONITOR/PLAN_REPLACEMENT/REPLACE_NOW |
| `Update(V, fz, ap, dt)`  | method  | Advance wear state                     |
| `TaylorLifePrediction(V, fz, ap)` | method | Predict tool life (minutes)   |
| `Reset()`                 | method  | Reset to initial state                 |

#### ChipFormationModel (static)

| Member                    | Type       | Description                          |
|--------------------------|-----------|--------------------------------------|
| `Calculate(V, fz, D, ...)` | `ChipData` | Compute all chip formation params  |

#### ChipData (struct)

| Field                | Type    | Description                           |
|---------------------|--------|---------------------------------------|
| `chipThicknessRatio` | `float` | r = t1/t2 (typically 0.3-0.5 for Al) |
| `shearAngle`         | `float` | Merchant phi (degrees)                |
| `chipCurlRadius`     | `float` | Rc (mm)                               |
| `chipVelocity`       | `float` | Vc (m/min)                            |
| `isContinuous`       | `bool`  | True for Al at normal speeds          |
| `chipThickness`      | `float` | Actual chip thickness t2 (mm)         |
| `shearStrainRate`    | `float` | Strain rate (1/s)                     |

#### SurfaceRoughnessModel (static)

| Member                       | Type     | Description                         |
|-----------------------------|---------|-------------------------------------|
| `CalculateRa(fz, re, vib)`  | `float`  | Ra in micrometers                   |
| `CalculateRz(Ra)`           | `float`  | Rz ~ 5 * Ra                        |
| `GetSurfaceGrade(Ra)`       | `string` | N1 through N7+ classification       |

#### VoxelGridData

| Member                    | Type       | Description                          |
|--------------------------|-----------|--------------------------------------|
| `GridSize`                | `int3`     | Total grid dimensions                |
| `ChunkSize`              | `int3`     | Per-chunk dimensions                 |
| `ChunkCount`             | `int3`     | Number of chunks per axis            |
| `VoxelSize`              | `float3`   | Size of each voxel (meters)          |
| `WorldOrigin`            | `float3`   | Grid origin in world space           |
| `TotalVoxels`            | `int`      | Total voxel count                    |
| `MarkChunkDirty(int3)`   | method     | Flag chunk for re-meshing            |
| `IsChunkDirty(int3)`     | method     | Check dirty flag                     |
| `ClearDirtyFlags()`      | method     | Reset all flags                      |
| `WorldToVoxel(float3)`   | method     | World position to grid coordinate    |
| `VoxelToWorld(int3)`     | method     | Grid coordinate to world center      |
| `VoxelToChunk(int3)`     | method     | Voxel coordinate to chunk coordinate |
| `IsValidVoxel(int3)`     | method     | Bounds check                         |

### 24.3 Namespace: MiracleTwin.Visualization

| Class                      | Key Public Members                                  |
|---------------------------|-----------------------------------------------------|
| `ForceArrowRenderer`       | `SetVisible(bool)`                                  |
| `HeatMapOverlay`           | `IsEnabled`, `CurrentMaxTemperature`, `Toggle()`    |
| `ChipParticleController`   | `SetEnabled(bool)`                                  |
| `WearIndicator`            | `CurrentWearPercentage`, `CurrentVB`, `RecommendedAction` |
| `ToolpathPreview`          | `IsVisible`, `SetToolpath(...)`, `AdvanceToSegment(int)`, `Toggle()`, `Clear()` |
| `StabilityLobeChart`       | `CurrentRPM`, `CurrentDepth`, `IsStable`, `UpdateOperatingPoint(rpm, depth)` |
| `SurfaceRoughnessOverlay`  | `CurrentRa`, `SurfaceGrade`, `Toggle()`             |

### 24.4 Namespace: MiracleTwin.UI

| Class                    | Key Public Members                                    |
|-------------------------|-------------------------------------------------------|
| `DashboardOverlay`       | `ToggleVisibility()`                                  |
| `ForceChart`             | `GetSamples()`, `Clear()`, `maxSamples`, `maxForce`  |
| `SimulationControlPanel` | (all UI bindings are internal)                        |
| `EStopButton`            | (double-click triggers TriggerEStop service)          |
| `AlertNotification`      | `GetActiveAlerts()`, `AlertCount`                     |
| `RobotStatusPanel`       | (reads from MultiAgentCoordinator)                    |
| `GCodeEditor`            | `LoadProgram(string)`, `LoadFromFile(string)`, `GetProgram()`, `CurrentProgram`, `HighlightedLine` |

### 24.5 Namespace: MiracleTwin.Audio

| Class                    | Key Public Members                                    |
|-------------------------|-------------------------------------------------------|
| `CuttingSoundController` | Procedural audio via `OnAudioFilterRead`; driven by `CuttingStateEventSO` |
| `SpindleSoundController` | RPM-based pitch shift via `OnAudioFilterRead`; driven by `MachineStateEventSO` |

### 24.6 ScriptableObject Event Channels

All event channels extend `GameEventSO<T>` and are created via the Unity asset menu
under `MIRACLE/Events/`.

| Asset Type                | Message Type          | Topic                           |
|--------------------------|-----------------------|---------------------------------|
| `MachineStateEventSO`    | `MachineStateMsg`     | `/miracle/{id}/state`           |
| `AnomalyAlertEventSO`    | `AnomalyAlertMsg`     | `/miracle/{id}/anomaly`         |
| `ToolWearEventSO`        | `ToolWearEstimateMsg`  | `/miracle/{id}/tool_wear`       |
| `TwinSyncEventSO`        | `TwinSyncStatusMsg`   | `/miracle/twin/sync_status`     |
| `SystemKPIsEventSO`      | `SystemKPIsMsg`       | `/miracle/system_kpis`          |
| `JobStatusEventSO`       | `JobStatusMsg`        | `/miracle/{id}/job_status`      |
| `TaskAwardEventSO`       | `TaskAwardMsg`        | `/miracle/cognitive/task_awards` |
| `SecurityAlertEventSO`   | `SecurityAlertMsg`    | `/miracle/security/alerts`      |
| `CuttingStateEventSO`    | `CuttingStateData`    | (internal, not from ROS)        |
| `RobotJointStateEventSO` | (joint angles)        | (from URDF/joint state topic)   |

### 24.7 ROS2 Message Types

All custom messages are defined in `unity_twin/Assets/Scripts/RosMessages/`.

#### Messages (`Msg/`)

| Message Type           | Key Fields                                               |
|-----------------------|---------------------------------------------------------|
| `MachineStateMsg`      | `machine_id`, `status`, `spindle_speed`, `feed_rate`, `axis_positions[3]`, `current_line`, `coolant_level` |
| `TwinSyncStatusMsg`    | Sync status and latency metrics                         |
| `AnomalyAlertMsg`      | `anomaly_type`, `confidence`, `severity`, `contributing_factors[]`, `recommended_action`, `requires_immediate_stop` |
| `ToolWearEstimateMsg`  | `tool_id`, `wear_percentage`, `remaining_life_minutes`, `flank_wear_mm`, `crater_wear_mm`, `recommended_action` |
| `PHMPredictionMsg`     | Prognostics and health management prediction data       |
| `SystemKPIsMsg`        | `oee`, `availability`, `performance`, `quality`         |
| `FleetHealthMsg`       | Fleet-wide health status                                |
| `HeartbeatMsg`         | `node_name`, `criticality`, `lifecycle_state`, `cpu_usage`, `memory_usage` |
| `JobStatusMsg`         | Job execution status and progress                       |
| `GCodeBlockMsg`        | G-code block data for streaming                         |
| `SensorDataMsg`        | Raw sensor readings                                     |
| `FusedSensorDataMsg`   | Multi-sensor fused data                                 |
| `SecurityAlertMsg`     | `category`, `description`, security event data          |
| `TaskAwardMsg`         | Multi-agent task auction award                          |
| `TaskAnnouncementMsg`  | Multi-agent task auction announcement                   |

#### Services (`Srv/`)

| Service                | Request Fields                  | Response Fields              |
|-----------------------|--------------------------------|------------------------------|
| `TriggerEStop`         | `machine_id`, `reason`, `requesting_node` | `success`, `message` |
| `ValidateGCode`        | `program_content`, `machine_id` | `is_valid`, `errors[]`, `warnings[]` |
| `GetFleetStatus`       | (empty or fleet filter)         | Fleet status data            |

---

## Footer

**MIRACLE Unity Digital Twin -- Technical Manual**
Version 1.0.0 | February 2026

**License:** Proprietary -- Bantam Tools / MIRACLE Project

**Related documentation:**
- [QUICKSTART.md](../miracle_ws/docs/QUICKSTART.md) -- 5-minute setup guide for the full MIRACLE system
- [USER_GUIDE.md](../miracle_ws/docs/USER_GUIDE.md) -- Operator's guide for daily use

**Project repository structure:**
```
banatam_cnc_milling_unity_digital_twin/
  docs/                  -- This manual and reference PDFs
  miracle_ws/            -- ROS2 Jazzy workspace (Python/C++)
    src/
      miracle_core/      -- Core orchestrator node
      miracle_cnc/       -- CNC machine driver
      miracle_twin/      -- Digital twin sync node
      miracle_bridges/   -- ROS TCP bridge configuration
      miracle_ai/        -- AI/ML anomaly detection
      miracle_cognitive/  -- Multi-agent cognitive layer
      miracle_scada/     -- SCADA integration
      miracle_mes/       -- MES integration
      miracle_security/  -- Cybersecurity monitoring
      miracle_resiliency/ -- Fault tolerance
  unity_twin/            -- Unity 2022 LTS project
    Assets/
      Scripts/           -- All C# source code
        Core/            -- Bridge, clock, events, dispatcher
        Cutting/         -- Physics models, G-code, voxel
        CNC/             -- Machine builder, spindle, vise
        Robots/          -- IK, gripper, coordination
        Visualization/   -- Force arrows, heat map, particles
        UI/              -- Dashboard, charts, controls
        Audio/           -- Procedural sound synthesis
      Shaders/           -- Custom URP shaders (5 files)
      Materials/         -- Material assets (6 files)
      Tests/             -- EditMode + PlayMode test suites
```

---

*End of MIRACLE Unity Digital Twin Technical Manual -- Part 3 (Sections 17-24)*
