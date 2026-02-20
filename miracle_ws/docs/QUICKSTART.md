# MIRACLE Quickstart Guide

Get the **Manufacturing Integrated Resilient Adaptive Control Loop Environment**
running in under 10 minutes.

---

## 1. Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04+ |
| ROS 2 Jazzy | [install guide](https://docs.ros.org/en/jazzy/Installation.html) |
| Python | 3.10+ |
| Node.js | 18+ (for the dashboard) |
| Docker & Docker Compose | Latest (only for Option B) |
| colcon | via `python3-colcon-common-extensions` |
| Unity | 6 LTS (2024.x+) (only for 3D Digital Twin) |
| GPU | DirectX 11+ / Vulkan / Metal (for Unity compute shaders) |

---

## 2. Clone & Setup

```bash
# Clone the repository
git clone <repo-url> banatam_cnc_milling_unity_digital_twin
cd banatam_cnc_milling_unity_digital_twin/miracle_ws

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Run workspace setup (installs Python deps, runs rosdep)
bash scripts/setup_workspace.sh

# (Optional) Clone ROS-TCP-Endpoint for Unity bridge
cd src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git ros_tcp_endpoint
cd ..
```

> **Tip:** Pass `--venv` to create an isolated Python virtual environment:
> ```bash
> bash scripts/setup_workspace.sh --venv
> ```

---

## 3. Build

```bash
bash scripts/build_all.sh
```

This runs a two-stage colcon build:

1. **Stage 1** -- `miracle_msgs` (message/service/action generation)
2. **Stage 2** -- All remaining packages

After building, source the workspace overlay in every new terminal:

```bash
source install/setup.bash
```

---

## 4. Quick Launch Options

### Option A: Full Simulation (all 5 layers)

Launches L1-L5 with 3 simulated CNC machines, digital twin, AI, security,
and cognitive layers. Includes the Unity Digital Twin bridge on TCP port 10000.

```bash
ros2 launch miracle_bringup miracle_full_system.launch.py \
    simulation_mode:=true \
    machine_count:=3
```

Customizable arguments:

| Argument | Default | Description |
|---|---|---|
| `machine_count` | `3` | Number of CNC machines |
| `simulation_mode` | `true` | Simulated or physical hardware |
| `security_enabled` | `true` | Enable L4 security & resiliency |
| `cognitive_enabled` | `true` | Enable L5 cognitive autonomy |

### Option B: Docker Compose (everything containerized)

No local ROS 2 install needed. Spins up the full stack including MQTT broker
and Kafka:

```bash
docker compose -f docker/docker-compose.yaml up -d
```

Monitor logs:

```bash
docker compose -f docker/docker-compose.yaml logs -f
```

Tear down:

```bash
docker compose -f docker/docker-compose.yaml down
```

Services started: `ros2_miracle`, `microros_agent`, `dashboard`,
`mqtt_broker`, `zookeeper`, `kafka`.

### Option C: Minimal (core + one CNC machine)

For quick iteration -- launches a single CNC machine in simulation mode
without the upper layers:

```bash
ros2 launch miracle_bringup cnc_machine.launch.py \
    machine_id:=cnc1 \
    simulation_mode:=true
```

This starts 6 nodes for the machine: `state_publisher`, `gcode_executor`,
`sensor_fusion`, `local_watchdog`, `spc_monitor`, and `rosbag_trigger`.

### Option D: Launch with Unity Digital Twin

Start the simulation with the Unity bridge enabled, then open the Unity project:

```bash
# Terminal 1: Launch MIRACLE simulation (includes Unity bridge)
ros2 launch miracle_bringup miracle_simulation.launch.py machine_count:=1

# Terminal 2: Verify the Unity endpoint is running
ros2 node list | grep unity
```

Then open `unity_twin/` in Unity 6 LTS, press Play, and verify the
"ROS Connected" indicator turns green.

---

## 5. Verify It's Working

Open a second terminal, source the workspace, and run these checks:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Check running nodes

```bash
ros2 node list
```

You should see nodes under `/miracle/` namespaces. For a full simulation expect
output like:

```
/miracle/cnc1/state_publisher
/miracle/cnc1/gcode_executor
/miracle/cnc1/local_watchdog
/miracle/scada/discovery_server
/miracle/mes/job_scheduler
/miracle/twin/sync_engine
/miracle/ai/anomaly_detector
/miracle/security/intrusion_detection
/miracle/resiliency/heartbeat_aggregator
/miracle/cognitive/knowledge_graph
...
```

### Check active topics

```bash
ros2 topic list | grep miracle
```

### Verify heartbeats

The watchdog nodes publish heartbeats at 1 Hz. Confirm they are flowing:

```bash
ros2 topic echo /miracle/cnc1/local_watchdog/heartbeat --once
```

### Quick health summary

```bash
ros2 node list | wc -l   # Full sim: expect 40+ nodes
ros2 topic list | wc -l   # Full sim: expect 80+ topics
```

### Check Unity bridge

```bash
ros2 node list | grep unity
# Expected: /miracle/unity/unity_endpoint_config
```

---

## 6. Start the Dashboard

The web dashboard gives you real-time visibility into the entire system.

```bash
cd src/miracle_dashboard
npm install
npm start
```

Open your browser to **http://localhost:3000**.

The dashboard connects to ROS 2 via `roslib` (rosbridge WebSocket). If you are
running under Docker (Option B), the dashboard is already served at
**http://localhost:3000** automatically.

---

## 7. Start the Unity Digital Twin

The Unity Digital Twin provides a 3D visualization of the manufacturing cell with
real-time CNC motion, voxel material removal, cutting force visualization, and
robot machine tending.

### First-time setup

1. Open Unity Hub and add the `unity_twin/` folder as a project
2. Unity 6 LTS will import all packages (ROS-TCP-Connector, URDF-Importer, Burst, VFX Graph)
3. Open the scene: `Assets/Scenes/ManufacturingCell.unity`
   - Or use the menu: **MIRACLE > Build Manufacturing Cell Scene** to auto-generate it
4. In the Hierarchy, select `_Managers/MiracleBridge` and verify:
   - ROS Bridge IP: `127.0.0.1`
   - ROS Bridge Port: `10000`
   - Machine ID: `cnc1`

### Running

1. Ensure MIRACLE ROS2 is running (Section 4)
2. Press Play in Unity
3. The dashboard should show "ROS Connected" (green indicator)
4. Use the simulation controls:
   - **Space**: Play/Pause simulation
   - **+/-**: Adjust simulation speed
   - **1-5**: Camera presets
   - **H**: Toggle HUD
   - **R**: Reset simulation

### Loading G-Code

Click "Load G-code" in the control panel and select from:
- `face_3x3_block.nc` -- Face milling the block top
- `pocket_50x50.nc` -- 50mm square pocket
- `contour_circle.nc` -- Circular contour pass
- `full_demo.nc` -- Combined roughing + finishing demo

For complete Unity documentation, see [UNITY_TWIN_MANUAL.md](../../docs/UNITY_TWIN_MANUAL.md).

---

## 8. Run Tests

```bash
bash scripts/run_tests.sh
```

This runs `colcon test` across all MIRACLE packages:

`miracle_core`, `miracle_cnc`, `miracle_scada`, `miracle_bridges`,
`miracle_mes`, `miracle_twin`, `miracle_ai`, `miracle_security`,
`miracle_resiliency`, `miracle_cognitive`

Results are printed as a summary table at the end. To re-run tests for a
single package:

```bash
colcon test --packages-select miracle_cnc --event-handlers console_cohesion+
colcon test-result --verbose
```

For Unity tests, open the Unity Test Runner (Window > General > Test Runner)
and run:
- **EditMode**: GCodeParser, CuttingForceEngine, ThermalModel, ToolWear,
  ChipFormation, SurfaceRoughness
- **PlayMode**: MiracleBridge, SimulationClock

---

## 9. What's Next

- **[USER_GUIDE.md](USER_GUIDE.md)** -- Deep dive into the 5-layer architecture,
  configuration, parameter tuning, and production deployment.
- **[UNITY_TWIN_MANUAL.md](../../docs/UNITY_TWIN_MANUAL.md)** -- Complete Unity Digital Twin reference covering cutting physics, voxel material removal, robot tending, and all 130 C# scripts.
- **Launch files** -- Browse `src/miracle_bringup/launch/` to understand
  per-layer launch options.
- **Security setup** -- Run `bash scripts/generate_security.sh` to generate
  SROS2 keystores for production use.
- **Deployment** -- Run `bash scripts/deploy.sh` for automated deployment
  workflows.

---

## 10. Common Issues

### `ROS2 Jazzy not found at /opt/ros/jazzy/setup.bash`

ROS 2 Jazzy is not installed or sourced. Install it from
[docs.ros.org/en/jazzy](https://docs.ros.org/en/jazzy/Installation.html), then
run `source /opt/ros/jazzy/setup.bash` before any other command.

### `colcon build` fails with "Could not find a package configuration file provided by miracle_msgs"

You must build `miracle_msgs` first. The `build_all.sh` script handles this
automatically. If building manually, run:

```bash
colcon build --packages-up-to miracle_msgs --symlink-install
source install/setup.bash
colcon build --symlink-install
```

### Nodes start but topics show no data

Make sure `ROS_DOMAIN_ID` matches across all terminals. The Docker setup uses
`ROS_DOMAIN_ID=42`. Set it explicitly:

```bash
export ROS_DOMAIN_ID=42
```

### Dashboard shows "WebSocket connection failed"

The dashboard needs `rosbridge_server` running. Launch it with:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Or ensure the `hmi_bridge` node in the SCADA layer is active, which provides
the WebSocket endpoint.

### Docker: `miracle_ros2` container restarts in a loop

Check logs with `docker logs miracle_ros2`. Common causes:
- MQTT broker or Kafka not ready yet (they have `depends_on` but no health
  checks). Restart the stack: `docker compose restart ros2_miracle`.
- Port conflicts on `1883`, `9092`, or `3000`. Free the ports or remap them in
  `docker-compose.yaml`.

### `rosdep install` fails with unresolved keys

Some keys may not resolve on all platforms. The setup script treats this as
non-fatal. Install missing system packages manually or skip with:

```bash
rosdep install --from-paths src --ignore-src -y --skip-keys "missing_key"
```

### Unity shows "ROS Disconnected"

Ensure the ROS2 system is running with the Unity bridge:

```bash
ros2 launch miracle_bringup miracle_simulation.launch.py
```

Check that port 10000 is not blocked by a firewall. Verify the endpoint:

```bash
ros2 node list | grep unity
```

### Unity compute shaders fail (pink/magenta materials)

Ensure your GPU supports DirectX 11+ compute shaders. On macOS, Metal is
required. Check Edit > Project Settings > Player > Other Settings > Graphics API.

### Unity packages fail to import

Open Packages/manifest.json and verify the ROS-TCP-Connector and URDF-Importer
Git URLs are accessible. If behind a corporate firewall, download the packages
manually and reference them as local paths.
