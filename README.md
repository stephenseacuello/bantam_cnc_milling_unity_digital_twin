# MIRACLE CNC Milling Digital Twin

**Manufacturing Intelligence, Robotics, Automation, CNC, Learning, Engineering**

A Unity 3D digital twin with a ROS2 Jazzy backend for real-time CNC milling simulation, predictive analytics, and closed-loop manufacturing control.

`2739 tests` | `54+ features` | `13 ROS2 packages` | `Apache-2.0`

---

## Project Overview

- **Unity 3D digital twin** for Bantam Tools Explorer and Coast Runner CR-1 CNC mills with real-time 3D visualization, multi-machine fleet monitoring, and interactive dashboards
- **ROS2 Jazzy backend** with lifecycle-managed nodes across 13 packages covering CNC control, SCADA, AI/ML, security, MES, cognitive reasoning, and resiliency
- **Real-time voxel-based cutting simulation** using GPU compute shaders with marching cubes mesh rendering, Altintas force models, and chatter prediction
- **Predictive analytics and closed-loop control** including adaptive feedrate, tool wear prediction, thermal compensation, anomaly detection, and decision support

## Architecture

```
Unity Digital Twin  <--- ROS-TCP-Connector --->  ROS2 Jazzy Backend
|                                                |
+-- Dashboard UI (UI Toolkit)                    +-- miracle_core
|     Fleet Overview                             |     Lifecycle nodes, heartbeat mixin
|     Decision Support                           |
|     Tolerance Analyzer                         +-- miracle_cnc
|                                                |     G-code executor + macros
+-- Voxel Cutting Engine (GPU)                   |
|     Compute shaders                            +-- miracle_twin
|     Marching cubes rendering                   |     Cutting sim, prediction,
|                                                |     adaptive control, thermal model,
+-- G-Code Executor + Lookahead                  |     tool library, material DB, chip load
|     Path smoothing                             |
|     Collision detection                        +-- miracle_scada
|                                                |     Alarms, escalation, alerts, KPIs,
+-- Stability Lobe Predictor                     |     shift reports, capability profiler,
+-- Vibration Analyzer                           |     notifications
+-- Force Chart (multi-axis)                     |
|                                                +-- miracle_ai
+-- Robot Arm Controllers                        |     Anomaly detection, PHM
|                                                |
+-- Record/Replay System                         +-- miracle_security
      Timeline scrubber                          |     IDS, audit, secure storage,
                                                 |     signing, attestation
                                                 |
                                                 +-- miracle_mes
                                                 |     Digital thread, energy,
                                                 |     job scheduler, maintenance
                                                 |
                                                 +-- miracle_cognitive
                                                 |     Knowledge graph, causal inference,
                                                 |     explanation, root cause,
                                                 |     action ranking, feedback
                                                 |
                                                 +-- miracle_resiliency
                                                 |     Recovery, chaos engineering,
                                                 |     partition detection
                                                 |
                                                 +-- miracle_bridges
                                                 |     Kafka, OPC-UA, Modbus, MQTT
                                                 |
                                                 +-- miracle_msgs
                                                 +-- miracle_bringup
                                                 +-- miracle_dashboard
                                                 +-- miracle_gazebo
                                                 +-- miracle_microros
                                                 +-- miracle_unity_bridge
```

## Key Capabilities

### Predictive Digital Twin

- Altintas cutting force model (tangential, radial, axial)
- Taylor tool wear prediction with remaining useful life estimation
- Thermal zone modeling and compensation
- Stability lobe diagram / chatter prediction
- G-code lookahead with path smoothing
- Collision detection and avoidance

### Situational Awareness

- Alert correlation and escalation workflows
- Anomaly pattern detection (AI/ML-based)
- Root cause analysis via causal inference
- Fleet comparison across multiple machines
- Decision support with action ranking
- Shift reports and operator notifications

### Closed-Loop Control

- Adaptive feedrate optimization
- Preemptive overrides based on predicted conditions
- G-code optimization (chip load, coolant advisory)
- Real-time parameter adjustment from twin feedback

### Manufacturing Intelligence

- Process capability analysis (Cp/Cpk)
- OEE tracking and KPI dashboards
- Energy monitoring and optimization
- Material genealogy and digital thread
- Predictive maintenance scheduling

### Security and Resiliency

- G-code signing and verification
- DDS encryption (SROS2) with mutual TLS
- Encrypted audit logs
- Intrusion detection system (IDS)
- Chaos engineering and fault injection
- Network partition detection and recovery

## Prerequisites

- Unity 2022.3 LTS or later
- .NET 6 / C# 10
- Python 3.11+
- Docker and Docker Compose
- (Optional) ROS2 Jazzy for native development
- (Optional) GPU with compute shader support (CPU fallback available)

## Quick Start

### Option A: Unity Only (Standalone, No ROS2 Needed)

1. Clone the repository:

   ```bash
   git clone https://github.com/your-org/banatam_cnc_milling_unity_digital_twin.git
   cd banatam_cnc_milling_unity_digital_twin
   ```

2. Open `unity_twin/` in Unity 2022.3 LTS or later.

3. From the menu bar: **MIRACLE > Wire Dashboard**

4. Press **Play**. The `LocalCNCTestDriver` provides simulated CNC data without any ROS2 backend.

### Option B: Full Stack (Docker Compose)

1. Clone and enter the workspace:

   ```bash
   git clone https://github.com/your-org/banatam_cnc_milling_unity_digital_twin.git
   cd banatam_cnc_milling_unity_digital_twin/miracle_ws
   ```

2. Start all services:

   ```bash
   docker compose -f docker/docker-compose.yaml up -d
   ```

3. Verify services are running:

   ```bash
   docker compose -f docker/docker-compose.yaml ps
   ```

4. Open `unity_twin/` in Unity, wire the dashboard, and press Play. The twin will connect to the ROS2 backend via ROS-TCP-Connector on port 10000.

5. To stop:

   ```bash
   docker compose -f docker/docker-compose.yaml down
   ```

### Option C: Native ROS2 Development

1. Install ROS2 Jazzy following the [official instructions](https://docs.ros.org/en/jazzy/Installation.html).

2. Build the workspace:

   ```bash
   cd miracle_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

3. Launch the system:

   ```bash
   ros2 launch miracle_bringup miracle_system.launch.py
   ```

4. Open `unity_twin/` in Unity and press Play.

## Testing

The project includes **2739 tests** across Python (ROS2 packages) and C# (Unity).

### Python Tests

```bash
cd miracle_ws
python3 -m pytest src/ --ignore=src/miracle_unity_bridge -v
```

### Unity Tests

Open **Window > General > Test Runner** in the Unity Editor to run EditMode and PlayMode tests.

### Full Suite

```bash
# Run all Python tests with coverage
python3 -m pytest src/ --ignore=src/miracle_unity_bridge -v --cov=src --cov-report=term-missing
```

## Key Controls (Play Mode)

| Key / Control    | Action                          |
|------------------|---------------------------------|
| G                | Execute loaded G-code           |
| H                | Toggle HUD overlay              |
| Escape (x2)     | E-STOP (double-tap)             |
| Machine dropdown | Switch between CNC machines     |
| REC button       | Start recording session         |
| Replay button    | Play back recorded session      |
| Timeline scrubber| Seek within replay              |
| File browser     | Load G-code files at runtime    |

## Configuration

Configuration is layered with the following precedence (highest to lowest):

1. **Environment variables**: `MIRACLE_<SECTION>_<KEY>` (e.g., `MIRACLE_ROS_DOMAIN_ID=42`)
2. **YAML config**: `miracle_ws/config/miracle_defaults.yaml`
3. **Unity ScriptableObject**: Assets > Create > MIRACLE > System Configuration

Key configuration files:

| File | Purpose |
|------|---------|
| `miracle_ws/config/miracle_defaults.yaml` | Central ROS2 configuration defaults |
| `miracle_ws/docker/docker-compose.yaml` | Docker stack orchestration |
| `miracle_ws/docker/prometheus/prometheus.yml` | Prometheus scrape targets |
| `miracle_ws/docker/grafana/provisioning/` | Grafana datasources and dashboard provisioning |

## Monitoring

| Service     | URL                        | Description                     |
|-------------|----------------------------|---------------------------------|
| Grafana     | http://localhost:3001      | Dashboards and visualization    |
| Prometheus  | http://localhost:9190      | Metrics collection and queries  |
| Loki        | http://localhost:3100      | Centralized log aggregation     |
| Web Dashboard | http://localhost:3000    | MIRACLE web monitoring UI       |
| MQTT Broker | localhost:1883             | Eclipse Mosquitto (MQTT)        |
| MQTT WebSocket | localhost:9001          | MQTT over WebSocket             |
| Kafka       | localhost:9092 / 29092     | Event streaming platform        |

Default credentials:

- Grafana: admin / `miracle` (override with `GRAFANA_ADMIN_PASSWORD`)
- MQTT: miracle / `miracle` (override with `MQTT_PASSWORD`)

## Documentation

| Document | Path |
|----------|------|
| Getting Started | [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) |
| Architecture | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| Feature Reference | [docs/FEATURES.md](docs/FEATURES.md) |
| Testing Guide | [docs/TESTING.md](docs/TESTING.md) |
| Unity Twin Manual | [docs/UNITY_TWIN_MANUAL.md](docs/UNITY_TWIN_MANUAL.md) |
| ROS2 API Reference | [docs/ROS2_API_REFERENCE.md](docs/ROS2_API_REFERENCE.md) |
| ROS2 Commands | [ROS2_COMMANDS.md](ROS2_COMMANDS.md) |
| Changelog | [CHANGELOG.md](CHANGELOG.md) |

## License

Apache-2.0 -- see [LICENSE](LICENSE) for details.
