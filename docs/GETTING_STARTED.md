# MIRACLE -- Getting Started Guide

This guide walks you from a fresh clone to a fully running CNC milling digital twin. Pick the path that matches your goals: Unity-only for a quick demo, Docker for the full stack, or native ROS2 for backend development.

---

## 1. System Requirements

### Hardware (minimum)

| Resource | Minimum | Recommended |
|----------|---------|-------------|
| RAM | 8 GB | 16 GB |
| GPU | Compute shader support (OpenGL 4.3 / Vulkan) | Discrete GPU with 4+ GB VRAM |
| Disk | 10 GB free | 20 GB (Docker images are large) |

### Software

| Tool | Version | Notes |
|------|---------|-------|
| Unity | 2022.3 LTS or newer | Required for the digital twin UI |
| Python | 3.11+ | Backend and tests |
| Docker Desktop | Latest | Full-stack deployment |
| Git | Any recent | Source control |

### Optional

| Tool | Version | Notes |
|------|---------|-------|
| ROS2 | Jazzy Jalisco | Only for native (non-Docker) backend development |

---

## 2. Clone and Setup

```bash
git clone https://github.com/your-org/banatam_cnc_milling_unity_digital_twin.git
cd banatam_cnc_milling_unity_digital_twin
```

Install Python dependencies (needed for tests and native ROS2 development):

```bash
pip install -r miracle_ws/requirements.txt
```

---

## 3. Option A -- Unity Standalone (Quickest Start)

This gets the 3D digital twin running with simulated sensor data. No Docker, no ROS2 required.

### Steps

1. **Open the project.** In Unity Hub, click "Open" and select the `unity_twin/` directory.

2. **Wait for import.** First-time import takes several minutes while Unity compiles shaders and processes assets.

3. **Wire the dashboard.** In the Unity menu bar, go to `MIRACLE > Wire Dashboard`. This connects all UI panels (force charts, status indicators, KPIs) to their data sources.

4. **Press Play.** Enter Play mode in the Unity Editor.

5. **Execute sample G-code.** Press the `G` key to load and run a built-in G-code program.

6. **What you should see:**
   - The CNC machine animates spindle rotation and axis movement
   - The voxel cutting engine removes material from the workpiece in real time
   - The force chart updates with simulated Fx/Fy/Fz cutting forces
   - The dashboard shows spindle RPM, feed rate, and tool wear indicators

### Key Controls

| Key | Action |
|-----|--------|
| `G` | Load and execute sample G-code |
| `R` | Start/stop replay recording |
| `Escape` | Stop current G-code execution |
| Mouse drag | Orbit camera around workpiece |
| Scroll wheel | Zoom in/out |
| Middle mouse | Pan camera |

### Machine Switching

Use the machine dropdown in the top-left corner of the dashboard to switch between available CNC machines (Bantam Tools Explorer, Coast Runner CR1).

---

## 4. Option B -- Full Docker Stack (Recommended)

This spins up the complete MIRACLE system: ROS2 backend, MQTT broker, Kafka, Prometheus, Grafana, Loki, and the web dashboard.

### Steps

**1. Copy and edit the environment file:**

```bash
cp miracle_ws/docker/.env.example miracle_ws/docker/.env
```

Edit `miracle_ws/docker/.env` and change the default passwords:

```
GRAFANA_ADMIN_PASSWORD=your_secure_password
MQTT_PASSWORD=your_secure_password
ROS_DOMAIN_ID=42
```

**2. Start the Docker stack:**

```bash
cd miracle_ws
docker compose -f docker/docker-compose.yaml up -d
```

**3. Wait for services to become healthy.** The ROS2 container has a 90-second start period. Monitor progress:

```bash
docker compose -f docker/docker-compose.yaml ps
```

Wait until all services show `(healthy)`. Expect 2-3 minutes on first launch.

**4. Verify services are running:**

```bash
# Check all containers
docker compose -f docker/docker-compose.yaml ps

# Expected healthy services:
#   miracle_ros2        (healthy)   -- port 10000
#   miracle_microros    (healthy)   -- port 8888/udp
#   miracle_dashboard   (healthy)   -- port 3000
#   miracle_mqtt        (healthy)   -- port 1883
#   miracle_zookeeper   (healthy)   -- port 2181
#   miracle_kafka       (healthy)   -- port 9092
#   miracle_prometheus              -- port 9190
#   miracle_grafana                 -- port 3001
#   miracle_loki                    -- port 3100
```

**5. Open Unity and connect.** Open the `unity_twin/` project, wire the dashboard (`MIRACLE > Wire Dashboard`), and press Play. The Unity twin connects to the ROS2 bridge on `localhost:10000` automatically.

**6. Verify the ROS2 bridge connection.** In the Unity Console, look for a log line confirming the TCP connection to the ROS-TCP-Endpoint. If you see connection errors, check that the `miracle_ros2` container is healthy and port 10000 is not blocked by a firewall.

### Useful Docker Commands

```bash
# View logs for a specific service
docker compose -f docker/docker-compose.yaml logs -f ros2_miracle

# Restart a single service
docker compose -f docker/docker-compose.yaml restart ros2_miracle

# Tear down everything (preserves volumes)
docker compose -f docker/docker-compose.yaml down

# Tear down everything and delete volumes
docker compose -f docker/docker-compose.yaml down -v
```

### Service Endpoints

| Service | URL | Credentials |
|---------|-----|-------------|
| Web Dashboard | http://localhost:3000 | -- |
| Grafana | http://localhost:3001 | admin / (your GRAFANA_ADMIN_PASSWORD) |
| Prometheus | http://localhost:9190 | -- |
| Loki | http://localhost:3100 | -- |
| MQTT Broker | localhost:1883 | miracle / (your MQTT_PASSWORD) |
| Kafka | localhost:29092 (host) | -- |
| ROS-TCP-Endpoint | localhost:10000 | -- |

---

## 5. Option C -- Native ROS2 Development

For developers who want to modify the ROS2 backend without Docker.

### Prerequisites

Install ROS2 Jazzy Jalisco following the [official instructions](https://docs.ros.org/en/jazzy/Installation.html) for your platform.

### Steps

**1. Install Python dependencies:**

```bash
pip install -r miracle_ws/requirements.txt
```

**2. Build the workspace.**

Full colcon build (if you have all ROS2 message dependencies):

```bash
cd miracle_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Lightweight approach (no colcon, for running tests and scripts only):

```bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)/miracle_ws/src"
```

**3. Source the workspace** (if you used colcon):

```bash
source miracle_ws/install/setup.bash
```

**4. Launch the system:**

```bash
ros2 launch miracle_bringup miracle_system.launch.py
```

**5. Run tests:**

```bash
python3 -m pytest miracle_ws/src/ --ignore=miracle_ws/src/miracle_unity_bridge -v --rootdir=miracle_ws
```

---

## 6. Running Tests

### Python Tests (full suite)

Run all Python tests across all ROS2 packages:

```bash
python3 -m pytest miracle_ws/src/ --ignore=miracle_ws/src/miracle_unity_bridge -v --rootdir=miracle_ws
```

The `--rootdir=miracle_ws` flag is required so pytest resolves package imports correctly.

### Quick Test (single package)

Target a specific package for faster iteration:

```bash
# miracle_twin
python3 -m pytest miracle_ws/src/miracle_twin/test/ -v --rootdir=miracle_ws

# miracle_cnc
python3 -m pytest miracle_ws/src/miracle_cnc/test/ -v --rootdir=miracle_ws

# miracle_scada
python3 -m pytest miracle_ws/src/miracle_scada/test/ -v --rootdir=miracle_ws
```

### Unity Tests

In the Unity Editor: `Window > General > Test Runner`. Run EditMode and PlayMode tests from there.

### Benchmarks

```bash
python3 -m pytest miracle_ws/benchmarks/ -v -s
```

The `-s` flag allows benchmark output (timing results) to print to the console.

---

## 7. Verify Everything Works

Use this checklist after setup to confirm all components are operational.

```bash
# 1. Python tests pass
python3 -m pytest miracle_ws/src/ --ignore=miracle_ws/src/miracle_unity_bridge -v --rootdir=miracle_ws

# 2. Docker services healthy (if using Docker)
docker compose -f miracle_ws/docker/docker-compose.yaml ps

# 3. ROS2 topics are flowing (inside Docker or native)
docker exec miracle_ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"

# 4. Grafana dashboard loads
#    Open http://localhost:3001 in your browser

# 5. Prometheus targets are up
#    Open http://localhost:9190/targets in your browser

# 6. Unity connects to ROS2
#    Press Play in Unity -- check the Console for "Connected to ROS"

# 7. G-code execution works
#    With Unity in Play mode, press G and watch the machine animate
```

---

## 8. First Steps After Setup

Once everything is running, try these in order:

1. **Execute G-code.** Press `G` in Play mode. Watch the spindle move along the toolpath while the voxel engine carves the workpiece.

2. **Watch the force chart.** The force panel shows real-time Fx, Fy, Fz cutting forces. During a cut, forces spike when the tool engages material.

3. **Switch machines.** Use the machine dropdown to switch between the Bantam Tools Explorer and Coast Runner CR1. Each has its own kinematics and workspace envelope.

4. **Open Grafana dashboards.** Navigate to http://localhost:3001 (default credentials: admin / your GRAFANA_ADMIN_PASSWORD). Pre-provisioned dashboards show spindle metrics, system health, and ROS2 node status.

5. **Check Prometheus metrics.** Navigate to http://localhost:9190. Query `miracle_` prefixed metrics to see what the system exposes.

6. **Try the fleet overview panel.** The dashboard includes a fleet panel that shows status for all connected machines simultaneously.

7. **Record a replay.** Press `R` during G-code execution to start recording. Press `R` again to stop. Replays capture the full machine state for later playback.

---

## 9. Project Structure

```
banatam_cnc_milling_unity_digital_twin/
├── unity_twin/                  # Unity project (C# digital twin)
│   ├── Assets/Scripts/
│   │   ├── Core/                # ROS bridge, dispatcher, event system
│   │   ├── CNC/                 # Machine controllers (Bantam, CoastRunner)
│   │   ├── Cutting/             # Voxel engine, cutting forces, G-code parser
│   │   ├── UI/                  # Dashboard panels, charts, status indicators
│   │   ├── Visualization/       # Rendering, materials, visual effects
│   │   ├── Audio/               # Machining sound synthesis
│   │   ├── Robots/              # Robotic arm integration
│   │   ├── RosMessages/         # Auto-generated ROS2 message types
│   │   ├── Editor/              # Dashboard wiring, custom inspectors
│   │   └── Testing/             # Test drivers, local simulation
│   └── Assets/UI/               # UXML layouts, USS stylesheets
│
├── miracle_ws/                  # ROS2 workspace (Python backend)
│   ├── src/                     # 16 ROS2 packages
│   │   ├── miracle_core/        # Lifecycle nodes, exceptions, utilities
│   │   ├── miracle_cnc/         # G-code executor, canned cycles, macros
│   │   ├── miracle_twin/        # Digital twin prediction, simulation, tools
│   │   ├── miracle_scada/       # Alarms, KPIs, OEE reports
│   │   ├── miracle_ai/          # Anomaly detection, PHM, ML models
│   │   ├── miracle_security/    # Intrusion detection, audit, message signing
│   │   ├── miracle_mes/         # Digital thread, job scheduling, energy
│   │   ├── miracle_cognitive/   # Knowledge graph, reasoning engine
│   │   ├── miracle_resiliency/  # Recovery, chaos engineering, fault injection
│   │   ├── miracle_bridges/     # OPC-UA, Modbus, MQTT, Kafka bridges
│   │   ├── miracle_dashboard/   # Web dashboard backend
│   │   ├── miracle_gazebo/      # Gazebo simulation integration
│   │   ├── miracle_microros/    # micro-ROS MCU bridge
│   │   ├── miracle_msgs/        # Custom ROS2 message definitions
│   │   ├── miracle_bringup/     # Launch files, system orchestration
│   │   └── miracle_unity_bridge/ # Unity TCP bridge node
│   ├── config/                  # YAML configs, SROS2 security policies
│   ├── docker/                  # Docker Compose, Dockerfiles, Grafana/Prometheus configs
│   ├── scripts/                 # setup_workspace.sh, build_all.sh, deploy.sh
│   └── requirements.txt         # Python dependencies
│
├── docs/                        # Technical documentation
├── CHANGELOG.md                 # Version history
├── ROS2_COMMANDS.md             # Copy-paste ROS2 commands for testing
└── README.md                    # Project overview
```

---

## 10. Troubleshooting

### Unity cannot connect to ROS2

- Verify the `miracle_ros2` Docker container is healthy: `docker ps | grep miracle_ros2`
- Confirm port 10000 is open: `lsof -i :10000`
- Check that no firewall or VPN is blocking localhost connections
- In Unity, verify the ROS connector is pointed at `127.0.0.1:10000`

### Tests fail with import errors

- Always pass `--rootdir=miracle_ws` to pytest
- Ensure you installed dependencies: `pip install -r miracle_ws/requirements.txt`
- If running inside a virtual environment, make sure it is activated

### Docker services stay unhealthy

- Check container logs: `docker compose -f miracle_ws/docker/docker-compose.yaml logs <service_name>`
- The ROS2 container has a 90-second start period -- give it time
- Kafka depends on ZooKeeper; if ZooKeeper fails, Kafka will never start
- On Apple Silicon Macs, some images may need `platform: linux/amd64` in the compose file

### GPU / shader errors

- MIRACLE includes automatic GPU fallback. If compute shaders are unsupported, the voxel engine switches to a CPU path. Check the Unity Console for fallback messages.
- Update your GPU drivers to the latest version
- On macOS, Metal is used instead of Vulkan -- this is handled automatically

### Grafana shows no data

- Prometheus scrapes metrics every 15-30 seconds. Wait at least 30 seconds after the stack starts.
- Verify Prometheus targets are up at http://localhost:9190/targets
- Confirm the ROS2 container is running (it exposes the metrics endpoint)

### Docker build fails

- Ensure Docker Desktop has at least 8 GB of memory allocated (Settings > Resources)
- Run `docker system prune` to free disk space if builds fail with storage errors
- On first build, image downloads may take 10+ minutes depending on network speed

---

## 11. Next Steps

Once you are up and running, dive deeper with these resources:

| Document | Description |
|----------|-------------|
| [docs/UNITY_TWIN_MANUAL.md](UNITY_TWIN_MANUAL.md) | Full Unity digital twin technical manual |
| [docs/ROS2_API_REFERENCE.md](ROS2_API_REFERENCE.md) | All ROS2 topics, services, and message definitions |
| [ROS2_COMMANDS.md](../ROS2_COMMANDS.md) | Copy-paste ROS2 CLI commands for testing and debugging |
| [CHANGELOG.md](../CHANGELOG.md) | Version history and release notes |
