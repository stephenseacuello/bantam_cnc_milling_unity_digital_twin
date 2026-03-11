# MIRACLE CNC Milling Digital Twin

**Manufacturing Intelligence, Robotics, Automation, CNC, Learning, Engineering**

A Unity 3D digital twin with a ROS2 Jazzy backend for real-time CNC milling simulation.

## Project Overview

- Unity 3D digital twin for **Bantam Tools Explorer** and **Coast Runner CR-1** CNC mills
- ROS2 Jazzy backend with lifecycle nodes
- Real-time voxel-based cutting simulation with GPU compute shaders
- Multi-machine monitoring, anomaly detection, predictive maintenance

## Architecture

```
Unity Digital Twin  <─── ROS-TCP-Connector ───>  ROS2 Jazzy
├── Dashboard UI (UI Toolkit)                    ├── miracle_core (lifecycle nodes)
├── Voxel Cutting Engine (GPU)                   ├── miracle_cnc (state publisher)
├── G-Code Executor                              ├── miracle_bridges (Kafka, OPC-UA, Modbus, MQTT)
├── Multi-Machine Selector                       ├── miracle_scada (HMI, Prometheus)
├── Record/Replay System                         ├── miracle_ai (anomaly detection, PHM)
├── Force/Thermal/Wear Charts                    ├── miracle_security (IDS, audit)
└── Robot Arm Controllers                        ├── miracle_cognitive (multi-agent)
                                                 ├── miracle_resiliency (fleet health)
                                                 └── miracle_twin (sync engine)
```

## Prerequisites

- Unity 2022.3 LTS or later
- .NET 6 / C# 10
- Python 3.11+
- Docker & Docker Compose
- (Optional) ROS2 Jazzy for native development

## Quick Start

1. Clone the repo
2. Open `unity_twin/` in Unity
3. Menu > MIRACLE > Wire Dashboard
4. Press Play — `LocalCNCTestDriver` provides simulated data

## Docker Stack

```bash
cd miracle_ws
docker compose -f docker/docker-compose.yaml up -d
```

Services: `ros2_miracle`, `microros_agent`, `dashboard`, `mqtt_broker`, `zookeeper`, `kafka`, `prometheus`, `grafana`

## Testing

```bash
# Python tests (547 tests)
python3 -m pytest miracle_ws/src/ --ignore=miracle_ws/src/miracle_unity_bridge -v

# Unity tests
# Open Window > General > Test Runner in Unity Editor
```

## Key Controls (Play Mode)

| Key / Control | Action |
|---|---|
| G | Execute loaded G-code |
| H | Toggle HUD |
| Escape (x2) | E-STOP |
| Machine dropdown | Switch CNC machines |
| REC / Replay | Record/playback sessions |

## Configuration

- `miracle_ws/config/miracle_defaults.yaml` — central config
- Env var overrides: `MIRACLE_<SECTION>_<KEY>`
- Unity: Assets > Create > MIRACLE > System Configuration

## Monitoring

| Service | URL |
|---|---|
| Grafana | http://localhost:3001 |
| Prometheus | http://localhost:9190 |
| Web Dashboard | http://localhost:3000 |

## License

Apache-2.0
