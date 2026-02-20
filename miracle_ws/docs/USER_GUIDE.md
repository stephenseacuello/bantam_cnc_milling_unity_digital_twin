# MIRACLE System User Guide

**Manufacturing Intelligence with Real-time Analytics, Control, and Logistics Engine**

Version 1.1.0 | Apache-2.0 License | ROS 2 Jazzy

---

## Table of Contents

- [1. Introduction](#1-introduction)
  - [1.1 What Is MIRACLE?](#11-what-is-miracle)
  - [1.2 Level 5 Autonomous Manufacturing](#12-level-5-autonomous-manufacturing)
  - [1.3 Five-Layer Architecture Overview](#13-five-layer-architecture-overview)
- [2. System Architecture](#2-system-architecture)
  - [2.1 Architecture Diagram](#21-architecture-diagram)
  - [2.2 Layer Descriptions](#22-layer-descriptions)
  - [2.3 Package-to-Layer Mapping](#23-package-to-layer-mapping)
  - [2.4 Namespace Topology](#24-namespace-topology)
- [3. Prerequisites](#3-prerequisites)
- [4. Installation](#4-installation)
  - [4.1 Clone the Repository](#41-clone-the-repository)
  - [4.2 Workspace Setup](#42-workspace-setup)
  - [4.3 Build the Workspace](#43-build-the-workspace)
  - [4.4 Source the Overlay](#44-source-the-overlay)
  - [4.5 Run Tests](#45-run-tests)
- [5. Configuration](#5-configuration)
  - [5.1 miracle_params.yaml Walkthrough](#51-miracle_paramsyaml-walkthrough)
  - [5.2 QoS Profiles](#52-qos-profiles)
  - [5.3 Per-Machine Configuration](#53-per-machine-configuration)
  - [5.4 Environment Variables](#54-environment-variables)
- [6. Package Reference](#6-package-reference)
  - [6.1 miracle_msgs](#61-miracle_msgs)
  - [6.2 miracle_core](#62-miracle_core)
  - [6.3 miracle_cnc](#63-miracle_cnc)
  - [6.4 miracle_scada](#64-miracle_scada)
  - [6.5 miracle_bridges](#65-miracle_bridges)
  - [6.6 miracle_twin](#66-miracle_twin)
  - [6.7 miracle_mes](#67-miracle_mes)
  - [6.8 miracle_ai](#68-miracle_ai)
  - [6.9 miracle_security](#69-miracle_security)
  - [6.10 miracle_resiliency](#610-miracle_resiliency)
  - [6.11 miracle_cognitive](#611-miracle_cognitive)
  - [6.12 miracle_gazebo](#612-miracle_gazebo)
  - [6.13 miracle_bringup](#613-miracle_bringup)
  - [6.14 miracle_dashboard](#614-miracle_dashboard)
  - [6.15 miracle_microros](#615-miracle_microros)
  - [6.16 miracle_unity_bridge](#616-miracle_unity_bridge)
- [7. Launch Files](#7-launch-files)
  - [7.1 Full System Launch](#71-full-system-launch)
  - [7.2 Simulation-Only Launch](#72-simulation-only-launch)
  - [7.3 Physical Deployment Launch](#73-physical-deployment-launch)
  - [7.4 Individual Layer Launch](#74-individual-layer-launch)
  - [7.5 Per-Machine Launch](#75-per-machine-launch)
  - [7.6 Unity Bridge Launch](#76-unity-bridge-launch)
- [8. Dashboard](#8-dashboard)
  - [8.1 Starting the Dashboard](#81-starting-the-dashboard)
  - [8.2 Dashboard Views](#82-dashboard-views)
  - [8.3 ROS Bridge Connection](#83-ros-bridge-connection)
  - [8.4 Unity Digital Twin 3D Visualization](#84-unity-digital-twin-3d-visualization)
- [9. Digital Twin](#9-digital-twin)
  - [9.1 Sync Engine](#91-sync-engine)
  - [9.2 Drift Detection and Correction](#92-drift-detection-and-correction)
  - [9.3 Prediction Scenarios](#93-prediction-scenarios)
  - [9.4 Unity 3D Digital Twin](#94-unity-3d-digital-twin)
- [10. AI/ML Pipeline](#10-aiml-pipeline)
  - [10.1 Anomaly Detection Ensemble](#101-anomaly-detection-ensemble)
  - [10.2 PHM Prediction](#102-phm-prediction)
  - [10.3 Tool Wear Estimation](#103-tool-wear-estimation)
  - [10.4 Chatter Detection](#104-chatter-detection)
  - [10.5 Federated Learning](#105-federated-learning)
- [11. Security](#11-security)
  - [11.1 SROS2 Setup](#111-sros2-setup)
  - [11.2 RBAC Roles](#112-rbac-roles)
  - [11.3 Intrusion Detection](#113-intrusion-detection)
  - [11.4 Attestation](#114-attestation)
  - [11.5 Audit Logging](#115-audit-logging)
- [12. Resiliency](#12-resiliency)
  - [12.1 Supervision Trees](#121-supervision-trees)
  - [12.2 Failover Coordination](#122-failover-coordination)
  - [12.3 Checkpointing](#123-checkpointing)
  - [12.4 Chaos Injection](#124-chaos-injection)
- [13. Cognitive Layer](#13-cognitive-layer)
  - [13.1 Knowledge Graph](#131-knowledge-graph)
  - [13.2 Reasoning Engine](#132-reasoning-engine)
  - [13.3 Planning: BT, HTN, GOAP](#133-planning-bt-htn-goap)
  - [13.4 Multi-Agent Coordination](#134-multi-agent-coordination)
  - [13.5 Reinforcement Learning](#135-reinforcement-learning)
  - [13.6 Self-X Properties](#136-self-x-properties)
- [14. Protocol Bridges](#14-protocol-bridges)
  - [14.1 OPC-UA Bridge](#141-opc-ua-bridge)
  - [14.2 Sparkplug B Bridge](#142-sparkplug-b-bridge)
  - [14.3 MTConnect Agent](#143-mtconnect-agent)
  - [14.4 Modbus TCP Bridge](#144-modbus-tcp-bridge)
  - [14.5 Kafka Bridge](#145-kafka-bridge)
- [15. micro-ROS Firmware](#15-micro-ros-firmware)
  - [15.1 ESP32 Sensor Bridge](#151-esp32-sensor-bridge)
  - [15.2 STM32 Sensor Bridge](#152-stm32-sensor-bridge)
  - [15.3 Flashing Firmware](#153-flashing-firmware)
  - [15.4 Debugging](#154-debugging)
- [16. Docker Deployment](#16-docker-deployment)
  - [16.1 Docker Compose Services](#161-docker-compose-services)
  - [16.2 Building Images](#162-building-images)
  - [16.3 Production Configuration](#163-production-configuration)
- [17. Monitoring and Observability](#17-monitoring-and-observability)
  - [17.1 System KPIs](#171-system-kpis)
  - [17.2 OEE Metrics](#172-oee-metrics)
  - [17.3 Heartbeat Monitoring](#173-heartbeat-monitoring)
  - [17.4 Fleet Health](#174-fleet-health)
- [18. Troubleshooting](#18-troubleshooting)
- [19. API Reference](#19-api-reference)
  - [19.1 Message Types](#191-message-types)
  - [19.2 Service Types](#192-service-types)
  - [19.3 Action Types](#193-action-types)
- [20. Unity Digital Twin](#20-unity-digital-twin)
- [21. Glossary](#21-glossary)

---

## 1. Introduction

### 1.1 What Is MIRACLE?

MIRACLE (Manufacturing Intelligence with Real-time Analytics, Control, and Logistics
Engine) is a comprehensive ROS 2-based framework for autonomous CNC milling
operations. It integrates real-time machine control, digital twin simulation,
AI-driven predictive analytics, cybersecurity, fault tolerance, and cognitive
autonomous decision-making into a single, layered architecture.

The system is designed to manage a fleet of CNC milling machines (default: three
machines -- a 3-axis vertical mill, a 5-axis horizontal mill, and a CNC lathe)
through a unified ROS 2 middleware bus running on ROS 2 Jazzy.

MIRACLE bridges the gap between traditional industrial automation (PLC/SCADA) and
modern autonomous systems by providing:

- Real-time sensor fusion from micro-ROS-enabled microcontrollers (ESP32, STM32)
- Digital twin synchronization with Gazebo simulation
- AI/ML inference for anomaly detection, predictive health management, and chatter suppression
- Zero-trust security with SROS2, intrusion detection, and device attestation
- Erlang-inspired supervision trees for fault tolerance and automatic recovery
- Level 5 cognitive autonomy with knowledge graphs, multi-agent task allocation, and reinforcement learning

### 1.2 Level 5 Autonomous Manufacturing

MIRACLE targets Level 5 Autonomous Manufacturing, defined as a system that can:

| Level | Name           | Description                                           |
|-------|----------------|-------------------------------------------------------|
| L1    | Manual         | Human operator controls all machine functions          |
| L2    | Assisted       | Automated monitoring with human-initiated actions      |
| L3    | Conditional    | System manages routine operations; human handles exceptions |
| L4    | Highly Auto.   | System manages all operations; human available for escalation |
| L5    | Full Autonomy  | System self-manages, self-heals, self-optimizes without human intervention |

At Level 5, the MIRACLE system can autonomously schedule jobs, optimize cutting
parameters, detect and respond to anomalies, recover from failures, and
continuously improve its own performance through reinforcement learning --
all without requiring human intervention for normal operations.

### 1.3 Five-Layer Architecture Overview

MIRACLE is organized into five distinct layers, each building upon the capabilities
of the layers beneath it:

| Layer | Name                     | Packages                                      |
|-------|--------------------------|-----------------------------------------------|
| L1    | CNC Machine Control      | `miracle_cnc`, `miracle_microros`             |
| L2    | SCADA / Supervisory      | `miracle_scada`, `miracle_bridges`            |
| L3    | MES / Digital Twin / AI  | `miracle_mes`, `miracle_twin`, `miracle_ai`   |
| L4    | Security / Resiliency    | `miracle_security`, `miracle_resiliency`      |
| L5    | Cognitive / Autonomous   | `miracle_cognitive`                           |

Cross-cutting packages: `miracle_msgs`, `miracle_core`, `miracle_bringup`,
`miracle_gazebo`, `miracle_dashboard`, `miracle_unity_bridge`.

---

## 2. System Architecture

### 2.1 Architecture Diagram

```
+=========================================================================+
|                    UNITY DIGITAL TWIN (3D Visualization)                |
|  miracle_unity_bridge                                                   |
|  +---------------------------+     +------------------------------+     |
|  | ROS-TCP-Endpoint          |     | Unity 6 LTS (URP)            |     |
|  | (TCP port 10000)          |<--->| Voxel Material Removal       |     |
|  | Joint States, Forces,     |     | Altintas Force Model         |     |
|  | Tool Wear, Thermal Data   |     | Thermal + Wear Visualization |     |
|  +---------------------------+     | VFX Graph Chip Particles     |     |
|                                    +------------------------------+     |
+=========================|===============================================+
                          | TCP/IP
+=========================|===============================================+
|                        L5 - COGNITIVE AUTONOMY                          |
|  miracle_cognitive                                                      |
|  +---------------+  +-----------+  +-------------+  +----------------+ |
|  | Knowledge     |  | Planning  |  | Multi-Agent |  | Self-X         | |
|  | Graph +       |  | BT / HTN  |  | Coordination|  | Optimizer +    | |
|  | Ontology +    |  | / GOAP    |  | Auction +   |  | Healer +      | |
|  | Reasoning +   |  | Goal Mgr  |  | Coalition + |  | Configurer +  | |
|  | Causal Inf.   |  |           |  | Consensus   |  | Protector     | |
|  +-------+-------+  +-----+-----+  +------+------+  +-------+--------+ |
|          |               |                |                  |          |
|  +-------+-------+  +---+---+                                          |
|  | RL Optimizer + |  | NLP + |    Learning + Human Interface            |
|  | Federated Lrn  |  | Expl. |                                         |
|  +----------------+  +-------+                                          |
+=========================|=====|=========================================+
                          |     |
+=========================|=====|=========================================+
|                    L4 - SECURITY + RESILIENCY                           |
|  miracle_security              miracle_resiliency                       |
|  +------------------+          +--------------------+                   |
|  | Intrusion Det.   |          | Supervisor Root    |                   |
|  | Attestation      |          | Heartbeat Agg.     |                   |
|  | Threat Response  |          | Failover Coord.    |                   |
|  | Access Enforcer  |          | Checkpoint Mgr     |                   |
|  | Audit Logger     |          | Recovery Orch.     |                   |
|  | SROS2 Manager    |          | Chaos Injector     |                   |
|  +--------+---------+          +---------+----------+                   |
+==========|============================|=================================+
           |                            |
+==========|============================|=================================+
|                 L3 - MES / DIGITAL TWIN / AI                            |
|  miracle_mes           miracle_twin          miracle_ai                 |
|  +--------------+      +---------------+     +-----------------+        |
|  | Job Sched.   |      | Sync Engine   |     | Anomaly Det.    |        |
|  | Fleet Mgr    |      | Gazebo Bridge |     | PHM Predictor   |        |
|  | Digital Thr. |      | Prediction    |     | Tool Wear Est.  |        |
|  | Resource Mgr |      | Scenario Mgr  |     | Chatter Det.    |        |
|  | OEE Calc.    |      |               |     | Model Manager   |        |
|  +------+-------+      +-------+-------+     +--------+--------+        |
+=========|===================|===================|=======================+
          |                   |                   |
+=========|===================|===================|=======================+
|                    L2 - SCADA / SUPERVISORY                             |
|  miracle_scada                    miracle_bridges                       |
|  +-------------------+           +-------------------+                  |
|  | Discovery Server  |           | OPC-UA Bridge     |                  |
|  | Traffic Manager   |           | Sparkplug B Bridge|                  |
|  | Alarm Manager     |           | MTConnect Agent   |                  |
|  | Historian         |           | Modbus TCP Bridge |                  |
|  | HMI Bridge        |           | Kafka Bridge      |                  |
|  +---------+---------+           +---------+---------+                  |
+=============|=================================|=========================+
              |                                 |
+=============|=================================|=========================+
|                    L1 - CNC MACHINE CONTROL                             |
|  miracle_cnc                         miracle_microros                   |
|  +-------------------+               +-------------------+              |
|  | State Publisher   |               | ESP32 Firmware    |              |
|  | G-code Executor   |  <-- ROS2 --> | (WiFi/UDP)        |              |
|  | Sensor Fusion     |               +-------------------+              |
|  | Local Watchdog    |               | STM32 Firmware    |              |
|  | SPC Monitor       |               | (Serial/UART)     |              |
|  | Rosbag Trigger    |               +-------------------+              |
|  +-------------------+                                                  |
|                                                                         |
|              +--[Physical CNC Machines]--+                              |
|              | CNC1: 3-axis Vertical Mill |                             |
|              | CNC2: 5-axis Horiz. Mill   |                             |
|              | CNC3: CNC Lathe            |                             |
|              +----------------------------+                             |
+==========================================================================+
```

### 2.2 Layer Descriptions

**L1 -- CNC Machine Control** provides the direct interface to physical or
simulated CNC machines. It publishes machine state at 50 Hz, fuses sensor
data at 100 Hz, executes G-code programs, monitors statistical process
control, and triggers event-driven rosbag recording. micro-ROS firmware on
ESP32 and STM32 MCUs provides raw sensor data (vibration, current, temperature,
acoustic emission, coolant flow) over WiFi/UDP and serial/UART transports.

**L2 -- SCADA / Supervisory** handles device discovery, network traffic
management, alarm aggregation, time-series data archival (via TimescaleDB),
and WebSocket-based HMI bridging. Protocol bridges translate between
ROS 2 and industrial protocols (OPC-UA, Sparkplug B, MTConnect, Modbus TCP,
Kafka) for integration with legacy SCADA/MES systems.

**L3 -- MES / Digital Twin / AI** orchestrates production at the factory level.
The MES subsystem schedules jobs, manages the machine fleet, tracks the digital
thread for full traceability, manages resources (tools, materials), and
calculates OEE metrics. The digital twin subsystem synchronizes physical machine
state with a Gazebo simulation, runs predictive what-if scenarios, and detects
drift between physical and virtual. The AI subsystem runs an ensemble anomaly
detector, predicts remaining useful life (PHM), estimates tool wear in real time,
detects chatter vibration, and manages ML model versioning and deployment.

**L4 -- Security / Resiliency** provides defense-in-depth security through
intrusion detection (hybrid signature + anomaly), device attestation, automated
threat response with isolation capability, RBAC access enforcement, tamper-proof
audit logging with hash chains, and SROS2 DDS security management. The resiliency
subsystem implements Erlang-style supervision trees (one-for-one restart strategy),
heartbeat aggregation for system health scoring, hot-standby failover with quorum,
periodic state checkpointing, progressive recovery orchestration, and chaos
injection for resilience testing.

**L5 -- Cognitive / Autonomous** implements full Level 5 autonomy through a
knowledge graph (Neo4j-backed), OWL ontology management with HermiT reasoning,
causal inference, hybrid planning (behavior trees, HTN, GOAP), multi-agent
coordination via contract-net auctions and Shapley-value coalition formation,
Raft consensus, PPO reinforcement learning, federated learning with differential
privacy, self-X properties (self-optimization, self-healing, self-configuration,
self-protection), and a natural language interface for human interaction.

### 2.3 Package-to-Layer Mapping

| Package              | Layer | Build Type   | Description                                    |
|----------------------|-------|--------------|------------------------------------------------|
| `miracle_msgs`       | All   | ament_cmake  | Message, service, and action definitions       |
| `miracle_core`       | All   | ament_python | Core utilities and base classes                |
| `miracle_cnc`        | L1    | ament_python | CNC machine control nodes                      |
| `miracle_microros`   | L1    | ament_cmake  | micro-ROS MCU firmware and config              |
| `miracle_scada`      | L2    | ament_python | SCADA/Supervisory nodes                        |
| `miracle_bridges`    | L2    | ament_python | Industrial protocol bridge nodes               |
| `miracle_mes`        | L3    | ament_python | MES/Orchestration nodes                        |
| `miracle_twin`       | L3    | ament_python | Digital twin nodes                             |
| `miracle_ai`         | L3    | ament_python | AI/ML nodes                                    |
| `miracle_security`   | L4    | ament_python | Security nodes                                 |
| `miracle_resiliency` | L4    | ament_python | Resiliency and fault tolerance nodes           |
| `miracle_cognitive`  | L5    | ament_python | Cognitive autonomy nodes (22 nodes)            |
| `miracle_gazebo`     | Sim   | ament_python | Gazebo simulation assets                       |
| `miracle_bringup`    | Infra | ament_python | Launch files and configuration                 |
| `miracle_dashboard`  | UI    | Node.js      | React web dashboard                            |
| `miracle_unity_bridge` | UI/Twin | ament_python | ROS-TCP-Endpoint bridge for Unity 3D visualization |

### 2.4 Namespace Topology

All MIRACLE nodes operate under the `/miracle/` namespace with the following
sub-namespaces:

```
/miracle/
  cnc1/                  # CNC Machine 1 (3-axis vertical mill)
    state_publisher
    gcode_executor
    sensor_fusion
    local_watchdog
    spc_monitor
    rosbag_trigger
  cnc2/                  # CNC Machine 2 (5-axis horizontal mill)
    (same nodes as cnc1)
  cnc3/                  # CNC Machine 3 (CNC lathe)
    (same nodes as cnc1)
  scada/
    discovery_server
    traffic_manager
    alarm_manager
    historian
    hmi_bridge
  bridges/
    opc_ua_bridge
    sparkplug_bridge
    mtconnect_agent
    modbus_bridge
    kafka_bridge
  mes/
    job_scheduler
    fleet_manager
    digital_thread
    resource_manager
    oee_calculator
  twin/
    sync_engine
    gazebo_bridge
    prediction_runner
    scenario_manager
  ai/
    anomaly_detector
    phm_predictor
    tool_wear_estimator
    chatter_detector
    model_manager
  security/
    intrusion_detection
    attestation_verifier
    threat_response
    access_enforcer
    audit_logger
    sros2_manager
  resiliency/
    supervisor_root
    heartbeat_aggregator
    failover_coordinator
    checkpoint_manager
    recovery_orchestrator
    chaos_injector
  cognitive/
    knowledge_graph
    ontology_manager
    reasoning_engine
    causal_inference
    goal_manager
    behavior_tree_executor
    htn_planner
    goap_planner
    agent_registry
    task_allocator
    auction_manager
    coalition_former
    consensus_protocol
    rl_optimizer
    federated_coordinator
    self_optimizer
    self_healer
    self_configurer
    self_protector
    nlp_interface
    explanation_generator
    human_escalation
  unity/
    unity_endpoint_config
```

---

## 3. Prerequisites

### System Requirements

| Requirement          | Minimum                        | Recommended                     |
|----------------------|--------------------------------|---------------------------------|
| Operating System     | Ubuntu 24.04 LTS               | Ubuntu 24.04 LTS               |
| ROS 2 Distribution   | Jazzy Jalisco                  | Jazzy Jalisco                   |
| Python               | 3.12+                          | 3.12+                           |
| Node.js              | 18 LTS                         | 20 LTS                          |
| Docker               | 24.0+                          | 25.0+                           |
| Docker Compose       | 2.20+                          | 2.24+                           |
| RAM                  | 8 GB                           | 16+ GB                          |
| Disk                 | 20 GB free                     | 50+ GB free                     |
| CPU                  | 4 cores                        | 8+ cores                        |
| Unity                | 6 LTS (2024.x+)               | 6 LTS (2024.x+)                |
| GPU                  | DirectX 11+ / Vulkan / Metal   | DirectX 12 / Vulkan (for compute shaders) |

### Software Dependencies

**ROS 2 Jazzy** -- Install from https://docs.ros.org/en/jazzy/Installation.html

```bash
# Verify ROS 2 installation
source /opt/ros/jazzy/setup.bash
ros2 --version
```

**Build tools:**

```bash
sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake
```

**Python libraries:**

```bash
pip3 install numpy scipy scikit-learn opcua paho-mqtt pymodbus kafka-python
```

**Node.js (for dashboard):**

```bash
# Using NodeSource
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs
```

**PlatformIO (for micro-ROS firmware):**

```bash
pip3 install platformio
```

**SROS2 (for security):**

```bash
sudo apt-get install -y ros-jazzy-sros2
```

---

## 4. Installation

### 4.1 Clone the Repository

```bash
git clone https://github.com/banatam/miracle_cnc_digital_twin.git
cd miracle_cnc_digital_twin/miracle_ws
```

### 4.2 Workspace Setup

The setup script checks for ROS 2 Jazzy, installs Python dependencies, runs
`rosdep`, and optionally creates a virtual environment:

```bash
# Standard setup
bash scripts/setup_workspace.sh

# With Python virtual environment
bash scripts/setup_workspace.sh --venv
```

The script performs the following steps:

1. Verifies ROS 2 Jazzy is installed at `/opt/ros/jazzy/setup.bash`
2. Optionally creates a Python venv at `.venv/` with `--system-site-packages`
3. Installs core Python dependencies (numpy, scipy, scikit-learn)
4. Installs optional protocol bridge dependencies (opcua, paho-mqtt, pymodbus, kafka-python)
5. Verifies `colcon` and `rosdep` are available
6. Runs `rosdep install` for all workspace packages

### 4.3 Build the Workspace

The build script uses a two-stage approach -- messages first, then all packages:

```bash
bash scripts/build_all.sh
```

**Stage 1:** Builds `miracle_msgs` (message generation must complete before
dependent packages can resolve message types):

```bash
colcon build --symlink-install --packages-up-to miracle_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Stage 2:** Builds all remaining packages:

```bash
source install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4.4 Source the Overlay

After building, source the workspace overlay in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source <workspace>/install/setup.bash
```

### 4.5 Run Tests

```bash
bash scripts/run_tests.sh
```

This runs `colcon test` for all Python packages (miracle_core through
miracle_cognitive) and prints a summary via `colcon test-result --verbose`.

---

## 5. Configuration

### 5.1 miracle_params.yaml Walkthrough

The master parameter file at `miracle_bringup/config/miracle_params.yaml` contains
default parameters for every node in the system, organized by package namespace.

**Structure:**

```yaml
miracle_cnc:           # L1 parameters
  state_publisher:
    ros__parameters:
      machine_id: "cnc1"
      publish_rate_hz: 50.0
      simulation_mode: true
      # ...

miracle_scada:         # L2 parameters
  discovery_server:
    ros__parameters:
      discovery_interval_sec: 5.0
      # ...

miracle_bridges:       # L2 protocol bridge parameters
miracle_mes:           # L3 MES parameters
miracle_twin:          # L3 digital twin parameters
miracle_ai:            # L3 AI/ML parameters
miracle_security:      # L4 security parameters
miracle_resiliency:    # L4 resiliency parameters
miracle_cognitive:     # L5 cognitive parameters
```

**Key parameters to customize for your deployment:**

| Parameter Path | Default | Description |
|---|---|---|
| `miracle_cnc.state_publisher.publish_rate_hz` | 50.0 | Machine state publish rate |
| `miracle_cnc.gcode_executor.max_spindle_rpm` | 24000.0 | Maximum spindle speed |
| `miracle_cnc.sensor_fusion.fusion_algorithm` | `extended_kalman` | Sensor fusion method |
| `miracle_scada.historian.database_host` | `localhost` | TimescaleDB host |
| `miracle_scada.hmi_bridge.websocket_port` | 9090 | rosbridge WebSocket port |
| `miracle_twin.sync_engine.sync_rate_hz` | 50.0 | Twin synchronization rate |
| `miracle_ai.anomaly_detector.detection_threshold` | 0.95 | Anomaly confidence threshold |
| `miracle_ai.phm_predictor.prediction_horizon_hours` | 168 | PHM lookahead (1 week) |
| `miracle_security.access_enforcer.default_deny` | true | Zero-trust default policy |
| `miracle_resiliency.supervisor_root.max_restarts` | 5 | Max restarts in window |
| `miracle_cognitive.knowledge_graph.graph_backend` | `neo4j` | Knowledge graph database |
| `miracle_cognitive.rl_optimizer.algorithm` | `ppo` | RL algorithm |

### 5.2 QoS Profiles

The file `miracle_bringup/config/qos_profiles.yaml` defines 11 DDS Quality of
Service profiles used across the system:

| Profile | Reliability | Durability | Deadline | Use Case |
|---|---|---|---|---|
| `sensor_data` | best_effort | volatile | 20 ms | High-frequency sensor streams |
| `machine_state` | reliable | transient_local | 50 ms | Machine state updates |
| `command` | reliable | transient_local | 100 ms | G-code, setpoints, mode changes |
| `alarm` | reliable | transient_local | 10 ms | Safety-critical alarms |
| `diagnostic` | reliable | volatile | 1000 ms | Health and diagnostic data |
| `heartbeat` | best_effort | volatile | 1000 ms | Node liveness heartbeats |
| `bulk_data` | reliable | transient_local | 5000 ms | Large payloads (models, images) |
| `event` | reliable | transient_local | 500 ms | Discrete events (job start/stop) |
| `parameter_update` | reliable | transient_local | 1000 ms | Dynamic config changes |
| `knowledge` | reliable | transient_local | 1000 ms | Knowledge graph updates |
| `consensus` | reliable | volatile | 200 ms | Multi-agent consensus messages |
| `security_audit` | reliable | transient_local | 5000 ms | Audit trail messages |

**Example: Using a QoS profile in code:**

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

self.create_subscription(SensorData, 'sensor_data', self.callback, sensor_qos)
```

### 5.3 Per-Machine Configuration

Each CNC machine has a dedicated override file that supersedes defaults from
`miracle_params.yaml`:

| File | Machine | Type | Key Differences |
|---|---|---|---|
| `cnc1.yaml` | CNC1 | 3-axis vertical mill | 800x500x400 mm envelope, 24000 RPM, 8 sensors |
| `cnc2.yaml` | CNC2 | 5-axis horizontal mill | 1000x800x600 mm envelope, 18000 RPM, 12 sensors |
| `cnc3.yaml` | CNC3 | CNC lathe / turning center | 300x0x600 mm envelope, 6000 RPM, 6 sensors |

**Example: cnc2.yaml overrides:**

```yaml
miracle_cnc:
  gcode_executor:
    ros__parameters:
      machine_id: "cnc2"
      max_feed_rate_mm_min: 12000.0
      max_spindle_rpm: 18000.0
      work_envelope_x_mm: 1000.0
      work_envelope_y_mm: 800.0
      work_envelope_z_mm: 600.0

  sensor_fusion:
    ros__parameters:
      machine_id: "cnc2"
      sensor_count: 12   # More sensors than cnc1
```

### 5.4 Environment Variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `42` | DDS domain isolation |
| `ROS_LOG_DIR` | `/miracle_ws/log` | Log output directory |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | (none) | FastDDS XML profile |
| `ROS_SECURITY_KEYSTORE` | (none) | SROS2 keystore path |
| `ROS_SECURITY_ENABLE` | `false` | Enable DDS security |
| `ROS_SECURITY_STRATEGY` | `Permissive` | `Enforce` or `Permissive` |

---

## 6. Package Reference

### 6.1 miracle_msgs

**Purpose:** Defines all custom message, service, and action interfaces for the
MIRACLE system. This is an `ament_cmake` package that must be built before all
other packages.

**Layer:** Cross-cutting (used by all layers)

**Contents:**

- 26 message types (.msg)
- 14 service types (.srv)
- 7 action types (.action)

See [Section 19: API Reference](#19-api-reference) for complete field-level documentation.

### 6.2 miracle_core

**Purpose:** Core utilities, base classes, and shared infrastructure used by all
MIRACLE packages. Provides lifecycle management helpers, parameter loading
utilities, and common data structures.

**Layer:** Cross-cutting

**Nodes:** None (library-only package)

**Dependencies:** `rclpy`, `lifecycle_msgs`, `miracle_msgs`, `std_msgs`

### 6.3 miracle_cnc

**Purpose:** Direct CNC machine control and monitoring. Provides the L1
interface between the ROS 2 bus and physical (or simulated) CNC machines.

**Layer:** L1 -- CNC Machine Control

**Nodes:**

| Node | Role |
|---|---|
| `state_publisher` | Publishes machine state (position, speed, load) at 50 Hz |
| `gcode_executor` | Interprets and executes G-code programs with lookahead |
| `sensor_fusion` | Fuses multi-sensor data streams using Extended Kalman Filter at 100 Hz |
| `local_watchdog` | Per-machine health monitor; triggers E-stop on timeout |
| `spc_monitor` | Statistical Process Control with control charts and Cpk tracking |
| `rosbag_trigger` | Event-driven rosbag recording on alarm/anomaly events |

**Published Topics:**

| Topic | Message Type | QoS | Description |
|---|---|---|---|
| `/miracle/{id}/machine_state` | `MachineState` | machine_state | Full machine state |
| `/miracle/{id}/sensor_data` | `SensorData` | sensor_data | Raw sensor readings |
| `/miracle/{id}/fused_sensor` | `FusedSensorData` | sensor_data | Fused sensor output |
| `/miracle/{id}/gcode_block` | `GCodeBlock` | command | Current G-code line |
| `/miracle/{id}/heartbeat` | `Heartbeat` | heartbeat | Node liveness |

**Subscribed Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/miracle/{id}/raw_sensor/*` | `Float32`/`Float32MultiArray` | micro-ROS sensor data |

**Services:**

| Service | Type | Description |
|---|---|---|
| `/miracle/{id}/trigger_estop` | `TriggerEStop` | Emergency stop |
| `/miracle/{id}/validate_gcode` | `ValidateGCode` | Pre-validate G-code program |

**Actions:**

| Action | Type | Description |
|---|---|---|
| `/miracle/{id}/execute_program` | `ExecuteProgram` | Execute a G-code program |
| `/miracle/{id}/perform_calibration` | `PerformCalibration` | Run machine calibration |

**Key Parameters:**

```yaml
publish_rate_hz: 50.0            # State publish rate
max_feed_rate_mm_min: 10000.0    # Maximum feed rate
max_spindle_rpm: 24000.0         # Maximum spindle speed
fusion_algorithm: "extended_kalman"  # EKF, UKF, or particle
heartbeat_interval_sec: 1.0      # Watchdog heartbeat period
cpk_minimum: 1.33               # Minimum process capability
```

### 6.4 miracle_scada

**Purpose:** SCADA/Supervisory layer providing device discovery, network traffic
management, alarm handling, time-series data archival, and HMI bridging.

**Layer:** L2 -- SCADA / Supervisory

**Nodes:**

| Node | Role |
|---|---|
| `discovery_server` | Discovers and registers CNC machines via multicast |
| `traffic_manager` | Enforces QoS policies and rate limiting |
| `alarm_manager` | Aggregates, persists, and escalates alarms |
| `historian` | Archives time-series data to TimescaleDB |
| `hmi_bridge` | WebSocket bridge (rosbridge) for web dashboard |

**Published Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/miracle/scada/alarms` | `AnomalyAlert` | Aggregated alarm feed |
| `/miracle/fleet_manager/fleet_health` | `FleetHealth` | Fleet health summary |

**Services:**

| Service | Type | Description |
|---|---|---|
| `/miracle/scada/register_device` | `RegisterDevice` | Register a new device |
| `/miracle/fleet_manager/get_fleet_status` | `GetFleetStatus` | Query fleet status |

**Key Parameters:**

```yaml
multicast_group: "239.255.0.1"      # Discovery multicast
historian.database_type: "timescaledb"
historian.retention_days: 365
hmi_bridge.websocket_port: 9090
hmi_bridge.max_clients: 50
```

### 6.5 miracle_bridges

**Purpose:** Protocol bridge nodes that translate between ROS 2 and standard
industrial communication protocols.

**Layer:** L2 -- SCADA / Supervisory

**Nodes:**

| Node | Protocol | Transport |
|---|---|---|
| `opc_ua_bridge` | OPC-UA | TCP (SignAndEncrypt, Basic256Sha256) |
| `sparkplug_bridge` | Sparkplug B | MQTT over TLS |
| `mtconnect_agent` | MTConnect | HTTP/XML (SHDR adapter) |
| `modbus_bridge` | Modbus TCP | TCP (register-mapped) |
| `kafka_bridge` | Apache Kafka | TCP (Snappy compression) |

**Key Parameters (opc_ua_bridge):**

```yaml
server_url: "opc.tcp://localhost:4840"
security_mode: "SignAndEncrypt"
security_policy: "Basic256Sha256"
polling_interval_ms: 100
```

**Key Parameters (kafka_bridge):**

```yaml
bootstrap_servers: "localhost:9092"
producer_topic_prefix: "miracle.ros"
consumer_topic_prefix: "miracle.cmd"
compression: "snappy"
schema_registry_url: "http://localhost:8081"
```

### 6.6 miracle_twin

**Purpose:** Digital twin synchronization, Gazebo simulation bridging,
predictive scenario execution, and drift detection.

**Layer:** L3 -- MES / Digital Twin / AI

**Nodes:**

| Node | Role |
|---|---|
| `sync_engine` | Synchronizes physical machine state to digital twin at 50 Hz |
| `gazebo_bridge` | Bridges between MIRACLE topics and Gazebo simulation |
| `prediction_runner` | Runs physics-informed neural network predictions (5 min horizon) |
| `scenario_manager` | Manages concurrent what-if simulation scenarios |

**Published Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/miracle/twin/sync_state` | `MachineState` | Synchronized twin state |
| `/miracle/twin/prediction` | `PHMPrediction` | Predictive results |

**Actions:**

| Action | Type | Description |
|---|---|---|
| `/miracle/twin/run_prediction` | `RunPrediction` | Execute a prediction run |

**Key Parameters:**

```yaml
sync_rate_hz: 50.0
max_sync_latency_ms: 20.0
drift_correction_enabled: true
prediction_horizon_sec: 300.0
monte_carlo_samples: 100
max_concurrent_scenarios: 5
```

### 6.7 miracle_mes

**Purpose:** Manufacturing Execution System providing job scheduling, fleet
management, digital thread traceability, resource management, and OEE calculation.

**Layer:** L3 -- MES / Digital Twin / AI

**Nodes:**

| Node | Role |
|---|---|
| `job_scheduler` | Priority-weighted shortest-job scheduling with preemption |
| `fleet_manager` | Weighted round-robin load balancing across machines |
| `digital_thread` | SHA-256 hash-linked traceability chain |
| `resource_manager` | Tool inventory and material tracking |
| `oee_calculator` | OEE = Availability x Performance x Quality |

**Published Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/miracle/mes/job_status` | `JobStatus` | Job progress updates |
| `/miracle/system_kpis` | `SystemKPIs` | OEE and system metrics |
| `/miracle/mes/digital_thread` | `DigitalThreadEntry` | Traceability entries |

**Services:**

| Service | Type | Description |
|---|---|---|
| `/miracle/mes/submit_task` | `SubmitTask` | Submit a manufacturing task |

**Actions:**

| Action | Type | Description |
|---|---|---|
| `/miracle/mes/execute_job` | `ExecuteJob` | Execute a full manufacturing job |

**Key Parameters:**

```yaml
scheduling_algorithm: "priority_weighted_shortest_job"
planning_horizon_min: 480           # 8-hour planning window
load_balancing_strategy: "weighted_round_robin"
target_oee: 0.85
target_availability: 0.90
target_performance: 0.95
target_quality: 0.99
```

### 6.8 miracle_ai

**Purpose:** AI/ML inference nodes for anomaly detection, predictive health
management, tool wear estimation, chatter detection, and model lifecycle
management.

**Layer:** L3 -- MES / Digital Twin / AI

**Nodes:**

| Node | Role |
|---|---|
| `anomaly_detector` | Isolation forest + autoencoder ensemble |
| `phm_predictor` | Remaining useful life prediction (168-hour horizon) |
| `tool_wear_estimator` | Real-time flank and crater wear estimation |
| `chatter_detector` | FFT-based chatter detection with spindle speed variation suppression |
| `model_manager` | ML model versioning, validation, and deployment with auto-rollback |

**Published Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/miracle/{id}/anomaly` | `AnomalyAlert` | Detected anomalies |
| `/miracle/{id}/phm_prediction` | `PHMPrediction` | Health predictions |
| `/miracle/{id}/tool_wear` | `ToolWearEstimate` | Tool wear status |

**Key Parameters:**

```yaml
anomaly_detector:
  algorithm: "isolation_forest_autoencoder"
  detection_threshold: 0.95
  window_size_samples: 256
  retrain_interval_hours: 24

phm_predictor:
  prediction_horizon_hours: 168
  failure_modes:
    - "bearing_degradation"
    - "spindle_wear"
    - "axis_backlash"
    - "thermal_drift"

chatter_detector:
  fft_size: 2048
  sampling_rate_hz: 10000.0
  suppression_strategy: "spindle_speed_variation"
```

### 6.9 miracle_security

**Purpose:** Cybersecurity layer providing intrusion detection, device attestation,
automated threat response, access control, audit logging, and SROS2 key management.

**Layer:** L4 -- Security / Resiliency

**Nodes:**

| Node | Role |
|---|---|
| `intrusion_detection` | Hybrid signature + anomaly-based IDS |
| `attestation_verifier` | Device firmware/config integrity verification |
| `threat_response` | Automated isolation and escalation |
| `access_enforcer` | RBAC with default-deny policy |
| `audit_logger` | Tamper-proof hash-chain audit log |
| `sros2_manager` | DDS security key management and rotation |

**Published Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/miracle/security/alerts` | `SecurityAlert` | Security alerts |
| `/miracle/security/trust_status` | `DeviceTrustStatus` | Device trust scores |
| `/miracle/security/attestation` | `AttestationReport` | Attestation results |

**Services:**

| Service | Type | Description |
|---|---|---|
| `/miracle/security/request_attestation` | `RequestAttestation` | Request device attestation |

**Actions:**

| Action | Type | Description |
|---|---|---|
| `/miracle/security/isolate_node` | `IsolateNode` | Isolate a compromised node |

**Key Parameters:**

```yaml
intrusion_detection:
  detection_mode: "hybrid"
  alert_threshold: 0.85

access_enforcer:
  rbac_enabled: true
  default_deny: true
  max_failed_attempts: 5
  lockout_duration_sec: 300.0

audit_logger:
  tamper_detection: true
  hash_chain_enabled: true
  log_retention_days: 90
```

### 6.10 miracle_resiliency

**Purpose:** Fault tolerance layer implementing supervision trees, heartbeat
monitoring, failover coordination, state checkpointing, recovery orchestration,
and chaos engineering.

**Layer:** L4 -- Security / Resiliency

**Nodes:**

| Node | Role |
|---|---|
| `supervisor_root` | Root supervisor with one-for-one restart strategy |
| `heartbeat_aggregator` | Aggregates heartbeats into fleet health score |
| `failover_coordinator` | Hot-standby failover with fencing-based split-brain resolution |
| `checkpoint_manager` | Periodic state checkpointing with zstd compression |
| `recovery_orchestrator` | Progressive recovery with state verification |
| `chaos_injector` | Fault injection for resilience testing |

**Published Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/miracle/resiliency/fleet_health` | `FleetHealth` | System health score |
| `/miracle/resiliency/node_failure` | `NodeFailure` | Node failure events |
| `/miracle/resiliency/recovery_request` | `RecoveryRequest` | Recovery requests |
| `/miracle/heartbeats` | `Heartbeat` | Aggregated heartbeats |

**Services:**

| Service | Type | Description |
|---|---|---|
| `/miracle/resiliency/trigger_failover` | `TriggerFailover` | Trigger manual failover |
| `/miracle/resiliency/restore_checkpoint` | `RestoreCheckpoint` | Restore from checkpoint |
| `/miracle/resiliency/inject_fault` | `InjectFault` | Inject a test fault |

**Key Parameters:**

```yaml
supervisor_root:
  supervision_strategy: "one_for_one"
  max_restarts: 5
  restart_window_sec: 60.0

failover_coordinator:
  failover_strategy: "hot_standby"
  quorum_size: 2
  split_brain_resolution: "fencing"

chaos_injector:
  enabled: false  # Enable only for testing
  fault_types:
    - "node_crash"
    - "message_delay"
    - "message_drop"
    - "cpu_stress"
```

### 6.11 miracle_cognitive

**Purpose:** Level 5 cognitive autonomy providing knowledge management,
planning, multi-agent coordination, learning, self-X properties, and
human interface capabilities.

**Layer:** L5 -- Cognitive / Autonomous

**Nodes (22 total):**

**Knowledge Management:**

| Node | Role |
|---|---|
| `knowledge_graph` | Neo4j-backed knowledge graph (100K nodes, 500K relationships) |
| `ontology_manager` | OWL manufacturing ontology with HermiT reasoner |
| `reasoning_engine` | Hybrid reasoning with configurable inference depth |
| `causal_inference` | Structural equation model-based causal analysis |

**Planning:**

| Node | Role |
|---|---|
| `goal_manager` | Multi-goal management with priority-based conflict resolution |
| `behavior_tree_executor` | BT execution at 30 Hz tick rate |
| `htn_planner` | Hierarchical Task Network planning with caching |
| `goap_planner` | Goal-Oriented Action Planning with relaxed-plan heuristic |

**Multi-Agent Coordination:**

| Node | Role |
|---|---|
| `agent_registry` | Agent registration with capability matching |
| `task_allocator` | Contract-net task allocation |
| `auction_manager` | Combinatorial auction management |
| `coalition_former` | Shapley-value coalition formation |
| `consensus_protocol` | Raft consensus with leader election |

**Learning:**

| Node | Role |
|---|---|
| `rl_optimizer` | PPO reinforcement learning optimizer |
| `federated_coordinator` | Federated averaging with differential privacy |
| `federated_client` | Per-machine federated learning client |

**Self-X Properties:**

| Node | Role |
|---|---|
| `self_optimizer` | Pareto multi-objective optimization (throughput, energy, quality) |
| `self_healer` | Automated diagnosis and healing (restart, reconfigure, migrate) |
| `self_configurer` | Dynamic configuration adaptation |
| `self_protector` | Autonomous security threat response |

**Human Interface:**

| Node | Role |
|---|---|
| `nlp_interface` | Natural language command interpretation |
| `explanation_generator` | Human-readable decision explanations |
| `human_escalation` | Multi-channel escalation (dashboard, email, SMS) |

**Published Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/miracle/cognitive/knowledge_events` | `KnowledgeUpdate` | Knowledge graph changes |
| `/miracle/cognitive/task_announcements` | `TaskAnnouncement` | Task auction announcements |
| `/miracle/cognitive/bids` | `AgentBid` | Agent auction bids |
| `/miracle/cognitive/task_awards` | `TaskAward` | Task award notifications |
| `/miracle/cognitive/bt_status` | `BehaviorTreeStatus` | BT execution status |
| `/miracle/cognitive/global_model` | `FederatedModel` | Global federated model |
| `/miracle/cognitive/model_updates` | `ModelUpdate` | Local model updates |
| `/miracle/cognitive/optimization` | `OptimizationAction` | Optimization actions |
| `/miracle/cognitive/experience` | `ManufacturingExperience` | Learned experiences |

**Services:**

| Service | Type | Description |
|---|---|---|
| `/miracle/cognitive/sparql_query` | `SPARQLQuery` | Query the knowledge graph |
| `/miracle/cognitive/submit_task` | `SubmitTask` | Submit task for allocation |
| `/miracle/cognitive/optimize` | `OptimizeParameters` | Request parameter optimization |
| `/miracle/cognitive/goap_plan` | `GOAPPlan` | Generate a GOAP plan |
| `/miracle/cognitive/htn_plan` | `HTNPlan` | Generate an HTN plan |
| `/miracle/cognitive/nlp_command` | `NLPCommand` | Natural language command |

**Actions:**

| Action | Type | Description |
|---|---|---|
| `/miracle/cognitive/train_rl` | `TrainRLPolicy` | Train an RL policy |
| `/miracle/cognitive/federated_round` | `FederatedRound` | Execute a federated learning round |

**Behavior Tree Files:**

The package ships with three XML behavior tree definitions:

- `autonomous_job.xml` -- Full autonomous job execution sequence
- `anomaly_response.xml` -- Anomaly detection and response tree
- `recovery_sequence.xml` -- System recovery behavior tree

### 6.12 miracle_gazebo

**Purpose:** Gazebo simulation assets including world files, model definitions
(CNC machine, cobot, sensor array), and launch files for the simulation
environment.

**Layer:** Simulation

**Models:**

- `cnc_machine` -- CNC milling machine SDF model
- `cobot` -- Collaborative robot for material handling
- `sensor_array` -- Multi-sensor array model

**No executable nodes** -- this is an asset-only package.

### 6.13 miracle_bringup

**Purpose:** Central launch files and configuration for bringing up the
entire MIRACLE system or individual layers.

**Layer:** Infrastructure

**Launch Files:** See [Section 7: Launch Files](#7-launch-files)

**Config Files:** See [Section 5: Configuration](#5-configuration)

### 6.14 miracle_dashboard

**Purpose:** React-based web dashboard for monitoring and controlling the
MIRACLE system. Communicates with ROS 2 via rosbridge WebSocket.

**Layer:** UI

**Technology Stack:**

- React 18
- Material-UI (MUI) 5
- Recharts (data visualization)
- roslib.js (ROS 2 WebSocket client)

See [Section 8: Dashboard](#8-dashboard) for details.

### 6.15 miracle_microros

**Purpose:** Firmware configurations and transport setup for micro-ROS MCU bridges
(ESP32, STM32). Provides sensor acquisition firmware, transport layer configuration,
and micro-ROS agent integration.

**Layer:** L1 -- CNC Machine Control

See [Section 15: micro-ROS Firmware](#15-micro-ros-firmware) for details.

### 6.16 miracle_unity_bridge

**Purpose:** ROS-TCP-Endpoint bridge for Unity Digital Twin 3D visualization.
Provides the server-side TCP endpoint that connects the Unity application
(via ROS-TCP-Connector) to the ROS 2 DDS bus, enabling real-time streaming
of joint states, cutting forces, tool wear, thermal data, and sensor fusion
outputs to the Unity 3D rendering engine.

**Layer:** UI / Digital Twin

**Package structure:**

```
miracle_unity_bridge/
  miracle_unity_bridge/
    __init__.py
  config/
    unity_params.yaml          # TCP endpoint and topic configuration
  launch/
    unity_bridge.launch.py     # Launches ROS-TCP-Endpoint node
  package.xml
  setup.py
  setup.cfg
```

**Launch file:** `unity_bridge.launch.py`

Starts the `ros_tcp_endpoint` node configured to listen on TCP port 10000.
The Unity application connects to this port via the ROS-TCP-Connector plugin
and subscribes/publishes to standard ROS 2 topics.

**Configuration:** `unity_params.yaml`

```yaml
ros_tcp_endpoint:
  ros__parameters:
    tcp_ip: "0.0.0.0"
    tcp_port: 10000
    ROS_IP: "0.0.0.0"
```

**How it works:**

1. The `ros_tcp_endpoint` node opens a TCP server on port 10000.
2. Unity (with the ROS-TCP-Connector package) connects as a TCP client.
3. ROS 2 topics are serialized and forwarded over TCP to Unity.
4. Unity publishes commands back through the same TCP channel.
5. The bridge supports standard message types (`JointState`, `WrenchStamped`,
   `Float64MultiArray`, custom `miracle_msgs`) with no DDS dependency in Unity.

---

## 7. Launch Files

### 7.1 Full System Launch

Brings up all layers L1 through L5:

```bash
ros2 launch miracle_bringup miracle_full_system.launch.py
```

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `machine_count` | `3` | Number of CNC machines to launch |
| `simulation_mode` | `true` | Simulated (`true`) or physical (`false`) |
| `security_enabled` | `true` | Enable L4 security/resiliency layer |
| `cognitive_enabled` | `true` | Enable L5 cognitive layer |

**Examples:**

```bash
# Full system, 3 machines, simulation mode, all layers
ros2 launch miracle_bringup miracle_full_system.launch.py

# 2 machines, physical mode, security disabled
ros2 launch miracle_bringup miracle_full_system.launch.py \
    machine_count:=2 \
    simulation_mode:=false \
    security_enabled:=false

# Simulation with cognitive layer disabled
ros2 launch miracle_bringup miracle_full_system.launch.py \
    cognitive_enabled:=false
```

### 7.2 Simulation-Only Launch

Launches CNC machines in simulation mode with `use_sim_time=true`, plus SCADA,
MES, digital twin, and AI layers (no security/cognitive):

```bash
ros2 launch miracle_bringup miracle_simulation.launch.py
```

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `machine_count` | `3` | Number of simulated CNC machines |

> **Note:** This launch also starts the Unity Digital Twin bridge
> (`miracle_unity_bridge`) for 3D visualization via ROS-TCP-Connector on
> TCP port 10000. Open the Unity project and press Play to connect.

### 7.3 Physical Deployment Launch

Same as simulation but with `use_sim_time=false` and includes all protocol
bridge nodes for real hardware communication:

```bash
ros2 launch miracle_bringup miracle_physical.launch.py
```

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `machine_count` | `3` | Number of physical CNC machines |

> **Note:** This launch also starts the Unity Digital Twin bridge
> (`miracle_unity_bridge`) for 3D visualization via ROS-TCP-Connector on
> TCP port 10000. Open the Unity project and press Play to connect.

### 7.4 Individual Layer Launch

**Security and Resiliency Layer (L4):**

```bash
ros2 launch miracle_bringup security_layer.launch.py
```

Launches all 6 security nodes under `/miracle/security/` and all 6 resiliency
nodes under `/miracle/resiliency/`. All nodes use `respawn=True` for automatic
restart on failure.

**Cognitive Layer (L5):**

```bash
ros2 launch miracle_bringup cognitive_layer.launch.py
```

Launches all 22 cognitive nodes under `/miracle/cognitive/` with `respawn=True`
and a 5-second respawn delay.

### 7.5 Per-Machine Launch

Launch a single CNC machine's node stack:

```bash
ros2 launch miracle_bringup cnc_machine.launch.py \
    machine_id:=cnc1 \
    simulation_mode:=true
```

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `machine_id` | (required) | Machine identifier (e.g., `cnc1`) |
| `params_file` | `miracle_params.yaml` | Parameter file path |
| `simulation_mode` | `true` | Simulation or physical mode |

This launches 6 LifecycleNodes under `/miracle/{machine_id}/`:
`state_publisher`, `gcode_executor`, `sensor_fusion`, `local_watchdog`,
`spc_monitor`, `rosbag_trigger`. All nodes have `respawn=True` with a
2-second delay.

### 7.6 Unity Bridge Launch

Launch the Unity Digital Twin bridge independently:

```bash
ros2 launch miracle_unity_bridge unity_bridge.launch.py
```

This starts the `ros_tcp_endpoint` node on TCP port 10000, allowing the
Unity application to connect and receive real-time ROS 2 topic data. Use
this when you want to connect Unity to an already-running ROS 2 system
without restarting the full simulation or physical launch.

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `tcp_ip` | `0.0.0.0` | IP address to bind the TCP endpoint |
| `tcp_port` | `10000` | TCP port for Unity connection |

---

## 8. Dashboard

### 8.1 Starting the Dashboard

**Development mode:**

```bash
cd miracle_ws/src/miracle_dashboard
npm install
npm start
```

The dashboard starts at `http://localhost:3000`.

**Docker mode:**

```bash
docker compose -f miracle_ws/docker/docker-compose.yaml up dashboard
```

The dashboard is served via nginx at `http://localhost:3000`.

**Prerequisites:**

The dashboard requires the rosbridge WebSocket server running on port 9090
(provided by the `hmi_bridge` node in `miracle_scada`).

### 8.2 Dashboard Views

The dashboard provides seven views accessible from the left sidebar:

| View | Key | Description |
|---|---|---|
| **Overview** | `overview` | Fleet-wide summary: machine statuses, active alarms, KPIs |
| **Machines** | `machines` | Per-machine detail: state, position, spindle, sensors |
| **Alerts** | `alerts` | Active alarm list with severity filtering |
| **Digital Twin** | `digital-twin` | Twin synchronization status and drift visualization |
| **AI / ML** | `ai-ml` | Anomaly detection, predictive maintenance, federated learning |
| **Security** | `security` | Zero-trust access control, IDS alerts, audit log viewer |
| **OEE** | `oee` | OEE breakdown: Availability, Performance, Quality charts |

The header bar shows a real-time ROS connection status indicator (green = connected,
red = disconnected) with automatic reconnection every 3 seconds.

### 8.3 ROS Bridge Connection

The dashboard connects to ROS 2 via roslib.js over WebSocket:

```javascript
// Default connection URL
const ROSBRIDGE_URL = 'ws://localhost:9090';

// Topic subscription example
subscribe(
  '/miracle/cnc1/machine_state',
  'miracle_msgs/msg/MachineState',
  (msg) => { /* handle message */ }
);

// Service call example
callService(
  '/miracle/mes/submit_task',
  'miracle_msgs/srv/SubmitTask',
  { task_type: 'milling', job_id: 'JOB-001', material: 'aluminum' }
);
```

**Monitored Topics:**

The dashboard subscribes to the following topics (defined in `rosTopics.js`):

| Topic Pattern | Message Type | Description |
|---|---|---|
| `/miracle/{id}/state` | `MachineState` | Per-machine state |
| `/miracle/{id}/sensor_data` | `SensorData` | Raw sensor data |
| `/miracle/{id}/anomaly` | `AnomalyAlert` | Anomaly alerts |
| `/miracle/{id}/job_status` | `JobStatus` | Job progress |
| `/miracle/{id}/tool_wear` | `ToolWearEstimate` | Tool wear |
| `/miracle/{id}/phm_prediction` | `PHMPrediction` | Health predictions |
| `/miracle/system_kpis` | `SystemKPIs` | OEE and KPIs |
| `/miracle/fleet_manager/fleet_health` | `FleetHealth` | Fleet health |
| `/miracle/security/alerts` | `SecurityAlert` | Security alerts |
| `/miracle/heartbeats` | `Heartbeat` | Node heartbeats |

### 8.4 Unity Digital Twin 3D Visualization

In addition to the React web dashboard, MIRACLE provides a high-fidelity 3D
digital twin built in **Unity 6 LTS** (Universal Render Pipeline). The Unity
application connects to the ROS 2 stack via `miracle_unity_bridge` (TCP port
10000) and provides:

- **Real-time 3D rendering** of the complete manufacturing cell (Bantam Tools
  Explorer CNC, Niryo Ned2 cobot, xArm 6 Lite)
- **GPU-accelerated voxel material removal** with marching cubes mesh extraction
- **Altintas mechanistic cutting force visualization** (shearing + edge forces)
- **Thermal field and tool wear overlays** via custom URP shaders
- **VFX Graph chip/coolant particle effects** driven by physics data
- **Interactive camera controls** with orbit, pan, zoom, and preset viewpoints

To connect the Unity visualization to a running ROS 2 system:

1. Build and source the MIRACLE workspace
2. Launch the simulation: `ros2 launch miracle_bringup miracle_simulation.launch.py`
3. Open the Unity project in Unity 6 LTS and press Play

The Unity application automatically connects to the `ros_tcp_endpoint` on
`localhost:10000`. See [Section 20: Unity Digital Twin](#20-unity-digital-twin)
and the dedicated `docs/UNITY_TWIN_MANUAL.md` for full documentation.

---

## 9. Digital Twin

### 9.1 Sync Engine

The sync engine (`miracle_twin.sync_engine`) maintains a real-time digital
replica of each physical CNC machine. It operates at 50 Hz and uses
interpolation to smooth state updates:

```
Physical Machine --> MachineState (50 Hz) --> Sync Engine --> Gazebo Bridge --> Gazebo
                                                  |
                                                  +--> Prediction Runner
                                                  |
                                                  +--> Scenario Manager
```

**Key behaviors:**

- Buffers up to 1000 state samples for interpolation
- Detects latency exceeding 20 ms and logs warnings
- Applies drift correction when enabled

### 9.2 Drift Detection and Correction

The sync engine continuously compares physical machine state against the digital
twin model. When drift exceeds a configured threshold:

1. The deviation is logged with contributing factors
2. A `KnowledgeUpdate` is published to the cognitive layer
3. If `drift_correction_enabled: true`, the twin state is corrected
4. Large drifts trigger a recalibration request

### 9.3 Prediction Scenarios

The `prediction_runner` uses Physics-Informed Neural Networks (PINNs) to
forecast machine behavior 5 minutes ahead:

```bash
# Trigger a prediction via ROS 2 action
ros2 action send_goal /miracle/twin/run_prediction \
    miracle_msgs/action/RunPrediction \
    "{machine_id: 'cnc1', prediction_type: 'thermal_drift', prediction_horizon_hours: 1.0}"
```

The `scenario_manager` can run up to 5 concurrent what-if scenarios with a
600-second timeout. Results are auto-saved to `/data/scenarios`.

### 9.4 Unity 3D Digital Twin

The MIRACLE system includes a Unity-based 3D digital twin that complements
the Gazebo simulation with high-fidelity visualization of the manufacturing
process. While the Gazebo-based twin in `miracle_twin` focuses on physics
simulation and state synchronization, the Unity twin provides an immersive
3D rendering environment optimized for operator visualization and analysis.

**Connection Architecture:**

```
ROS 2 DDS Bus
    |
    +-- ros_tcp_endpoint (miracle_unity_bridge, TCP port 10000)
            |
            +-- [TCP/IP] --> ROS-TCP-Connector (Unity plugin)
                                |
                                +-- Unity 6 LTS Application (URP)
                                      |
                                      +-- Voxel Material Removal Engine
                                      +-- Altintas Force Model Visualization
                                      +-- Thermal / Wear Simulation Overlay
                                      +-- VFX Graph Chip Particles
                                      +-- Manufacturing Cell Scene
```

**Features:**

- **Voxel Material Removal:** GPU compute shader-based voxel grid (configurable
  resolution up to 200x200x200) with marching cubes isosurface extraction for
  real-time visualization of CNC material removal during milling operations.
- **Altintas Mechanistic Cutting Force Model:** Implements shearing and edge
  force coefficients for tangential, radial, and axial force computation.
  Forces are visualized as 3D vector arrows and color-mapped overlays.
- **Thermal Simulation:** Temperature field visualization driven by ROS 2
  thermal sensor data, rendered as color gradient overlays on the workpiece.
- **Tool Wear Visualization:** Real-time flank wear (VB) and crater wear (KT)
  visualization on the cutting tool, fed by `miracle_ai.tool_wear_estimator`.
- **Manufacturing Cell:** Complete 3D scene including:
  - Bantam Tools Explorer CNC desktop milling machine
  - Niryo Ned2 collaborative robot (pick-and-place)
  - xArm 6 Lite robotic arm (machine tending)
  - Workpiece fixtures, tool holders, and shop environment

**How to Launch:**

```bash
# 1. Build and source the workspace
cd miracle_ws
colcon build --symlink-install
source install/setup.bash

# 2. Launch the simulation (includes Unity bridge)
ros2 launch miracle_bringup miracle_simulation.launch.py

# 3. Open the Unity project in Unity 6 LTS and press Play
#    Unity connects automatically to localhost:10000
```

For complete Unity Digital Twin documentation including shader configuration,
voxel resolution tuning, and custom visualization setup, see
`docs/UNITY_TWIN_MANUAL.md`.

---

## 10. AI/ML Pipeline

### 10.1 Anomaly Detection Ensemble

The `anomaly_detector` uses a hybrid Isolation Forest + Autoencoder ensemble:

```
Fused Sensor Data (256 samples, 32 features)
    |
    +--> Isolation Forest --> Anomaly Score 1
    |
    +--> Autoencoder --> Reconstruction Error --> Anomaly Score 2
    |
    +--> Ensemble Fusion --> Confidence > 0.95?
              |
              +--> YES: Publish AnomalyAlert
              |         (type, severity, contributing_factors, recommended_action)
              |
              +--> NO: Continue monitoring
```

The model retrains every 24 hours from the most recent 10,000+ samples.

### 10.2 PHM Prediction

The `phm_predictor` monitors four failure modes with a 168-hour (1 week)
prediction horizon:

| Failure Mode | Indicators | Typical RUL Range |
|---|---|---|
| Bearing degradation | Vibration envelope, temperature trend | 100-500 hours |
| Spindle wear | Load profile, runout measurements | 200-1000 hours |
| Axis backlash | Position error, reversal spikes | 300-800 hours |
| Thermal drift | Temperature gradient, dimensional error | Continuous |

### 10.3 Tool Wear Estimation

The `tool_wear_estimator` tracks both flank wear (VB) and crater wear (KT)
in real time:

```yaml
# Alert thresholds
remaining_life_warning_pct: 20.0   # Alert at 20% remaining life
force_threshold_n: 500.0           # Force-based wear indicator
vibration_threshold_g: 2.5         # Vibration-based wear indicator
```

The estimator publishes `ToolWearEstimate` messages every 10 seconds with
the wear percentage, remaining life in minutes, wear type classification,
and recommended action (continue, plan replacement, immediate replacement).

### 10.4 Chatter Detection

The `chatter_detector` performs FFT-based analysis on vibration data:

```
Vibration Signal (10 kHz) --> 2048-point FFT --> Band Analysis
    |
    +--> [500-2000 Hz]  Low-frequency chatter
    +--> [2000-5000 Hz] Mid-frequency chatter
    +--> [5000-10000 Hz] High-frequency chatter
    |
    +--> Chatter Detected? --> Spindle Speed Variation suppression
```

The default suppression strategy is `spindle_speed_variation`, which
modulates the spindle RPM to break the regenerative chatter feedback loop.

### 10.5 Federated Learning

MIRACLE uses federated learning to train shared models across multiple
machines without centralizing sensitive production data:

```
Machine 1 (Local Training) --+
Machine 2 (Local Training) --+--> Federated Coordinator
Machine 3 (Local Training) --+    (FedAvg + Differential Privacy)
                                         |
                                         v
                                   Global Model
                                   (Published to all)
```

**Key parameters:**

```yaml
federated_coordinator:
  aggregation_strategy: "federated_averaging"
  min_participants: 2
  privacy_budget_epsilon: 1.0    # Differential privacy
  secure_aggregation: true
```

---

## 11. Security

### 11.1 SROS2 Setup

Generate the SROS2 keystore and per-node keys:

```bash
bash scripts/generate_security.sh
```

This script:

1. Creates an SROS2 keystore at `<workspace>/security/keystore`
2. Generates keys for all 30+ MIRACLE nodes
3. Writes `security/security_env.bash` with environment variables

**Enable DDS security:**

```bash
source security/security_env.bash
# Sets:
#   ROS_SECURITY_KEYSTORE=<path>
#   ROS_SECURITY_ENABLE=true
#   ROS_SECURITY_STRATEGY=Enforce
```

### 11.2 RBAC Roles

The `access_enforcer` implements role-based access control:

| Role | Permissions |
|---|---|
| `operator` | Read machine state, start/stop jobs, acknowledge alarms |
| `engineer` | All operator permissions + modify parameters, run calibration |
| `admin` | All permissions + security config, user management |
| `readonly` | Read-only access to all topics |

**Policy configuration:**

```yaml
access_enforcer:
  rbac_enabled: true
  default_deny: true
  session_timeout_sec: 3600.0
  max_failed_attempts: 5
  lockout_duration_sec: 300.0
```

### 11.3 Intrusion Detection

The `intrusion_detection` node runs in hybrid mode:

- **Signature-based:** Matches network patterns against a known attack database
- **Anomaly-based:** Builds a 72-hour behavioral baseline and alerts on deviations

```yaml
intrusion_detection:
  detection_mode: "hybrid"
  alert_threshold: 0.85
  scan_interval_sec: 1.0
  monitored_interfaces:
    - "eth0"
    - "can0"
```

### 11.4 Attestation

Device integrity is verified through the `attestation_verifier`:

```bash
# Request attestation via service call
ros2 service call /miracle/security/request_attestation \
    miracle_msgs/srv/RequestAttestation \
    "{device_id: 'cnc1', challenge_nonce: 'abc123'}"
```

The attestation report includes firmware hash, configuration hash, integrity
status, violations list, and trust score. TPM support can be enabled for
hardware-backed attestation.

### 11.5 Audit Logging

The `audit_logger` creates a tamper-proof audit trail:

- SHA-256 hash chain links each entry to the previous
- Logs rotate at 100 MB with 90-day retention
- Tamper detection verifies hash chain integrity
- Stored at `/data/security/audit/`

---

## 12. Resiliency

### 12.1 Supervision Trees

The `supervisor_root` implements an Erlang-inspired one-for-one supervision
strategy:

```
supervisor_root
    |
    +-- /miracle/cnc1/*      (6 nodes)
    +-- /miracle/cnc2/*      (6 nodes)
    +-- /miracle/cnc3/*      (6 nodes)
    +-- /miracle/scada/*     (5 nodes)
    +-- /miracle/mes/*       (5 nodes)
```

**Restart policy:** Up to 5 restarts within a 60-second window. If the limit is
exceeded, the supervisor escalates to the cognitive layer's self-healer.

### 12.2 Failover Coordination

The `failover_coordinator` provides hot-standby failover:

```yaml
failover_strategy: "hot_standby"
detection_timeout_sec: 3.0        # Detect failure in 3 seconds
switchover_max_time_sec: 5.0      # Complete switchover in 5 seconds
quorum_size: 2                    # Minimum nodes for quorum
split_brain_resolution: "fencing" # Fencing-based split-brain resolution
```

**Manual failover:**

```bash
ros2 service call /miracle/resiliency/trigger_failover \
    miracle_msgs/srv/TriggerFailover \
    "{failed_node: 'cnc1/gcode_executor', strategy: 'hot_standby'}"
```

### 12.3 Checkpointing

The `checkpoint_manager` creates periodic state snapshots:

```yaml
checkpoint_interval_sec: 60.0
checkpoint_path: "/data/checkpoints"
max_checkpoints: 10               # Rolling window
compression_enabled: true
async_checkpointing: true
state_topics:
  - "/miracle/*/machine_state"
  - "/miracle/mes/job_queue"
```

**Restore from checkpoint:**

```bash
ros2 service call /miracle/resiliency/restore_checkpoint \
    miracle_msgs/srv/RestoreCheckpoint \
    "{node_name: 'mes/job_scheduler', checkpoint_id: 'latest'}"
```

### 12.4 Chaos Injection

The `chaos_injector` enables controlled fault injection for resilience testing:

```yaml
chaos_injector:
  enabled: false                  # MUST be explicitly enabled
  injection_probability: 0.01
  fault_types:
    - "node_crash"
    - "message_delay"
    - "message_drop"
    - "cpu_stress"
  max_fault_duration_sec: 30.0
  excluded_nodes:                 # Never fault these
    - "supervisor_root"
    - "access_enforcer"
```

**Inject a specific fault:**

```bash
ros2 service call /miracle/resiliency/inject_fault \
    miracle_msgs/srv/InjectFault \
    "{target_node: 'cnc1/sensor_fusion', fault_type: 'message_delay', \
      duration_sec: 10.0, intensity: 0.5}"
```

---

## 13. Cognitive Layer

### 13.1 Knowledge Graph

The `knowledge_graph` node maintains a manufacturing knowledge graph in Neo4j:

```yaml
graph_backend: "neo4j"
graph_host: "localhost"
graph_port: 7687
max_nodes: 100000
max_relationships: 500000
inference_enabled: true
```

**Query the knowledge graph:**

```bash
ros2 service call /miracle/cognitive/sparql_query \
    miracle_msgs/srv/SPARQLQuery \
    "{query: 'SELECT ?m WHERE { ?m rdf:type :CNCMachine }', graph_name: 'manufacturing'}"
```

### 13.2 Reasoning Engine

The `reasoning_engine` provides hybrid deductive/abductive reasoning:

```yaml
reasoning_mode: "hybrid"
max_inference_depth: 10
timeout_sec: 5.0
confidence_threshold: 0.70
explanation_enabled: true         # Generate human-readable explanations
```

The `causal_inference` node uses structural equation models with Granger
causality to identify root causes of anomalies.

### 13.3 Planning: BT, HTN, GOAP

MIRACLE supports three complementary planning paradigms:

**Behavior Trees (BT):**

```yaml
behavior_tree_executor:
  tree_directory: "behavior_trees"
  tick_rate_hz: 30.0
  max_tree_depth: 20
  blackboard_size: 1000
```

Three pre-built BT definitions are included:
- `autonomous_job.xml` -- Complete autonomous job execution
- `anomaly_response.xml` -- Anomaly detection response
- `recovery_sequence.xml` -- System recovery sequence

**Hierarchical Task Networks (HTN):**

```bash
ros2 service call /miracle/cognitive/htn_plan \
    miracle_msgs/srv/HTNPlan \
    "{task_name: 'manufacture_part', task_parameters: ['aluminum', '6061'], \
      available_methods: ['3axis_milling', '5axis_milling']}"
```

**Goal-Oriented Action Planning (GOAP):**

```bash
ros2 service call /miracle/cognitive/goap_plan \
    miracle_msgs/srv/GOAPPlan \
    "{current_state: ['machine_idle', 'tool_loaded'], \
      goal_state: ['part_complete', 'quality_verified'], \
      max_planning_time_sec: 5.0}"
```

### 13.4 Multi-Agent Coordination

Each CNC machine operates as an autonomous agent. Tasks are allocated via a
contract-net protocol:

```
1. Task Allocator publishes TaskAnnouncement (auction)
2. Machine agents evaluate and submit AgentBid
3. Auction Manager selects winner (combinatorial optimization)
4. TaskAward published to winning agent
5. Coalition Former builds multi-machine coalitions for complex jobs
6. Consensus Protocol (Raft) ensures agreement on shared decisions
```

### 13.5 Reinforcement Learning

The `rl_optimizer` uses Proximal Policy Optimization (PPO) to continuously
improve cutting parameters:

```yaml
rl_optimizer:
  algorithm: "ppo"
  learning_rate: 0.0003
  discount_factor: 0.99
  batch_size: 64
  buffer_size: 100000
  exploration_rate: 0.1
```

**Train a policy:**

```bash
ros2 action send_goal /miracle/cognitive/train_rl \
    miracle_msgs/action/TrainRLPolicy \
    "{policy_name: 'feed_rate_optimizer', environment_id: 'cnc1_sim', \
      num_episodes: 1000, learning_rate: 0.0003}"
```

### 13.6 Self-X Properties

**Self-Optimization:** Pareto multi-objective optimization over throughput,
energy efficiency, and quality with constraint satisfaction.

**Self-Healing:** Automated diagnosis with four healing strategies: restart,
reconfigure, migrate, degrade. Verification timeout of 30 seconds ensures
healing was successful.

**Self-Configuration:** Dynamic parameter adaptation with stability window
validation (60 seconds) to prevent oscillation.

**Self-Protection:** Autonomous security response with 50 ms response budget,
isolation capability, and graceful degradation under attack.

---

## 14. Protocol Bridges

### 14.1 OPC-UA Bridge

Bidirectional bridge between ROS 2 topics and an OPC-UA server:

```yaml
server_url: "opc.tcp://localhost:4840"
security_mode: "SignAndEncrypt"
security_policy: "Basic256Sha256"
certificate_path: "/certs/opcua_client.pem"
private_key_path: "/certs/opcua_client.key"
polling_interval_ms: 100
subscription_interval_ms: 50
max_nodes: 500
```

### 14.2 Sparkplug B Bridge

Publishes MIRACLE data to an MQTT broker using the Sparkplug B specification:

```yaml
mqtt_broker_host: "localhost"
mqtt_broker_port: 1883
mqtt_use_tls: true
sparkplug_group_id: "MIRACLE"
sparkplug_edge_node_id: "EdgeNode1"
birth_certificate_interval_sec: 30.0
```

### 14.3 MTConnect Agent

Exposes MIRACLE data via an MTConnect-compliant HTTP/XML agent:

```yaml
agent_host: "localhost"
agent_port: 5000
device_uuid: "miracle-cnc-001"
adapter_port: 7878
shdr_format: true
```

### 14.4 Modbus TCP Bridge

Bidirectional bridge for legacy Modbus TCP devices:

```yaml
modbus_host: "192.168.1.100"
modbus_port: 502
slave_id: 1
polling_interval_ms: 50
register_map_file: "modbus_registers.yaml"
timeout_sec: 3.0
retries: 3
```

### 14.5 Kafka Bridge

Streams ROS 2 data to Apache Kafka with schema registry integration:

```yaml
bootstrap_servers: "localhost:9092"
producer_topic_prefix: "miracle.ros"
consumer_topic_prefix: "miracle.cmd"
consumer_group: "miracle_ros_bridge"
batch_size: 100
linger_ms: 10
compression: "snappy"
schema_registry_url: "http://localhost:8081"
```

---

## 15. micro-ROS Firmware

### 15.1 ESP32 Sensor Bridge

The ESP32 firmware acquires vibration (3-axis) and spindle current data:

**Hardware:**

| Pin | Function | Sensor |
|---|---|---|
| GPIO34 (ADC1_CH6) | Vibration X-axis | ADXL345 accelerometer |
| GPIO35 (ADC1_CH7) | Vibration Y-axis | ADXL345 accelerometer |
| GPIO36 (ADC1_CH0) | Vibration Z-axis | ADXL345 accelerometer |
| GPIO39 (ADC1_CH3) | Spindle current | ACS712-05B current sensor |
| GPIO2 | Status LED | Onboard LED |

**Transport:** WiFi UDP to micro-ROS agent at port 8888

**Publish rates:**

| Data | Rate | QoS |
|---|---|---|
| Vibration X/Y/Z | 1000 Hz | best_effort |
| Current draw | 500 Hz | best_effort |
| Heartbeat | 1 Hz | reliable |

**Command subscription:** Listens on `/miracle/{machine_id}/command` for
`ESTOP`, `CALIBRATE`, and `RESET` commands.

**LED status indicators:**

| Pattern | Meaning |
|---|---|
| Solid ON | Normal operation, agent connected |
| Slow blink (500 ms) | Connecting to WiFi / agent |
| Fast blink (100 ms) | Error state |
| OFF | E-stop active |

### 15.2 STM32 Sensor Bridge

The STM32 Nucleo F446RE firmware acquires temperature, acoustic emission,
and coolant flow data:

**Hardware:**

| Pin | Function | Sensor |
|---|---|---|
| PA0 (ADC1_IN0) | Temperature | Thermocouple / NTC |
| PA1 (ADC1_IN1) | Acoustic emission | AE sensor with preamp |
| PA2 (ADC1_IN2) | Coolant flow | Pulse-based flow sensor |
| LED_BUILTIN | Status LED | Nucleo LD2 |

**Transport:** Serial UART at 921600 baud to micro-ROS agent

**Publish rates:**

| Data | Raw Rate | Publish Rate | Method |
|---|---|---|---|
| Temperature | 10 Hz | 10 Hz | Direct |
| Acoustic | 44100 Hz | 1000 Hz | RMS downsampled (window=44) |
| Coolant flow | 10 Hz | 10 Hz | Direct |
| Heartbeat | 1 Hz | 1 Hz | Direct |

### 15.3 Flashing Firmware

**ESP32:**

```bash
cd miracle_ws/src/miracle_microros/firmware/esp32

# Configure WiFi credentials (create platformio_override.ini)
cat > platformio_override.ini << 'EOF'
[env:esp32dev]
build_flags =
    ${env:esp32dev.build_flags}
    -D MIRACLE_WIFI_SSID=\"YOUR_SSID\"
    -D MIRACLE_WIFI_PASSWORD=\"YOUR_PASSWORD\"
    -D MIRACLE_AGENT_IP=\"192.168.1.1\"
    -D MIRACLE_MACHINE_ID=\"cnc_001\"
EOF

# Build and upload
pio run --target upload
```

**STM32:**

```bash
cd miracle_ws/src/miracle_microros/firmware/stm32

# Build and upload via ST-Link
pio run --target upload
```

### 15.4 Debugging

**ESP32 serial monitor:**

```bash
pio device monitor --baud 115200
```

**STM32 debugger (ST-Link GDB):**

```bash
pio debug --interface=gdb
```

**Starting the micro-ROS agent:**

```bash
# UDP agent for ESP32
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Serial agent for STM32
ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyUSB0 -b 921600
```

**Docker micro-ROS agent:**

```bash
docker compose -f docker/docker-compose.yaml up microros_agent
# Exposes UDP port 8888
```

---

## 16. Docker Deployment

### 16.1 Docker Compose Services

The `docker-compose.yaml` defines six services:

| Service | Image | Ports | Description |
|---|---|---|---|
| `ros2_miracle` | `miracle_ros2:latest` | (internal) | Full MIRACLE ROS 2 system |
| `microros_agent` | `miracle_microros:latest` | 8888/udp | micro-ROS agent |
| `dashboard` | `miracle_dashboard:latest` | 3000:80 | Web dashboard (nginx) |
| `mqtt_broker` | `eclipse-mosquitto:2` | 1883, 9001 | MQTT + WebSocket |
| `zookeeper` | `cp-zookeeper:7.6.0` | 2181 | ZooKeeper (Kafka dep.) |
| `kafka` | `cp-kafka:7.6.0` | 9092, 29092 | Apache Kafka |

**Startup:**

```bash
# Start all services
docker compose -f miracle_ws/docker/docker-compose.yaml up -d

# View logs
docker compose -f miracle_ws/docker/docker-compose.yaml logs -f

# Stop all services
docker compose -f miracle_ws/docker/docker-compose.yaml down
```

**Volumes:**

| Volume | Purpose |
|---|---|
| `miracle_data` | Persistent data (rosbags, checkpoints, resources) |
| `miracle_logs` | Log files |

**Network:**

All services share the `miracle_net` bridge network.

### 16.2 Building Images

**ROS 2 system image (multi-stage):**

```bash
docker build -f docker/Dockerfile.ros2 -t miracle_ros2:latest ..
```

Stage 1 (builder): installs dependencies and compiles the workspace.
Stage 2 (runtime): slim image with only the install overlay.

**micro-ROS agent image:**

```bash
docker build -f docker/Dockerfile.microros -t miracle_microros:latest ..
```

**Dashboard image (multi-stage):**

```bash
docker build -f docker/Dockerfile.dashboard -t miracle_dashboard:latest ..
```

Stage 1 (builder): `npm install` + `npm run build`.
Stage 2 (runtime): nginx:alpine serving the static build.

### 16.3 Production Configuration

For production deployments:

1. Set `ROS_SECURITY_ENABLE=true` and `ROS_SECURITY_STRATEGY=Enforce`
2. Configure `FASTRTPS_DEFAULT_PROFILES_FILE` for FastDDS tuning
3. Use named Docker volumes for persistent storage
4. Set MQTT authentication (disable `allow_anonymous`)
5. Configure Kafka `KAFKA_LISTENER_SECURITY_PROTOCOL_MAP` for SASL/SSL
6. Run `scripts/generate_security.sh` before deployment

---

## 17. Monitoring and Observability

### 17.1 System KPIs

The `oee_calculator` publishes `SystemKPIs` at a configurable interval (default 60s):

```
ros2 topic echo /miracle/system_kpis
```

**Fields:**

| KPI | Description |
|---|---|
| `oee` | Overall Equipment Effectiveness (0.0 - 1.0) |
| `availability` | Uptime / (Uptime + Downtime) |
| `performance` | Actual throughput / Theoretical throughput |
| `quality` | Good parts / Total parts |
| `cpk` | Process capability index |
| `mtbf` | Mean Time Between Failures (hours) |
| `mttr` | Mean Time To Repair (hours) |
| `energy_efficiency` | Output per energy unit |
| `schedule_adherence` | On-time completion percentage |
| `tool_life_utilization` | Tool life used vs. available |
| `jobs_completed_today` | Count of completed jobs |
| `jobs_in_progress` | Count of active jobs |
| `jobs_queued` | Count of queued jobs |

### 17.2 OEE Metrics

OEE is calculated as: **OEE = Availability x Performance x Quality**

```yaml
oee_calculator:
  calculation_interval_sec: 60.0
  shift_duration_hours: 8.0
  planned_downtime_codes:
    - "maintenance"
    - "changeover"
    - "break"
  target_oee: 0.85
  target_availability: 0.90
  target_performance: 0.95
  target_quality: 0.99
```

### 17.3 Heartbeat Monitoring

Every node publishes a `Heartbeat` message at its configured interval:

```
builtin_interfaces/Time timestamp
string node_name
string node_namespace
string criticality         # "critical", "high", "medium", "low"
string lifecycle_state     # "unconfigured", "inactive", "active", "finalized"
string[] dependencies
float32 cpu_usage
float32 memory_usage
```

The `heartbeat_aggregator` collects all heartbeats and computes a weighted
health score:

```bash
ros2 topic echo /miracle/heartbeats
```

### 17.4 Fleet Health

The `FleetHealth` message provides a fleet-wide health summary:

```bash
ros2 topic echo /miracle/resiliency/fleet_health
```

**Fields:**

| Field | Description |
|---|---|
| `total_nodes` | Total nodes in the system |
| `healthy_nodes` | Nodes reporting healthy |
| `degraded_nodes` | Nodes in degraded state |
| `failed_nodes` | Nodes that have failed |
| `critical_healthy` | Critical nodes that are healthy |
| `critical_total` | Total critical nodes |
| `health_score` | Overall health (0.0 - 1.0) |
| `failed_node_names` | List of failed node names |
| `degraded_node_names` | List of degraded node names |

---

## 18. Troubleshooting

### Common Issues and Solutions

**Issue: "ROS2 Jazzy not found" during setup**

```
[ERROR] ROS2 Jazzy not found at /opt/ros/jazzy/setup.bash
```

Solution: Install ROS 2 Jazzy from https://docs.ros.org/en/jazzy/Installation.html
and ensure `/opt/ros/jazzy/setup.bash` exists.

---

**Issue: Build fails with "miracle_msgs not found"**

```
Could not find a package configuration file provided by "miracle_msgs"
```

Solution: Build miracle_msgs first, then source before building other packages:

```bash
colcon build --packages-up-to miracle_msgs
source install/setup.bash
colcon build
```

---

**Issue: Dashboard shows "ROS Disconnected"**

Solution: Ensure the `hmi_bridge` node is running and the rosbridge WebSocket
server is listening on port 9090:

```bash
ros2 node list | grep hmi_bridge
ss -tlnp | grep 9090
```

---

**Issue: micro-ROS agent not connecting to ESP32**

Solution: Verify:
1. ESP32 and agent host are on the same WiFi network
2. Agent is running on UDP port 8888: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
3. `ROS_DOMAIN_ID` matches (default: 42)
4. No firewall blocking UDP port 8888

---

**Issue: Nodes not discovering each other across machines**

Solution: Ensure all machines use the same `ROS_DOMAIN_ID` and are on the same
network segment. Check DDS multicast:

```bash
ros2 multicast receive
ros2 multicast send
```

---

**Issue: SROS2 security blocks communication**

Solution: Verify keys exist for all nodes and security environment is properly set:

```bash
ls security/keystore/enclaves/
echo $ROS_SECURITY_ENABLE
echo $ROS_SECURITY_STRATEGY
```

For debugging, temporarily set `ROS_SECURITY_STRATEGY=Permissive`.

---

**Issue: Kafka bridge fails to connect**

Solution: Ensure ZooKeeper and Kafka are running:

```bash
docker compose up zookeeper kafka
# Verify Kafka is ready
docker exec miracle_kafka kafka-topics --bootstrap-server localhost:9092 --list
```

---

**Issue: Neo4j connection refused (knowledge_graph node)**

Solution: Start Neo4j separately or add it to your Docker Compose:

```bash
docker run -d --name miracle_neo4j \
    -p 7474:7474 -p 7687:7687 \
    -e NEO4J_AUTH=neo4j/miracle_kg \
    neo4j:5
```

---

**Issue: Chaos injector active in production**

The chaos injector is disabled by default (`enabled: false`). If faults are
occurring unexpectedly, verify:

```bash
ros2 param get /miracle/resiliency/chaos_injector enabled
```

---

**Issue: High CPU usage from sensor_fusion node**

Solution: Reduce the publish rate or switch to a lighter fusion algorithm:

```bash
ros2 param set /miracle/cnc1/sensor_fusion publish_rate_hz 50.0
```

---

**Issue: Remote deployment fails**

The `deploy.sh` script uses rsync + SSH. Verify:

```bash
# Test SSH connectivity
ssh operator@192.168.1.100 "echo ok"

# Deploy
bash scripts/deploy.sh operator@192.168.1.100 --remote-dir /opt/miracle_ws
```

---

## 19. API Reference

### 19.1 Message Types

#### Heartbeat.msg

Node liveness heartbeat published by every MIRACLE node.

```
builtin_interfaces/Time timestamp    # Publication time
string node_name                     # Node name (e.g., "state_publisher")
string node_namespace                # Namespace (e.g., "/miracle/cnc1")
string criticality                   # "critical", "high", "medium", "low"
string lifecycle_state               # Lifecycle state string
string[] dependencies                # List of dependent node names
float32 cpu_usage                    # CPU usage percentage (0.0-100.0)
float32 memory_usage                 # Memory usage percentage (0.0-100.0)
```

#### MachineState.msg

Complete CNC machine state snapshot.

```
builtin_interfaces/Time timestamp    # State sample time
string machine_id                    # Machine identifier
string status                        # "idle", "running", "error", "estop"
float64 spindle_speed                # Current spindle speed (RPM)
float64 feed_rate                    # Current feed rate (mm/min)
float64[] axis_positions             # Axis positions [X, Y, Z, A, B] (mm/deg)
float64[] axis_velocities            # Axis velocities (mm/min or deg/min)
float64 spindle_load                 # Spindle load percentage (0-100)
float64 coolant_level                # Coolant level percentage (0-100)
string current_program               # Active G-code program name
uint32 current_line                  # Current G-code line number
float64 cycle_time_elapsed           # Elapsed cycle time (seconds)
float64 cycle_time_remaining         # Estimated remaining time (seconds)
```

#### SensorData.msg

Raw sensor data from a single sensor channel.

```
builtin_interfaces/Time timestamp    # Sample time
string machine_id                    # Machine identifier
string sensor_type                   # "vibration", "temperature", "current", etc.
string sensor_id                     # Unique sensor identifier
float64[] values                     # Sensor readings
string[] labels                      # Labels for each value
float64 sample_rate                  # Sample rate (Hz)
uint32 sequence_number               # Monotonic sequence counter
float64 quality                      # Data quality score (0.0-1.0)
```

#### FusedSensorData.msg

Multi-sensor fusion output from the Extended Kalman Filter.

```
builtin_interfaces/Time timestamp    # Fusion time
string machine_id                    # Machine identifier
float64[] imu_features               # Fused IMU/vibration features
float64[] current_features           # Fused current features
float64[] audio_features             # Fused audio/acoustic features
float64[] vision_features            # Fused vision features
float64[] feature_vector             # Combined feature vector
uint8 sensor_health                  # Sensor health bitmask
float64 synchronization_quality      # Time sync quality (0.0-1.0)
```

#### GCodeBlock.msg

Single G-code line being executed.

```
builtin_interfaces/Time timestamp    # Execution time
string machine_id                    # Machine identifier
string program_name                  # G-code program name
uint32 line_number                   # Line number in program
string raw_line                      # Raw G-code text
string command                       # Parsed command (e.g., "G01")
float64[] parameters                 # Parsed numeric parameters
float64 feed_rate                    # Feed rate for this move
float64 spindle_speed                # Spindle speed for this move
string comment                       # Inline comment text
bool is_rapid                        # True if rapid traverse (G00)
```

#### JobStatus.msg

Manufacturing job progress status.

```
builtin_interfaces/Time timestamp    # Status time
string job_id                        # Unique job identifier
string machine_id                    # Assigned machine
string status                        # "queued", "running", "complete", "error"
string program_name                  # G-code program name
uint32 total_lines                   # Total G-code lines
uint32 current_line                  # Current line number
float64 progress                     # Progress percentage (0.0-100.0)
float64 estimated_remaining_sec      # Estimated time remaining
float64 elapsed_sec                  # Elapsed time
string[] warnings                    # Active warnings
string[] errors                      # Active errors
```

#### AnomalyAlert.msg

Anomaly detection alert from the AI layer.

```
builtin_interfaces/Time timestamp    # Detection time
string machine_id                    # Affected machine
string anomaly_type                  # Classification of anomaly
float64 confidence                   # Detection confidence (0.0-1.0)
float64 severity                     # Severity score (0.0-1.0)
string[] contributing_factors        # Top contributing features
float64[] feature_contributions      # Feature contribution scores
string recommended_action            # Suggested response action
bool requires_immediate_stop         # E-stop required flag
```

#### PHMPrediction.msg

Prognostic Health Management prediction.

```
builtin_interfaces/Time timestamp            # Prediction time
string machine_id                            # Target machine
string component                             # Component being monitored
string prediction_type                       # Failure mode type
float64 remaining_useful_life_hours          # RUL estimate
float64 confidence                           # Prediction confidence
float64 health_index                         # Health index (0.0-1.0)
string recommended_action                    # Recommended maintenance action
builtin_interfaces/Time predicted_failure_time  # Predicted failure timestamp
float64[] trend_data                         # Historical trend values
```

#### ToolWearEstimate.msg

Real-time tool wear estimation.

```
builtin_interfaces/Time timestamp    # Estimation time
string machine_id                    # Machine identifier
string tool_id                       # Tool identifier
float64 wear_percentage              # Total wear percentage (0-100)
float64 remaining_life_minutes       # Remaining useful life
float64 confidence                   # Estimation confidence
string wear_type                     # "flank", "crater", "chipping", "built_up_edge"
float64 flank_wear_mm                # Flank wear (VB) in mm
float64 crater_wear_mm               # Crater wear (KT) in mm
string recommended_action            # "continue", "plan_replacement", "replace_now"
```

#### SecurityAlert.msg

Security subsystem alert.

```
builtin_interfaces/Time timestamp    # Alert time
string alert_id                      # Unique alert identifier
string severity                      # "info", "warning", "critical", "emergency"
string category                      # Alert category
string source_node                   # Node that triggered the alert
string description                   # Human-readable description
string[] affected_nodes              # List of affected nodes
string recommended_action            # Recommended response
bool requires_isolation              # Node isolation required
float64 confidence                   # Detection confidence
```

#### DeviceTrustStatus.msg

Device trust score and attestation status.

```
builtin_interfaces/Time timestamp    # Status time
string device_id                     # Device identifier
string device_type                   # Device type classification
float64 trust_score                  # Trust score (0.0-1.0)
string attestation_status            # "verified", "pending", "failed"
builtin_interfaces/Time last_attestation  # Last attestation time
string[] active_policies             # Active security policies
bool is_quarantined                  # Quarantine status
```

#### AttestationReport.msg

Device integrity attestation report.

```
builtin_interfaces/Time timestamp    # Report time
string device_id                     # Device identifier
string firmware_hash                 # SHA-256 firmware hash
string config_hash                   # SHA-256 configuration hash
bool integrity_verified              # Integrity check result
string[] violations                  # List of detected violations
float64 trust_score                  # Updated trust score
string report_signature              # Cryptographic signature
```

#### FleetHealth.msg

Fleet-wide health summary.

```
builtin_interfaces/Time timestamp    # Summary time
uint32 total_nodes                   # Total nodes in fleet
uint32 healthy_nodes                 # Healthy node count
uint32 degraded_nodes                # Degraded node count
uint32 failed_nodes                  # Failed node count
uint32 critical_healthy              # Critical nodes that are healthy
uint32 critical_total                # Total critical nodes
float64 health_score                 # Overall fleet health (0.0-1.0)
string[] failed_node_names           # Names of failed nodes
string[] degraded_node_names         # Names of degraded nodes
```

#### NodeFailure.msg

Node failure event notification.

```
builtin_interfaces/Time timestamp    # Failure detection time
string node_name                     # Failed node name
string criticality                   # Node criticality level
string state                         # State at failure
float64 time_since_last_heartbeat    # Seconds since last heartbeat
string lifecycle_state               # Last known lifecycle state
string[] dependent_nodes             # Nodes that depend on this node
```

#### RecoveryRequest.msg

Recovery request from the supervisor.

```
builtin_interfaces/Time timestamp    # Request time
string failed_node                   # Node to recover
string criticality                   # Node criticality
string[] dependents                  # Dependent nodes
string strategy                      # Recovery strategy
string reason                        # Failure reason
uint32 attempt_number                # Recovery attempt count
```

#### KnowledgeUpdate.msg

Knowledge graph modification event.

```
builtin_interfaces/Time timestamp    # Update time
string update_type                   # "add", "modify", "delete"
string subject                       # Triple subject
string predicate                     # Triple predicate
string object_value                  # Triple object
float64 confidence                   # Confidence in this knowledge
string source                        # Source of the knowledge
string reasoning                     # Reasoning chain
```

#### TaskAnnouncement.msg

Multi-agent task auction announcement.

```
builtin_interfaces/Time timestamp    # Announcement time
string auction_id                    # Unique auction identifier
string task_type                     # Type of manufacturing task
string job_id                        # Associated job identifier
string material                      # Material to be processed
float64 complexity                   # Task complexity (0.0-1.0)
builtin_interfaces/Time deadline     # Task completion deadline
string[] required_capabilities       # Required machine capabilities
float64 estimated_duration           # Estimated duration (seconds)
string priority                      # "low", "medium", "high", "critical"
```

#### AgentBid.msg

Agent bid response to a task announcement.

```
builtin_interfaces/Time timestamp    # Bid time
string auction_id                    # Auction being bid on
string agent_id                      # Bidding agent identifier
float64 proposed_cost                # Proposed execution cost
float64 proposed_completion_time     # Proposed completion time (seconds)
float64 confidence                   # Bid confidence
float64 current_load                 # Agent current load (0.0-1.0)
string[] capabilities_offered        # Capabilities this agent offers
```

#### TaskAward.msg

Task award notification to the winning agent.

```
builtin_interfaces/Time timestamp    # Award time
string auction_id                    # Auction identifier
string task_type                     # Task type
string awarded_agent_id              # Winning agent identifier
string job_id                        # Associated job
float64 agreed_cost                  # Agreed cost
float64 agreed_completion_time       # Agreed completion time
```

#### BehaviorTreeStatus.msg

Behavior tree execution status.

```
builtin_interfaces/Time timestamp    # Status time
string tree_id                       # Behavior tree identifier
string root_status                   # "success", "failure", "running"
string tip_name                      # Currently active leaf node
string[] active_behaviors            # List of active behavior nodes
float64 execution_time_sec           # Total execution time
uint32 tick_count                    # Number of ticks executed
```

#### ManufacturingExperience.msg

Learned manufacturing experience for knowledge accumulation.

```
builtin_interfaces/Time timestamp    # Experience time
string machine_id                    # Machine that generated experience
string job_id                        # Job identifier
string material                      # Material processed
string operation_type                # Operation type
float64[] process_parameters         # Process parameters used
float64[] quality_metrics            # Quality outcomes
float64[] sensor_summary             # Sensor data summary
float64 oee_achieved                 # OEE achieved for this job
string outcome                       # "success", "partial", "failure"
string[] lessons_learned             # Extracted lessons
```

#### FederatedModel.msg

Global federated model broadcast.

```
builtin_interfaces/Time timestamp    # Broadcast time
string model_id                      # Model identifier
string model_type                    # Model architecture type
uint32 round_number                  # Federated round number
uint32 num_participants              # Participating clients
float64[] global_weights             # Aggregated global weights
float64 global_loss                  # Global loss metric
float64 convergence_metric           # Convergence indicator
```

#### ModelUpdate.msg

Local model update from a federated client.

```
builtin_interfaces/Time timestamp    # Update time
string model_id                      # Model identifier
string client_id                     # Client identifier
uint32 round_number                  # Round number
float64[] local_weights              # Local model weights
float64 local_loss                   # Local loss value
uint32 num_samples                   # Training samples used
float64[] gradient_norms             # Gradient norms per layer
```

#### OptimizationAction.msg

Self-optimization action taken by the cognitive layer.

```
builtin_interfaces/Time timestamp    # Action time
string machine_id                    # Target machine
string action_type                   # Optimization action type
string parameter_name                # Parameter being optimized
float64 old_value                    # Previous value
float64 new_value                    # New optimized value
float64 expected_improvement         # Expected improvement percentage
string reasoning                     # Reasoning for the change
float64 confidence                   # Confidence in improvement
```

#### SystemKPIs.msg

System-wide key performance indicators.

```
builtin_interfaces/Time timestamp    # KPI calculation time
float64 oee                          # Overall Equipment Effectiveness
float64 availability                 # Availability factor
float64 performance                  # Performance factor
float64 quality                      # Quality factor
float64 cpk                          # Process capability index
float64 mtbf                         # Mean Time Between Failures (hours)
float64 mttr                         # Mean Time To Repair (hours)
float64 energy_efficiency            # Energy efficiency metric
float64 schedule_adherence           # Schedule adherence (0.0-1.0)
float64 tool_life_utilization        # Tool life utilization (0.0-1.0)
uint32 jobs_completed_today          # Jobs completed today
uint32 jobs_in_progress              # Jobs currently in progress
uint32 jobs_queued                   # Jobs waiting in queue
```

#### DigitalThreadEntry.msg

Digital thread traceability entry with hash chain.

```
builtin_interfaces/Time timestamp    # Entry time
string entry_id                      # Unique entry identifier
string job_id                        # Associated job
string entry_type                    # Entry type classification
string source_node                   # Source node name
string data_json                     # JSON payload
string[] tags                        # Searchable tags
string previous_entry_id             # Previous entry (hash chain)
string hash_value                    # SHA-256 hash of this entry
```

### 19.2 Service Types

#### RegisterDevice.srv

Register a new device in the MIRACLE system.

```
# Request
string device_id                     # Device identifier
string device_type                   # Device type
string[] capabilities                # Device capabilities
string firmware_version              # Firmware version string
---
# Response
bool success                         # Registration success
string message                       # Status message
float64 initial_trust_score          # Assigned initial trust score
```

#### GetFleetStatus.srv

Query fleet-wide health status with optional filtering.

```
# Request
string filter_criticality            # Filter by criticality (empty = all)
string filter_state                  # Filter by state (empty = all)
---
# Response
miracle_msgs/FleetHealth fleet_health  # Fleet health summary
string[] node_details_json           # Per-node detail JSON strings
```

#### ValidateGCode.srv

Pre-validate a G-code program before execution.

```
# Request
string program_content               # G-code program text
string machine_id                    # Target machine
---
# Response
bool is_valid                        # Validation result
string[] errors                      # Validation errors
string[] warnings                    # Validation warnings
float64 estimated_duration_sec       # Estimated execution time
```

#### TriggerEStop.srv

Trigger an emergency stop on a machine.

```
# Request
string machine_id                    # Target machine
string reason                        # E-stop reason
string requesting_node               # Node requesting E-stop
---
# Response
bool success                         # E-stop triggered successfully
string message                       # Status message
```

#### RequestAttestation.srv

Request device integrity attestation.

```
# Request
string device_id                     # Device to attest
string challenge_nonce               # Cryptographic nonce
---
# Response
bool success                         # Attestation completed
miracle_msgs/AttestationReport report  # Full attestation report
```

#### TriggerFailover.srv

Trigger a manual failover for a node.

```
# Request
string failed_node                   # Failed node name
string strategy                      # Failover strategy
---
# Response
bool success                         # Failover success
string backup_node                   # Backup node activated
string message                       # Status message
```

#### RestoreCheckpoint.srv

Restore a node's state from a checkpoint.

```
# Request
string node_name                     # Node to restore
string checkpoint_id                 # Checkpoint ID ("latest" for most recent)
---
# Response
bool success                         # Restore success
string message                       # Status message
builtin_interfaces/Time checkpoint_timestamp  # Checkpoint time
```

#### SPARQLQuery.srv

Query the knowledge graph using SPARQL.

```
# Request
string query                         # SPARQL query string
string graph_name                    # Target graph name
---
# Response
bool success                         # Query success
string result_json                   # Results as JSON
uint32 num_results                   # Number of results
```

#### SubmitTask.srv

Submit a manufacturing task for multi-agent allocation.

```
# Request
string task_type                     # Task type
string job_id                        # Job identifier
string material                      # Material
float64 complexity                   # Complexity score
string priority                      # Priority level
string[] required_capabilities       # Required capabilities
---
# Response
bool accepted                        # Task accepted for allocation
string auction_id                    # Auction identifier
string message                       # Status message
```

#### OptimizeParameters.srv

Request parameter optimization from the RL optimizer.

```
# Request
string machine_id                    # Target machine
string job_id                        # Job context
string optimization_target           # Target metric
float64[] current_parameters         # Current parameter values
string[] parameter_names             # Parameter names
---
# Response
bool success                         # Optimization success
float64[] optimized_parameters       # Optimized values
float64 expected_improvement         # Expected improvement
string reasoning                     # Optimization reasoning
```

#### GOAPPlan.srv

Generate a Goal-Oriented Action Plan.

```
# Request
string[] current_state               # Current world state predicates
string[] goal_state                  # Desired goal state predicates
float64 max_planning_time_sec        # Planning time budget
---
# Response
bool success                         # Plan found
string[] action_sequence             # Ordered action list
float64 total_cost                   # Total plan cost
string plan_explanation              # Human-readable explanation
```

#### HTNPlan.srv

Generate a Hierarchical Task Network plan.

```
# Request
string task_name                     # Top-level task
string[] task_parameters             # Task parameters
string[] available_methods           # Available decomposition methods
---
# Response
bool success                         # Plan found
string[] plan_steps                  # Ordered primitive actions
float64 estimated_duration           # Estimated total duration
string plan_tree_json                # Full plan tree as JSON
```

#### NLPCommand.srv

Interpret a natural language command.

```
# Request
string natural_language_input        # User's natural language input
string context_json                  # Contextual information
---
# Response
bool understood                      # Input was understood
string interpreted_action            # Interpreted action
string[] parameters                  # Extracted parameters
float64 confidence                   # Interpretation confidence
string clarification_question        # Follow-up question (if needed)
```

#### InjectFault.srv

Inject a controlled fault for resilience testing.

```
# Request
string target_node                   # Node to inject fault into
string fault_type                    # "node_crash", "message_delay", etc.
float64 duration_sec                 # Fault duration
float64 intensity                    # Fault intensity (0.0-1.0)
---
# Response
bool success                         # Injection success
string fault_id                      # Fault tracking identifier
string message                       # Status message
```

### 19.3 Action Types

#### ExecuteProgram.action

Execute a G-code program on a CNC machine with progress feedback.

```
# Goal
string machine_id                    # Target machine
string program_name                  # Program name
string program_content               # G-code content
float64[] override_parameters        # Parameter overrides
---
# Result
bool success                         # Execution success
string message                       # Result message
float64 total_time_sec               # Total execution time
uint32 lines_executed                # Lines successfully executed
float64[] quality_metrics            # Quality measurement results
---
# Feedback
uint32 current_line                  # Current line being executed
uint32 total_lines                   # Total lines in program
float64 progress                     # Progress percentage (0-100)
string current_operation             # Current operation description
float64 elapsed_sec                  # Elapsed time
float64 estimated_remaining_sec      # Estimated remaining time
```

#### RunPrediction.action

Run a predictive analysis on a machine component.

```
# Goal
string machine_id                    # Target machine
string prediction_type               # Prediction type
float64 prediction_horizon_hours     # Lookahead horizon
---
# Result
bool success                         # Prediction success
miracle_msgs/PHMPrediction prediction  # Prediction result
string detailed_report_json          # Detailed JSON report
---
# Feedback
float64 progress                     # Analysis progress (0-100)
string current_phase                 # Current analysis phase
```

#### PerformCalibration.action

Run a machine calibration procedure.

```
# Goal
string machine_id                    # Target machine
string calibration_type              # Calibration type
string[] axes                        # Axes to calibrate
---
# Result
bool success                         # Calibration success
float64[] offsets                     # Calibration offsets
float64[] accuracies                 # Achieved accuracies
string report_json                   # Calibration report
---
# Feedback
float64 progress                     # Calibration progress
string current_step                  # Current calibration step
string current_axis                  # Axis being calibrated
```

#### IsolateNode.action

Isolate a compromised or faulty node from the network.

```
# Goal
string target_node                   # Node to isolate
string reason                        # Isolation reason
string isolation_level               # "network", "topic", "full"
---
# Result
bool success                         # Isolation success
string[] affected_nodes              # Nodes affected by isolation
string message                       # Status message
---
# Feedback
float64 progress                     # Isolation progress
string current_step                  # Current isolation step
```

#### ExecuteJob.action

Execute a complete manufacturing job with full lifecycle tracking.

```
# Goal
string job_id                        # Job identifier
string machine_id                    # Target machine
string program_name                  # G-code program
string material                      # Material type
string priority                      # Job priority
---
# Result
bool success                         # Job success
string message                       # Result message
float64 total_time_sec               # Total job time
float64 oee_achieved                 # OEE for this job
float64[] quality_metrics            # Quality results
---
# Feedback
float64 progress                     # Job progress (0-100)
string current_phase                 # Current phase
string current_operation             # Current operation
float64 elapsed_sec                  # Elapsed time
float64 estimated_remaining_sec      # Estimated remaining time
miracle_msgs/MachineState machine_state  # Current machine state
```

#### TrainRLPolicy.action

Train a reinforcement learning policy.

```
# Goal
string policy_name                   # Policy identifier
string environment_id                # Training environment
uint32 num_episodes                  # Number of training episodes
float64 learning_rate                # Learning rate
---
# Result
bool success                         # Training success
float64 final_reward                 # Final episode reward
float64[] reward_history             # Reward history
string model_path                    # Saved model path
---
# Feedback
uint32 current_episode               # Current episode number
uint32 total_episodes                # Total episodes
float64 current_reward               # Current episode reward
float64 average_reward               # Running average reward
```

#### FederatedRound.action

Execute one round of federated learning.

```
# Goal
string model_id                      # Model identifier
uint32 round_number                  # Round number
uint32 min_participants              # Minimum participants required
float64 timeout_sec                  # Round timeout
---
# Result
bool success                         # Round success
uint32 num_participants              # Actual participants
float64 global_loss                  # Aggregated global loss
float64 convergence_metric           # Convergence indicator
---
# Feedback
uint32 updates_received              # Updates received so far
uint32 updates_expected              # Total updates expected
float64 elapsed_sec                  # Elapsed time
string current_phase                 # "collecting", "aggregating", "distributing"
```

---

## 20. Unity Digital Twin

The Unity Digital Twin is a high-fidelity 3D visualization layer for the
MIRACLE system, built in Unity 6 LTS with the Universal Render Pipeline (URP).
It connects to the ROS 2 stack via the `miracle_unity_bridge` package and
provides real-time 3D rendering of the entire manufacturing cell.

### 20.1 Overview

The Unity twin complements the Gazebo-based simulation (`miracle_twin`) by
providing:

- Photorealistic rendering of machines, robots, and workpieces
- GPU-accelerated voxel material removal with marching cubes mesh extraction
- Mechanistic cutting force visualization (Altintas model)
- Thermal field overlays and tool wear progression
- VFX Graph-driven chip and coolant particle effects
- Interactive camera system for operator monitoring

### 20.2 Manufacturing Cell

The Unity scene includes the following equipment:

| Equipment | Role | ROS 2 Interface |
|---|---|---|
| Bantam Tools Explorer CNC | Desktop 3-axis milling | Joint states, G-code status, sensor data |
| Niryo Ned2 | Collaborative robot (pick-and-place) | Joint trajectory, gripper state |
| xArm 6 Lite | Machine tending robot | Joint trajectory, end-effector pose |

### 20.3 Connection Setup

The Unity application communicates with ROS 2 through a TCP bridge:

1. **ROS side:** `miracle_unity_bridge` launches `ros_tcp_endpoint` on port 10000
2. **Unity side:** The `ROS-TCP-Connector` package connects to `localhost:10000`
3. **Data flow:** ROS 2 topics are serialized, sent over TCP, deserialized in Unity

```
[ROS 2 Nodes] <--> [DDS] <--> [ros_tcp_endpoint :10000] <--> [TCP] <--> [Unity ROS-TCP-Connector] <--> [Unity C# Scripts]
```

### 20.4 Key Features

**Voxel Material Removal:**
A 3D voxel grid represents the workpiece. As the CNC tool follows its
programmed path, voxels are removed in real time using GPU compute shaders.
The marching cubes algorithm extracts a smooth triangle mesh from the
remaining voxel data each frame for rendering.

**Altintas Cutting Force Model:**
The mechanistic cutting force model computes tangential, radial, and axial
forces using shearing coefficients (Ktc, Krc, Kac) and edge coefficients
(Kte, Kre, Kae). Forces are visualized as 3D arrows and drive chip particle
emission rates.

**Thermal Simulation:**
Temperature data from ROS 2 thermal sensors is mapped onto the workpiece
surface using custom URP shaders, displaying heat gradients in real time.

**Tool Wear Tracking:**
Flank wear (VB) and crater wear (KT) values from `miracle_ai.tool_wear_estimator`
are visualized on the cutting tool geometry with progressive material
degradation effects.

### 20.5 Quick Start

```bash
# Terminal 1: Launch the MIRACLE simulation with Unity bridge
cd miracle_ws
source install/setup.bash
ros2 launch miracle_bringup miracle_simulation.launch.py

# Terminal 2 (optional): Launch Unity bridge separately
ros2 launch miracle_unity_bridge unity_bridge.launch.py

# Unity: Open the project in Unity 6 LTS and press Play
```

For complete documentation, see `docs/UNITY_TWIN_MANUAL.md`.

---

## 21. Glossary

| Term | Definition |
|---|---|
| **AE** | Acoustic Emission -- high-frequency stress waves from tool-workpiece interaction |
| **Altintas Model** | Mechanistic cutting force model using shearing + edge coefficients |
| **BT** | Behavior Tree -- hierarchical decision-making structure for autonomous agents |
| **Cpk** | Process Capability Index -- statistical measure of process consistency |
| **DDS** | Data Distribution Service -- middleware standard underlying ROS 2 |
| **Digital Thread** | Complete traceability chain linking design, manufacturing, and inspection data |
| **Digital Twin** | Virtual replica of a physical machine synchronized in real time |
| **E-stop** | Emergency Stop -- immediate cessation of all machine motion |
| **EKF** | Extended Kalman Filter -- nonlinear state estimation algorithm |
| **FedAvg** | Federated Averaging -- aggregation algorithm for federated learning |
| **FFT** | Fast Fourier Transform -- frequency domain analysis of vibration signals |
| **G-code** | Machine language for CNC programming (ISO 6983) |
| **GOAP** | Goal-Oriented Action Planning -- AI planning paradigm from game AI |
| **HMI** | Human-Machine Interface -- operator control and visualization |
| **HTN** | Hierarchical Task Network -- plan decomposition method |
| **IDS** | Intrusion Detection System -- cybersecurity monitoring |
| **Lifecycle Node** | ROS 2 managed node with configurable state transitions |
| **Marching Cubes** | GPU isosurface extraction algorithm for voxel mesh rendering |
| **MES** | Manufacturing Execution System -- production management layer |
| **micro-ROS** | ROS 2 for microcontrollers (ESP32, STM32, etc.) |
| **MQTT** | Message Queuing Telemetry Transport -- lightweight pub/sub protocol |
| **MTBF** | Mean Time Between Failures |
| **MTTR** | Mean Time To Repair |
| **OEE** | Overall Equipment Effectiveness = Availability x Performance x Quality |
| **OPC-UA** | Open Platform Communications Unified Architecture -- industrial protocol |
| **OWL** | Web Ontology Language -- knowledge representation standard |
| **PHM** | Prognostic Health Management -- predictive maintenance |
| **PINN** | Physics-Informed Neural Network -- ML model constrained by physics |
| **PLC** | Programmable Logic Controller -- industrial control hardware |
| **PPO** | Proximal Policy Optimization -- reinforcement learning algorithm |
| **QoS** | Quality of Service -- DDS communication reliability/durability settings |
| **Raft** | Distributed consensus protocol for leader election |
| **RBAC** | Role-Based Access Control -- security authorization model |
| **RL** | Reinforcement Learning -- optimization through reward-based training |
| **ROS-TCP-Connector** | Unity Technologies' TCP bridge between Unity and ROS 2 |
| **rosbridge** | WebSocket bridge between ROS 2 and web applications |
| **RUL** | Remaining Useful Life -- predicted time until component failure |
| **SCADA** | Supervisory Control and Data Acquisition |
| **SHDR** | Simple Hierarchical Data Representation -- MTConnect adapter format |
| **Sparkplug B** | MQTT-based protocol for industrial IoT (Eclipse Foundation) |
| **SPC** | Statistical Process Control -- quality monitoring with control charts |
| **SROS2** | Secure ROS 2 -- DDS security extension with encryption and authentication |
| **TimescaleDB** | Time-series database extension for PostgreSQL |
| **TPM** | Trusted Platform Module -- hardware security chip |
| **URP** | Universal Render Pipeline -- Unity's default scriptable render pipeline |
| **VB** | Flank Wear width -- ISO 3685 tool wear measurement |
| **VFX Graph** | Unity's GPU-based visual effects system for particle simulation |
| **Voxel** | Volumetric pixel -- 3D grid element used for material removal simulation |

---

*This document was generated for MIRACLE v1.1.0. For Unity Digital Twin documentation, see docs/UNITY_TWIN_MANUAL.md.
For the latest version, see the repository at https://github.com/banatam/miracle_cnc_digital_twin.*
