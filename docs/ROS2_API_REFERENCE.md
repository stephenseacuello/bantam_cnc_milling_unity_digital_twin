# MIRACLE ROS2 API Reference

> Comprehensive reference for all ROS2 topics, services, actions, and message types in the MIRACLE (Manufacturing Intelligence, Resilience, And Cognitive Learning Engine) digital twin system.

---

## Overview

The MIRACLE system is a distributed ROS2 architecture for CNC milling digital twins. It connects a Unity 3D visualization front-end to a fleet of ROS2 nodes spanning machine control, AI/ML analytics, manufacturing execution, SCADA, security, resiliency, and cognitive planning.

**Key architectural layers:**

- **CNC Layer** (`miracle_cnc`) -- Sensor fusion, G-code execution, SPC monitoring, and machine watchdog.
- **AI Layer** (`miracle_ai`) -- Anomaly detection, PHM prediction, tool wear estimation, and chatter detection.
- **MES Layer** (`miracle_mes`) -- Job scheduling, fleet management, OEE calculation, and digital thread traceability.
- **SCADA Layer** (`miracle_scada`) -- Device discovery, traffic management, alarm aggregation, HMI WebSocket bridge, and Prometheus metrics export.
- **Twin Layer** (`miracle_twin`) -- State synchronization, Gazebo bridge, prediction runner, and scenario management.
- **Resiliency Layer** (`miracle_resiliency`) -- Heartbeat monitoring, Erlang-style supervision trees, failover coordination, checkpointing, and chaos testing.
- **Security Layer** (`miracle_security`) -- Intrusion detection, remote attestation, threat response, access control, and audit logging.
- **Cognitive Layer** (`miracle_cognitive`) -- Knowledge graphs, planning (GOAP/HTN), multi-agent task allocation, reinforcement learning, federated learning, and self-x autonomic capabilities.
- **Bridges** (`miracle_bridges`) -- Protocol bridges for OPC-UA, Modbus TCP, MTConnect, Sparkplug B (MQTT), and Apache Kafka.
- **Unity Client** -- `MiracleBridge` (C#) connects via ROS-TCP-Connector over TCP port 10000.

**Communication model:** All lifecycle nodes extend `MiracleLifecycleNode`, which provides automatic heartbeat publishing to `/miracle/heartbeats`. Per-machine topics use the pattern `/miracle/{machine_id}/...` where `machine_id` defaults to `cnc1`, `cnc2`, `cnc3`. The ROS2 domain ID is **42**.

---

## Table of Contents

- [ROS2 Topics](#ros2-topics)
  - [Per-Machine Topics](#per-machine-topics)
  - [System-Wide Topics](#system-wide-topics)
- [ROS2 Services](#ros2-services)
- [ROS2 Actions](#ros2-actions)
- [Message Definitions (miracle_msgs)](#message-definitions-miraclemsgs)
  - [Messages](#messages)
  - [Services](#services)
  - [Actions](#actions)
- [Network Ports](#network-ports)
- [QoS Profiles](#qos-profiles)
- [Node Summary](#node-summary)
  - [miracle_cnc](#miracle_cnc)
  - [miracle_ai](#miracle_ai)
  - [miracle_bridges](#miracle_bridges)
  - [miracle_mes](#miracle_mes)
  - [miracle_scada](#miracle_scada)
  - [miracle_twin](#miracle_twin)
  - [miracle_resiliency](#miracle_resiliency)
  - [miracle_security](#miracle_security)
  - [miracle_cognitive](#miracle_cognitive)
  - [miracle_unity_bridge](#miracle_unity_bridge)
  - [miracle_bringup](#miracle_bringup)
  - [Unity (MiracleBridge)](#unity-miraclebridge)

---

## ROS2 Topics

### Per-Machine Topics

These topics are namespaced per machine: `/miracle/{machine_id}/...` where `machine_id` is typically `cnc1`, `cnc2`, `cnc3`, etc.

| Topic | Message Type | Publisher(s) | Subscriber(s) | QoS Profile |
|-------|-------------|-------------|---------------|-------------|
| `/miracle/{machine_id}/state` | `miracle_msgs/MachineState` | `state_publisher`, `opc_ua_bridge`, `sparkplug_bridge`, `mtconnect_agent` | `unity_bridge` (Unity), `hmi_bridge`, `spc_monitor`, `local_watchdog`, `tool_wear_estimator`, `chatter_detector`, `fleet_manager`, `oee_calculator`, `job_scheduler`, `checkpoint_manager`, `sync_engine`, `prometheus_exporter`, `behavior_tree_executor`, `kafka_bridge`, `historian` | State |
| `/miracle/{machine_id}/anomaly` | `miracle_msgs/AnomalyAlert` | `anomaly_detector`, `local_watchdog` | `unity_bridge` (Unity), `hmi_bridge`, `rosbag_trigger`, `alarm_manager`, `kafka_bridge`, `historian`, `digital_thread`, `causal_inference`, `behavior_tree_executor`, `self_healer`, `prometheus_exporter` | Alert |
| `/miracle/{machine_id}/tool_wear` | `miracle_msgs/ToolWearEstimate` | `tool_wear_estimator` | `unity_bridge` (Unity) | State |
| `/miracle/{machine_id}/job_status` | `miracle_msgs/JobStatus` | `job_scheduler` | `unity_bridge` (Unity), `oee_calculator`, `kafka_bridge`, `historian`, `checkpoint_manager`, `digital_thread` | State |
| `/miracle/{machine_id}/sensor/fused` | `miracle_msgs/FusedSensorData` | `sensor_fusion` | `anomaly_detector`, `phm_predictor`, `tool_wear_estimator`, `chatter_detector`, `sync_engine` | Sensor |
| `/miracle/{machine_id}/sensor/imu` | `sensor_msgs/Imu` | (hardware/external) | `sensor_fusion` | Sensor |
| `/miracle/{machine_id}/sensor/current` | `std_msgs/Float32MultiArray` | (hardware/external) | `sensor_fusion` | Sensor |
| `/miracle/{machine_id}/sensor/audio_fft` | `std_msgs/Float32MultiArray` | (hardware/external) | `sensor_fusion` | Sensor |
| `/miracle/{machine_id}/sensor/camera` | `sensor_msgs/Image` | (hardware/external) | `sensor_fusion` | Sensor |
| `/miracle/{machine_id}/sensor_data` | `miracle_msgs/SensorData` | `modbus_bridge`, `opc_ua_bridge`, `sparkplug_bridge` | (sensor_fusion pipeline) | Sensor |
| `/miracle/{machine_id}/gcode_block` | `miracle_msgs/GCodeBlock` | `gcode_executor` | (downstream consumers) | Command |
| `/miracle/{machine_id}/spc_alert` | `miracle_msgs/AnomalyAlert` | `spc_monitor` | (downstream consumers) | Alert |
| `/miracle/{machine_id}/chatter_alert` | `miracle_msgs/AnomalyAlert` | `chatter_detector` | (downstream consumers) | Alert |
| `/miracle/{machine_id}/phm_prediction` | `miracle_msgs/PHMPrediction` | `phm_predictor` | (downstream consumers) | State |

### System-Wide Topics

| Topic | Message Type | Publisher(s) | Subscriber(s) | QoS Profile |
|-------|-------------|-------------|---------------|-------------|
| `/miracle/heartbeats` | `miracle_msgs/Heartbeat` | All `MiracleLifecycleNode` instances (via `HeartbeatMixin`) | `heartbeat_aggregator`, `traffic_manager`, `intrusion_detection`, `agent_registry` | Heartbeat |
| `/miracle/unity/heartbeat` | `miracle_msgs/HeartbeatMsg` | Unity (`MiracleBridge`) | `unity_endpoint` | Heartbeat |
| `/miracle/system_kpis` | `miracle_msgs/SystemKPIs` | `oee_calculator` | Unity (`MiracleBridge`), `hmi_bridge`, `kafka_bridge`, `prometheus_exporter`, `goal_manager`, `self_optimizer` | State |
| `/miracle/twin/sync_status` | `miracle_msgs/TwinSyncStatus` | `sync_engine` | Unity (`MiracleBridge`) | State |
| `/miracle/twin/twin_state` | `miracle_msgs/MachineState` | `sync_engine` | `gazebo_bridge` | State |
| `/miracle/resiliency/fleet_health` | `miracle_msgs/FleetHealth` | `heartbeat_aggregator`, `fleet_manager` | `supervisor_root`, `hmi_bridge`, `self_healer` | State |
| `/miracle/resiliency/failures` | `miracle_msgs/NodeFailure` | `heartbeat_aggregator` | `supervisor_root`, `failover_coordinator` | Alert |
| `/miracle/resiliency/recovery_requests` | `miracle_msgs/RecoveryRequest` | `heartbeat_aggregator`, `supervisor_root` | `recovery_orchestrator` | Alert |
| `/miracle/security/alerts` | `miracle_msgs/SecurityAlert` | `intrusion_detection`, `access_enforcer` | Unity (`MiracleBridge`), `threat_response`, `audit_logger`, `alarm_manager`, `self_protector` | Alert |
| `/miracle/cognitive/task_announcements` | `miracle_msgs/TaskAnnouncement` | `job_scheduler`, `auction_manager` | (agent bidders) | Command |
| `/miracle/cognitive/bids` | `miracle_msgs/AgentBid` | (agent bidders) | `auction_manager`, `task_allocator` | Command |
| `/miracle/cognitive/task_awards` | `miracle_msgs/TaskAward` | `auction_manager`, `task_allocator` | Unity (`MiracleBridge`) | Command |
| `/miracle/cognitive/knowledge_updates` | `miracle_msgs/KnowledgeUpdate` | (external knowledge sources) | `knowledge_graph` | State |
| `/miracle/cognitive/knowledge_events` | `miracle_msgs/KnowledgeUpdate` | `knowledge_graph` | `reasoning_engine` | State |
| `/miracle/cognitive/inferences` | `miracle_msgs/KnowledgeUpdate` | `reasoning_engine` | (downstream consumers) | State |
| `/miracle/cognitive/causal_inferences` | `miracle_msgs/KnowledgeUpdate` | `causal_inference` | (downstream consumers) | State |
| `/miracle/cognitive/optimization_actions` | `miracle_msgs/OptimizationAction` | `self_optimizer`, `rl_optimizer` | (actuators/controllers) | Command |
| `/miracle/cognitive/tree_status` | `miracle_msgs/BehaviorTreeStatus` | `behavior_tree_executor` | (monitoring) | State |
| `/miracle/cognitive/model_updates` | `miracle_msgs/ModelUpdate` | `federated_client` | `federated_coordinator` | Bulk |
| `/miracle/cognitive/global_model` | `miracle_msgs/FederatedModel` | `federated_coordinator` | `federated_client` | Bulk |
| `/miracle/cognitive/escalation_notifications` | `miracle_msgs/SecurityAlert` | `human_escalation` | (notification consumers) | Alert |
| `/miracle/cognitive/predictions` | `miracle_msgs/PHMPrediction` | `prediction_runner` | (downstream consumers) | State |
| `/miracle/scada/device_status` | `miracle_msgs/DeviceTrustStatus` | `discovery_server` | (monitoring) | State |
| `/miracle/security/device_trust` | `miracle_msgs/DeviceTrustStatus` | `attestation_verifier` | (monitoring) | State |
| `/miracle/mes/entries` | `miracle_msgs/DigitalThreadEntry` | `digital_thread` | (audit/traceability) | Logging |
| `/miracle/ai/model_updates` | `miracle_msgs/ModelUpdate` | `model_manager` | (model consumers) | State |
| `/miracle/robots/joint_states` | `miracle_msgs/RobotJointState` | Unity (`MiracleBridge`) | Unity (`MiracleBridge`) | (default) |
| `~/connection_status` | `std_msgs/Bool` | `unity_endpoint` | (monitoring) | Reliable/TransientLocal |

---

## ROS2 Services

| Service | Type | Server Node | Client(s) | Package |
|---------|------|-------------|-----------|---------|
| `/miracle/{machine_id}/trigger_estop` | `miracle_msgs/srv/TriggerEStop` | `state_publisher` | Unity (`MiracleBridge`), `local_watchdog` | miracle_cnc |
| `/miracle/{machine_id}/validate_gcode` | `miracle_msgs/srv/ValidateGCode` | `gcode_executor` | Unity (`MiracleBridge`) | miracle_cnc |
| `/miracle/mes/validate_gcode` | `miracle_msgs/srv/ValidateGCode` | (routed) | Unity (`MiracleBridge`) | miracle_cnc |
| `/miracle/fleet/get_status` | `miracle_msgs/srv/GetFleetStatus` | `fleet_manager` | Unity (`MiracleBridge`) | miracle_mes |
| `/miracle/mes/submit_task` | `miracle_msgs/srv/SubmitTask` | `job_scheduler` | (MES clients) | miracle_mes |
| `/miracle/scada/register_device` | `miracle_msgs/srv/RegisterDevice` | `discovery_server` | (device registrations) | miracle_scada |
| `/miracle/security/request_attestation` | `miracle_msgs/srv/RequestAttestation` | `attestation_verifier` | (security clients) | miracle_security |
| `/miracle/security/isolate_node` | `miracle_msgs/srv/IsolateNode` | `threat_response` | (dashboard/security) | miracle_security |
| `/miracle/resiliency/trigger_failover` | `miracle_msgs/srv/TriggerFailover` | `failover_coordinator` | (resiliency clients) | miracle_resiliency |
| `/miracle/resiliency/restore_checkpoint` | `miracle_msgs/srv/RestoreCheckpoint` | `checkpoint_manager` | (recovery clients) | miracle_resiliency |
| `/miracle/resiliency/inject_fault` | `miracle_msgs/srv/InjectFault` | `chaos_injector` | (chaos testing) | miracle_resiliency |
| `/miracle/cognitive/sparql_query` | `miracle_msgs/srv/SPARQLQuery` | `knowledge_graph` | (knowledge clients) | miracle_cognitive |
| `/miracle/cognitive/goap_plan` | `miracle_msgs/srv/GOAPPlan` | `goap_planner` | (planning clients) | miracle_cognitive |
| `/miracle/cognitive/htn_plan` | `miracle_msgs/srv/HTNPlan` | `htn_planner` | (planning clients) | miracle_cognitive |
| `/miracle/cognitive/nlp_command` | `miracle_msgs/srv/NLPCommand` | `nlp_interface` | (HMI/voice) | miracle_cognitive |
| `/miracle/cognitive/rl_optimize` | `miracle_msgs/srv/OptimizeParameters` | `rl_optimizer` | (optimization clients) | miracle_cognitive |
| `/miracle/twin/run_prediction` | `miracle_msgs/srv/RunPrediction` | `prediction_runner` | (dashboard/API) | miracle_twin |

---

## ROS2 Actions

| Action | Type | Server Node | Package | Description |
|--------|------|-------------|---------|-------------|
| `/miracle/{machine_id}/execute_program` | `miracle_msgs/action/ExecuteProgram` | `gcode_executor` | miracle_cnc | Execute a G-code program on a CNC machine |
| `/miracle/mes/execute_job` | `miracle_msgs/action/ExecuteJob` | `job_scheduler` | miracle_mes | Execute a complete manufacturing job |
| `/miracle/twin/run_prediction` | `miracle_msgs/action/RunPrediction` | `prediction_runner` | miracle_twin | Run predictive what-if scenarios |
| `/miracle/security/isolate_node` | `miracle_msgs/action/IsolateNode` | `threat_response` | miracle_security | Isolate a compromised node |
| `/miracle/cognitive/train_policy` | `miracle_msgs/action/TrainRLPolicy` | `rl_optimizer` | miracle_cognitive | Train a reinforcement learning policy |
| `/miracle/cognitive/federated_round` | `miracle_msgs/action/FederatedRound` | `federated_coordinator` | miracle_cognitive | Execute a federated learning round |

---

## Message Definitions (miracle_msgs)

### Messages

#### MachineState
```
builtin_interfaces/Time timestamp
string machine_id
string status
float64 spindle_speed
float64 feed_rate
float64[] axis_positions
float64[] axis_velocities
float64 spindle_load
float64 coolant_level
string current_program
uint32 current_line
float64 cycle_time_elapsed
float64 cycle_time_remaining
```

#### SensorData
```
builtin_interfaces/Time timestamp
string machine_id
string sensor_type
string sensor_id
float64[] values
string[] labels
float64 sample_rate
uint32 sequence_number
float64 quality
```

#### FusedSensorData
```
builtin_interfaces/Time timestamp
string machine_id
float64[] imu_features
float64[] current_features
float64[] audio_features
float64[] vision_features
float64[] feature_vector
uint8 sensor_health
float64 synchronization_quality
```

#### GCodeBlock
```
builtin_interfaces/Time timestamp
string machine_id
string program_name
uint32 line_number
string raw_line
string command
float64[] parameters
float64 feed_rate
float64 spindle_speed
string comment
bool is_rapid
```

#### JobStatus
```
builtin_interfaces/Time timestamp
string job_id
string machine_id
string status
string program_name
uint32 total_lines
uint32 current_line
float64 progress
float64 estimated_remaining_sec
float64 elapsed_sec
string[] warnings
string[] errors
```

#### AnomalyAlert
```
builtin_interfaces/Time timestamp
string machine_id
string anomaly_type
float64 confidence
float64 severity
string[] contributing_factors
float64[] feature_contributions
string recommended_action
bool requires_immediate_stop
```

#### PHMPrediction
```
builtin_interfaces/Time timestamp
string machine_id
string component
string prediction_type
float64 remaining_useful_life_hours
float64 confidence
float64 health_index
string recommended_action
builtin_interfaces/Time predicted_failure_time
float64[] trend_data
```

#### ToolWearEstimate
```
builtin_interfaces/Time timestamp
string machine_id
string tool_id
float64 wear_percentage
float64 remaining_life_minutes
float64 confidence
string wear_type
float64 flank_wear_mm
float64 crater_wear_mm
string recommended_action
```

#### Heartbeat
```
builtin_interfaces/Time timestamp
string node_name
string node_namespace
string criticality
string lifecycle_state
string[] dependencies
float32 cpu_usage
float32 memory_usage
```

#### FleetHealth
```
builtin_interfaces/Time timestamp
uint32 total_nodes
uint32 healthy_nodes
uint32 degraded_nodes
uint32 failed_nodes
uint32 critical_healthy
uint32 critical_total
float64 health_score
string[] failed_node_names
string[] degraded_node_names
```

#### NodeFailure
```
builtin_interfaces/Time timestamp
string node_name
string criticality
string state
float64 time_since_last_heartbeat
string lifecycle_state
string[] dependent_nodes
```

#### RecoveryRequest
```
builtin_interfaces/Time timestamp
string failed_node
string criticality
string[] dependents
string strategy
string reason
uint32 attempt_number
```

#### SecurityAlert
```
builtin_interfaces/Time timestamp
string alert_id
string severity
string category
string source_node
string description
string[] affected_nodes
string recommended_action
bool requires_isolation
float64 confidence
```

#### DeviceTrustStatus
```
builtin_interfaces/Time timestamp
string device_id
string device_type
float64 trust_score
string attestation_status
builtin_interfaces/Time last_attestation
string[] active_policies
bool is_quarantined
```

#### AttestationReport
```
builtin_interfaces/Time timestamp
string device_id
string firmware_hash
string config_hash
bool integrity_verified
string[] violations
float64 trust_score
string report_signature
```

#### SystemKPIs
```
builtin_interfaces/Time timestamp
float64 oee
float64 availability
float64 performance
float64 quality
float64 cpk
float64 mtbf
float64 mttr
float64 energy_efficiency
float64 schedule_adherence
float64 tool_life_utilization
uint32 jobs_completed_today
uint32 jobs_in_progress
uint32 jobs_queued
```

#### TwinSyncStatus
```
builtin_interfaces/Time timestamp
string machine_id
float64 sync_quality
float64 drift_magnitude
float64[] axis_drift
float64 sync_latency_ms
bool correction_active
uint32 corrections_applied
```

#### RobotJointState
```
builtin_interfaces/Time timestamp
string robot_id
float64[] positions
float64[] velocities
float64[] efforts
string gripper_state
string task_state
```

#### KnowledgeUpdate
```
builtin_interfaces/Time timestamp
string update_type
string subject
string predicate
string object_value
float64 confidence
string source
string reasoning
```

#### TaskAnnouncement
```
builtin_interfaces/Time timestamp
string auction_id
string task_type
string job_id
string material
float64 complexity
builtin_interfaces/Time deadline
string[] required_capabilities
float64 estimated_duration
string priority
```

#### AgentBid
```
builtin_interfaces/Time timestamp
string auction_id
string agent_id
float64 proposed_cost
float64 proposed_completion_time
float64 confidence
float64 current_load
string[] capabilities_offered
```

#### TaskAward
```
builtin_interfaces/Time timestamp
string auction_id
string task_type
string awarded_agent_id
string job_id
float64 agreed_cost
float64 agreed_completion_time
```

#### BehaviorTreeStatus
```
builtin_interfaces/Time timestamp
string tree_id
string root_status
string tip_name
string[] active_behaviors
float64 execution_time_sec
uint32 tick_count
```

#### OptimizationAction
```
builtin_interfaces/Time timestamp
string machine_id
string action_type
string parameter_name
float64 old_value
float64 new_value
float64 expected_improvement
string reasoning
float64 confidence
```

#### ManufacturingExperience
```
builtin_interfaces/Time timestamp
string machine_id
string job_id
string material
string operation_type
float64[] process_parameters
float64[] quality_metrics
float64[] sensor_summary
float64 oee_achieved
string outcome
string[] lessons_learned
```

#### FederatedModel
```
builtin_interfaces/Time timestamp
string model_id
string model_type
uint32 round_number
uint32 num_participants
float64[] global_weights
float64 global_loss
float64 convergence_metric
```

#### ModelUpdate
```
builtin_interfaces/Time timestamp
string model_id
string client_id
uint32 round_number
float64[] local_weights
float64 local_loss
uint32 num_samples
float64[] gradient_norms
```

#### DigitalThreadEntry
```
builtin_interfaces/Time timestamp
string entry_id
string job_id
string entry_type
string source_node
string data_json
string[] tags
string previous_entry_id
string hash_value
```

### Services

#### TriggerEStop
```
# Request
string machine_id
string reason
string requesting_node
---
# Response
bool success
string message
```

#### ValidateGCode
```
# Request
string program_content
string machine_id
---
# Response
bool is_valid
string[] errors
string[] warnings
float64 estimated_duration_sec
```

#### GetFleetStatus
```
# Request
string filter_criticality
string filter_state
---
# Response
miracle_msgs/FleetHealth fleet_health
string[] node_details_json
```

#### RegisterDevice
```
# Request
string device_id
string device_type
string[] capabilities
string firmware_version
---
# Response
bool success
string message
float64 initial_trust_score
```

#### RequestAttestation
```
# Request
string device_id
string challenge_nonce
---
# Response
bool success
miracle_msgs/AttestationReport report
```

#### TriggerFailover
```
# Request
string failed_node
string strategy
---
# Response
bool success
string backup_node
string message
```

#### RestoreCheckpoint
```
# Request
string node_name
string checkpoint_id
---
# Response
bool success
string message
builtin_interfaces/Time checkpoint_timestamp
```

#### InjectFault
```
# Request
string target_node
string fault_type
float64 duration_sec
float64 intensity
---
# Response
bool success
string fault_id
string message
```

#### IsolateNode (srv)
```
# Request
string node_id
string reason
---
# Response
bool success
string message
```

#### SPARQLQuery
```
# Request
string query
string graph_name
---
# Response
bool success
string result_json
uint32 num_results
```

#### GOAPPlan
```
# Request
string[] current_state
string[] goal_state
float64 max_planning_time_sec
---
# Response
bool success
string[] action_sequence
float64 total_cost
string plan_explanation
```

#### HTNPlan
```
# Request
string task_name
string[] task_parameters
string[] available_methods
---
# Response
bool success
string[] plan_steps
float64 estimated_duration
string plan_tree_json
```

#### NLPCommand
```
# Request
string natural_language_input
string context_json
---
# Response
bool understood
string interpreted_action
string[] parameters
float64 confidence
string clarification_question
```

#### OptimizeParameters
```
# Request
string machine_id
string job_id
string optimization_target
float64[] current_parameters
string[] parameter_names
---
# Response
bool success
float64[] optimized_parameters
float64 expected_improvement
string reasoning
```

#### SubmitTask
```
# Request
string task_type
string job_id
string material
float64 complexity
string priority
string[] required_capabilities
---
# Response
bool accepted
string auction_id
string message
```

#### RunPrediction (srv)
```
# Request
string machine_id
string scenario_type
float64 prediction_horizon_hours
---
# Response
bool success
string summary
float64 confidence
string detailed_report_json
```

### Actions

#### ExecuteProgram
```
# Goal
string machine_id
string program_name
string program_content
float64[] override_parameters
---
# Result
bool success
string message
float64 total_time_sec
uint32 lines_executed
float64[] quality_metrics
---
# Feedback
uint32 current_line
uint32 total_lines
float64 progress
string current_operation
float64 elapsed_sec
float64 estimated_remaining_sec
```

#### ExecuteJob
```
# Goal
string job_id
string machine_id
string program_name
string material
string priority
---
# Result
bool success
string message
float64 total_time_sec
float64 oee_achieved
float64[] quality_metrics
---
# Feedback
float64 progress
string current_phase
string current_operation
float64 elapsed_sec
float64 estimated_remaining_sec
miracle_msgs/MachineState machine_state
```

#### RunPrediction (action)
```
# Goal
string machine_id
string prediction_type
float64 prediction_horizon_hours
---
# Result
bool success
miracle_msgs/PHMPrediction prediction
string detailed_report_json
---
# Feedback
float64 progress
string current_phase
```

#### IsolateNode (action)
```
# Goal
string target_node
string reason
string isolation_level
---
# Result
bool success
string[] affected_nodes
string message
---
# Feedback
float64 progress
string current_step
```

#### PerformCalibration
```
# Goal
string machine_id
string calibration_type
string[] axes
---
# Result
bool success
float64[] offsets
float64[] accuracies
string report_json
---
# Feedback
float64 progress
string current_step
string current_axis
```

#### TrainRLPolicy
```
# Goal
string policy_name
string environment_id
uint32 num_episodes
float64 learning_rate
---
# Result
bool success
float64 final_reward
float64[] reward_history
string model_path
---
# Feedback
uint32 current_episode
uint32 total_episodes
float64 current_reward
float64 average_reward
```

#### FederatedRound
```
# Goal
string model_id
uint32 round_number
uint32 min_participants
float64 timeout_sec
---
# Result
bool success
uint32 num_participants
float64 global_loss
float64 convergence_metric
---
# Feedback
uint32 updates_received
uint32 updates_expected
float64 elapsed_sec
string current_phase
```

---

## Network Ports

All port defaults are defined in `miracle_ws/config/miracle_defaults.yaml` and can be overridden via environment variables (`MIRACLE_NETWORK_<KEY>`).

| Port | Protocol | Service | Container Mapping | Description |
|------|----------|---------|-------------------|-------------|
| **10000** | TCP | ROS-TCP-Connector | `10000:10000` | Unity-to-ROS2 bridge (ros_tcp_endpoint). Carries all ROS2 topic/service traffic between Unity `MiracleBridge` and the ROS2 system. |
| **9090** | WebSocket | HMI Bridge | `9091:9090` (host 9091) | WebSocket server for the web-based HMI dashboard. Streams machine state, KPIs, fleet health, and anomaly data to browser clients. |
| **1883** | MQTT | Eclipse Mosquitto | `1883:1883` | MQTT broker for Sparkplug B IIoT integration. Used by `sparkplug_bridge` for NBIRTH/DBIRTH/NDATA/DDATA messages. |
| **9092** | TCP | Apache Kafka | `9092:9092` | Kafka broker for enterprise event streaming. Used by `kafka_bridge` to forward machine state, anomalies, job status, and KPIs. |
| **29092** | TCP | Kafka (host) | `29092:29092` | Kafka external listener for host-side clients. |
| **502** | TCP | Modbus TCP | -- | Modbus TCP server for PLC/sensor integration. Used by `modbus_bridge` to read holding registers. |
| **4840** | TCP | OPC-UA | -- | OPC-UA server for industrial data access. Used by `opc_ua_bridge` to read machine state and sensor values. |
| **9100** | HTTP | Prometheus | -- | Node exporter / Prometheus scrape target for `prometheus_exporter` metrics. |
| **9190** | HTTP | Prometheus Server | `9190:9090` (host 9190) | Prometheus server UI and query API. |
| **3001** | HTTP | Grafana | `3001:3000` (host 3001) | Grafana dashboard for visualization of Prometheus metrics and Loki logs. |
| **3100** | HTTP | Loki | -- | Loki log aggregation endpoint (internal). Used by Promtail to push logs. |

**ROS2 DDS Configuration:**

| Setting | Value | Description |
|---------|-------|-------------|
| `ROS_DOMAIN_ID` | 42 | DDS domain isolation for MIRACLE traffic |
| Heartbeat interval | 0.5 s | Default heartbeat publish rate for all lifecycle nodes |
| Keepalive timeout | 10.0 s | Connection keepalive timeout |
| QoS history depth | 10 | Default DDS history depth |
| FastDDS profile | `docker/fastdds_profile.xml` | Custom Fast-DDS transport configuration |

---

## QoS Profiles

Defined in `miracle_ws/src/miracle_core/miracle_core/qos_profiles.py`:

| Profile | Reliability | Durability | History | Depth | Deadline | Use Case |
|---------|------------|------------|---------|-------|----------|----------|
| **sensor_data** | Best-Effort | Volatile | Keep Last | 5 | 100ms | High-frequency sensor streams (IMU, current, audio) |
| **state_data** | Reliable | Transient Local | Keep Last | 1 | -- | Machine state, KPIs, sync status; late joiners get last value |
| **command** | Reliable | Volatile | Keep Last | 10 | -- | G-code blocks, task announcements, optimization actions |
| **alert** | Reliable | Transient Local | Keep All | -- | -- | Anomaly alerts, security alerts, failure notifications |
| **heartbeat** | Best-Effort | Volatile | Keep Last | 1 | -- | Periodic liveness signals; missed beats signal failure |
| **bulk_data** | Reliable | Volatile | Keep Last | 50 | -- | Federated model weights, large dataset transfers |
| **logging** | Reliable | Transient Local | Keep All | -- | -- | Digital thread entries, audit trail records |

---

## Node Summary

### miracle_cnc

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `state_publisher` | `StatePublisherNode` | Yes | -- | Publishes machine state at configurable rate; hosts E-Stop service; supports OPC-UA/Modbus/MTConnect bridge backends |
| `gcode_executor` | `GCodeExecutorNode` | Yes | -- | Parses/executes G-code programs; publishes block-by-block execution; provides validation service and ExecuteProgram action |
| `sensor_fusion` | `SensorFusionNode` | Yes | -- | Fuses IMU, current, audio, and vision sensor streams using time-synchronized subscriptions |
| `local_watchdog` | `LocalWatchdogNode` | Yes | -- | Monitors spindle load and coolant levels; triggers anomaly alerts and E-Stop on safety violations |
| `spc_monitor` | `SPCMonitorNode` | Yes | -- | Statistical Process Control monitoring; publishes alerts when process variables exceed control limits |
| `rosbag_trigger` | `RosbagTriggerNode` | Yes | -- | Triggers rosbag recording on anomaly events for post-incident analysis |

### miracle_ai

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `anomaly_detector` | `AnomalyDetectorNode` | Yes | -- | Ensemble anomaly detection (statistical, autoencoder, isolation forest) on fused sensor data |
| `phm_predictor` | `PHMPredictorNode` | Yes | -- | Prognostics and Health Management; predicts remaining useful life from sensor trends |
| `tool_wear_estimator` | `ToolWearEstimatorNode` | Yes | -- | Estimates tool wear from machine state and sensor data; tracks flank and crater wear |
| `chatter_detector` | `ChatterDetectorNode` | Yes | -- | Detects machining chatter from audio FFT energy ratios in the chatter frequency band |
| `model_manager` | `ModelManagerNode` | Yes | -- | Manages AI model lifecycle (loading, versioning, hot-swapping); publishes model update notifications |

### miracle_bridges

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `opc_ua_bridge` | `OPCUABridgeNode` | Yes | -- | Bridges OPC-UA server to ROS2; publishes machine state and sensor data from OPC-UA node readings |
| `sparkplug_bridge` | `SparkplugBridgeNode` | Yes | -- | Bridges Sparkplug B (MQTT) to ROS2; publishes machine state and sensor data from MQTT payloads |
| `mtconnect_agent` | `MTConnectAgentNode` | Yes | -- | Polls MTConnect agent REST API and publishes machine state |
| `modbus_bridge` | `ModbusBridgeNode` | Yes | -- | Reads Modbus TCP registers and publishes sensor data |
| `kafka_bridge` | `KafkaBridgeNode` | Yes | -- | Forwards machine state, anomalies, job status, and KPIs to Apache Kafka topics |

### miracle_mes

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `job_scheduler` | `JobSchedulerNode` | Yes | -- | Schedules manufacturing jobs; provides SubmitTask service and ExecuteJob action; publishes task announcements for auction |
| `fleet_manager` | `FleetManagerNode` | Yes | -- | Monitors all machines; publishes fleet health; provides GetFleetStatus service |
| `oee_calculator` | `OEECalculatorNode` | Yes | -- | Calculates OEE, availability, performance, quality, and other KPIs from machine state and job status |
| `digital_thread` | `DigitalThreadNode` | Yes | -- | Records tamper-evident digital thread entries (hash-chained) for manufacturing traceability |
| `resource_manager` | `ResourceManagerNode` | Yes | -- | Tracks tooling, materials, and fixtures across machines |

### miracle_scada

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `discovery_server` | `DiscoveryServerNode` | Yes | -- | Device registration and trust score management |
| `traffic_manager` | `TrafficManagerNode` | Yes | -- | Monitors DDS network traffic and bandwidth utilization |
| `alarm_manager` | `AlarmManagerNode` | Yes | -- | Aggregates anomaly and security alerts; manages alarm escalation |
| `historian` | `HistorianNode` | Yes | -- | Persists machine state, anomaly, and job data to time-series storage |
| `hmi_bridge` | `HMIBridgeNode` | Yes | -- | WebSocket bridge for HMI dashboards; forwards state, anomaly, KPI, and health data |
| `prometheus_exporter` | `PrometheusExporterNode` | Yes | -- | Exports machine state, anomaly counts, and KPIs as Prometheus metrics |

### miracle_twin

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `sync_engine` | `SyncEngineNode` | Yes | -- | Synchronizes physical machine state to digital twin; publishes twin state and sync quality |
| `gazebo_bridge` | `GazeboBridgeNode` | Yes | -- | Forwards twin state to Gazebo simulation for physics-based visualization |
| `prediction_runner` | `PredictionRunnerNode` | Yes | -- | Runs predictive what-if scenarios; provides RunPrediction action and service |
| `scenario_manager` | `ScenarioManagerNode` | Yes | -- | Manages simulation scenarios for digital twin experiments |

### miracle_resiliency

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `heartbeat_aggregator` | `HeartbeatAggregatorNode` | Yes | -- | Collects heartbeats from all nodes; detects failures by criticality-based timeouts; publishes fleet health and failure/recovery events |
| `supervisor_root` | `SupervisorRootNode` | Yes | -- | Root supervisor; monitors fleet health and failures; initiates recovery requests |
| `failover_coordinator` | `FailoverCoordinatorNode` | Yes | -- | Manages standby nodes; provides TriggerFailover service for manual/automatic failover |
| `checkpoint_manager` | `CheckpointManagerNode` | Yes | -- | Periodically checkpoints machine and job state; provides RestoreCheckpoint service |
| `recovery_orchestrator` | `RecoveryOrchestratorNode` | Yes | -- | Executes recovery strategies (restart, failover, degrade) from recovery requests |
| `chaos_injector` | `ChaosInjectorNode` | Yes | -- | Injects faults for resilience testing via InjectFault service |

### miracle_security

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `intrusion_detection` | `IntrusionDetectionNode` | Yes | -- | Monitors heartbeat traffic for anomalous patterns; publishes security alerts |
| `attestation_verifier` | `AttestationVerifierNode` | Yes | -- | Verifies device firmware/config integrity; manages trust scores; provides RequestAttestation service |
| `threat_response` | `ThreatResponseNode` | Yes | -- | Responds to security alerts; can isolate compromised nodes via action/service |
| `access_enforcer` | `AccessEnforcerNode` | Yes | -- | Role-based access control; publishes security alerts on policy violations |
| `audit_logger` | `AuditLoggerNode` | Yes | -- | Persists security alerts to tamper-evident audit log files |
| `sros2_manager` | `SROS2ManagerNode` | Yes | -- | Manages SROS2 security policies and key distribution |

### miracle_cognitive

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `knowledge_graph` | `KnowledgeGraphNode` | Yes | -- | RDF-based manufacturing knowledge graph; SPARQL query service; publishes knowledge events |
| `ontology_manager` | `OntologyManagerNode` | Yes | -- | Manages manufacturing ontologies (ISA-95, OPC-UA information models) |
| `reasoning_engine` | `ReasoningEngineNode` | Yes | -- | Rule-based inference over knowledge graph events; publishes derived knowledge |
| `causal_inference` | `CausalInferenceNode` | Yes | -- | Identifies root causes of anomalies using causal models; publishes causal inferences |
| `goal_manager` | `GoalManagerNode` | Yes | -- | Tracks manufacturing goals (OEE, quality, safety); evaluates against live KPIs |
| `behavior_tree_executor` | `BehaviorTreeExecutorNode` | Yes | -- | Ticks behavior trees for autonomous manufacturing decisions based on machine state and anomalies |
| `htn_planner` | `HTNPlannerNode` | Yes | Medium | Hierarchical Task Network planner for manufacturing operations |
| `goap_planner` | `GOAPPlannerNode` | Yes | Medium | Goal-Oriented Action Planning for adaptive manufacturing |
| `agent_registry` | `AgentRegistryNode` | Yes | High | Tracks registered agents and their capabilities via heartbeats |
| `task_allocator` | `TaskAllocatorNode` | Yes | High | Allocates tasks to agents using auction/round-robin/least-loaded strategies |
| `auction_manager` | `AuctionManagerNode` | Yes | Medium | Manages task auctions; publishes announcements and awards |
| `coalition_former` | `CoalitionFormerNode` | Yes | -- | Forms multi-agent coalitions for complex tasks |
| `consensus_protocol` | `ConsensusProtocolNode` | Yes | -- | Distributed consensus for multi-agent coordination |
| `rl_optimizer` | `RLOptimizerNode` | Yes | -- | Reinforcement learning for process parameter optimization; provides OptimizeParameters service and TrainRLPolicy action |
| `federated_coordinator` | `FederatedCoordinatorNode` | Yes | -- | Coordinates federated learning rounds across machines; provides FederatedRound action |
| `federated_client` | `FederatedClientNode` | Yes | -- | Local federated learning client; trains on local data and publishes model updates |
| `self_optimizer` | `SelfOptimizerNode` | Yes | -- | Continuously optimizes system parameters based on KPI trends |
| `self_healer` | `SelfHealerNode` | Yes | -- | Self-healing responses to fleet health degradation and anomalies |
| `self_configurer` | `SelfConfigurerNode` | Yes | -- | Dynamic self-configuration of system parameters |
| `self_protector` | `SelfProtectorNode` | Yes | High | Adjusts system posture in response to security alerts |
| `nlp_interface` | `NLPInterfaceNode` | Yes | -- | Natural language command processing for operator interaction |
| `explanation_generator` | `ExplanationGeneratorNode` | Yes | -- | Generates human-readable explanations for AI decisions |
| `human_escalation` | `HumanEscalationNode` | Yes | High | Manages human-in-the-loop escalation for critical decisions |

### miracle_unity_bridge

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `unity_endpoint` | `UnityEndpointConfig` | No (plain Node) | -- | ROS-TCP-Connector endpoint; bridges ROS2 topics to/from Unity over TCP; monitors Unity connection health |

### miracle_bringup

| Node | Class | Lifecycle | Criticality | Description |
|------|-------|-----------|-------------|-------------|
| `lifecycle_autostart` | `LifecycleAutoStartNode` | No (plain Node) | -- | Automatically transitions lifecycle nodes through configure/activate on startup |

### Unity (MiracleBridge)

The Unity-side `MiracleBridge` (C#) connects via ROS-TCP-Connector to the `unity_endpoint` node.

**Subscriptions (Unity receives):**
| Topic | Message Type |
|-------|-------------|
| `/miracle/{machine_id}/state` | `MachineStateMsg` |
| `/miracle/{machine_id}/anomaly` | `AnomalyAlertMsg` |
| `/miracle/{machine_id}/tool_wear` | `ToolWearEstimateMsg` |
| `/miracle/{machine_id}/job_status` | `JobStatusMsg` |
| `/miracle/twin/sync_status` | `TwinSyncStatusMsg` |
| `/miracle/system_kpis` | `SystemKPIsMsg` |
| `/miracle/cognitive/task_awards` | `TaskAwardMsg` |
| `/miracle/security/alerts` | `SecurityAlertMsg` |
| `/miracle/robots/joint_states` | `RobotJointStateMsg` |

**Publications (Unity sends):**
| Topic | Message Type |
|-------|-------------|
| `/miracle/unity/heartbeat` | `HeartbeatMsg` |
| `/miracle/robots/joint_states` | `RobotJointStateMsg` |

**Service Clients (Unity calls):**
| Service | Type |
|---------|------|
| `/miracle/{machine_id}/trigger_estop` | `TriggerEStop` |
| `/miracle/mes/validate_gcode` | `ValidateGCode` |
| `/miracle/fleet/get_status` | `GetFleetStatus` |

Unity also supports multi-machine monitoring via `additionalMachineIds`, subscribing to per-machine topics for each configured machine ID.

---

## Namespace Convention

All MIRACLE topics follow the pattern:

```
/miracle/{scope}/{topic_name}
```

Where `{scope}` is one of:
- **`{machine_id}`** -- per-machine data (e.g., `cnc1`, `cnc2`)
- **`heartbeats`** -- global heartbeat bus
- **`system_kpis`** -- system-wide metrics
- **`twin`** -- digital twin subsystem
- **`resiliency`** -- resiliency/health subsystem
- **`security`** -- security subsystem
- **`cognitive`** -- cognitive/AI subsystem
- **`mes`** -- manufacturing execution subsystem
- **`scada`** -- SCADA subsystem
- **`unity`** -- Unity digital twin client
- **`robots`** -- robotic systems
- **`ai`** -- AI model management

Nodes using `MiracleLifecycleNode` with relative topic names (e.g., `'state'`) are automatically namespaced under `/miracle/{machine_id}/` at launch via ROS2 namespace remapping.
