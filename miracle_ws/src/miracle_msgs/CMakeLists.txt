cmake_minimum_required(VERSION 3.8)
project(miracle_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/Heartbeat.msg"
  "msg/MachineState.msg"
  "msg/SensorData.msg"
  "msg/FusedSensorData.msg"
  "msg/GCodeBlock.msg"
  "msg/JobStatus.msg"
  "msg/AnomalyAlert.msg"
  "msg/PHMPrediction.msg"
  "msg/ToolWearEstimate.msg"
  "msg/SecurityAlert.msg"
  "msg/DeviceTrustStatus.msg"
  "msg/AttestationReport.msg"
  "msg/FleetHealth.msg"
  "msg/NodeFailure.msg"
  "msg/RecoveryRequest.msg"
  "msg/KnowledgeUpdate.msg"
  "msg/TaskAnnouncement.msg"
  "msg/AgentBid.msg"
  "msg/TaskAward.msg"
  "msg/BehaviorTreeStatus.msg"
  "msg/ManufacturingExperience.msg"
  "msg/FederatedModel.msg"
  "msg/ModelUpdate.msg"
  "msg/OptimizationAction.msg"
  "msg/SystemKPIs.msg"
  "msg/DigitalThreadEntry.msg"
  "msg/TwinSyncStatus.msg"
  "msg/RobotJointState.msg"
  "msg/CorrelatedAlert.msg"
  "msg/FeedOverride.msg"
  "msg/CuttingState.msg"
  "msg/Explanation.msg"
  "msg/MaterialBatch.msg"
  "msg/PartRecord.msg"
  "msg/StabilityRecommendation.msg"
  "msg/InferredAction.msg"
)

set(srv_files
  "srv/RegisterDevice.srv"
  "srv/GetFleetStatus.srv"
  "srv/ValidateGCode.srv"
  "srv/TriggerEStop.srv"
  "srv/RequestAttestation.srv"
  "srv/TriggerFailover.srv"
  "srv/RestoreCheckpoint.srv"
  "srv/SPARQLQuery.srv"
  "srv/SubmitTask.srv"
  "srv/OptimizeParameters.srv"
  "srv/GOAPPlan.srv"
  "srv/HTNPlan.srv"
  "srv/NLPCommand.srv"
  "srv/InjectFault.srv"
  "srv/RunPrediction.srv"
  "srv/IsolateNode.srv"
)

set(action_files
  "action/ExecuteProgram.action"
  "action/RunPrediction.action"
  "action/PerformCalibration.action"
  "action/IsolateNode.action"
  "action/ExecuteJob.action"
  "action/TrainRLPolicy.action"
  "action/FederatedRound.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
