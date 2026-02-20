/**
 * MIRACLE ROS2 Topic Name Constants
 *
 * Centralised registry of all ROS2 topic names used by the MIRACLE
 * manufacturing system.  The "+" wildcard in topic patterns represents a
 * per-machine segment (e.g. /miracle/cnc_01/state).
 */

// ---------------------------------------------------------------------------
// Machine-scoped topics  (replace {machineId} at runtime)
// ---------------------------------------------------------------------------

/** Pattern: /miracle/{machineId}/state  -- MachineState */
export const MACHINE_STATE_PATTERN = '/miracle/+/state';
export const machineStateTopic = (machineId) => `/miracle/${machineId}/state`;

/** Pattern: /miracle/{machineId}/sensor_data  -- SensorData */
export const SENSOR_DATA_PATTERN = '/miracle/+/sensor_data';
export const sensorDataTopic = (machineId) => `/miracle/${machineId}/sensor_data`;

/** Pattern: /miracle/{machineId}/anomaly  -- AnomalyAlert */
export const ANOMALY_PATTERN = '/miracle/+/anomaly';
export const anomalyTopic = (machineId) => `/miracle/${machineId}/anomaly`;

/** Pattern: /miracle/{machineId}/job_status  -- JobStatus */
export const JOB_STATUS_PATTERN = '/miracle/+/job_status';
export const jobStatusTopic = (machineId) => `/miracle/${machineId}/job_status`;

/** Pattern: /miracle/{machineId}/tool_wear  -- ToolWearEstimate */
export const TOOL_WEAR_PATTERN = '/miracle/+/tool_wear';
export const toolWearTopic = (machineId) => `/miracle/${machineId}/tool_wear`;

/** Pattern: /miracle/{machineId}/phm_prediction  -- PHMPrediction */
export const PHM_PREDICTION_PATTERN = '/miracle/+/phm_prediction';
export const phmPredictionTopic = (machineId) => `/miracle/${machineId}/phm_prediction`;

// ---------------------------------------------------------------------------
// System-wide topics
// ---------------------------------------------------------------------------

/** /miracle/system_kpis  -- SystemKPIs (OEE, availability, etc.) */
export const SYSTEM_KPIS = '/miracle/system_kpis';

/** /miracle/mes/fleet_health  -- FleetHealth */
export const FLEET_HEALTH = '/miracle/mes/fleet_health';

/** /miracle/heartbeats  -- Heartbeat */
export const HEARTBEATS = '/miracle/heartbeats';

// ---------------------------------------------------------------------------
// Alert / Security topics
// ---------------------------------------------------------------------------

/** /miracle/security/alerts  -- SecurityAlert */
export const SECURITY_ALERTS = '/miracle/security/alerts';

// ---------------------------------------------------------------------------
// Cognitive / AI topics
// ---------------------------------------------------------------------------

/** /miracle/cognitive/task_announcements  -- TaskAnnouncement */
export const TASK_ANNOUNCEMENTS = '/miracle/cognitive/task_announcements';

/** /miracle/cognitive/bids  -- AgentBid */
export const AGENT_BIDS = '/miracle/cognitive/bids';

/** /miracle/cognitive/task_awards  -- TaskAward */
export const TASK_AWARDS = '/miracle/cognitive/task_awards';

/** /miracle/cognitive/knowledge_events  -- KnowledgeUpdate */
export const KNOWLEDGE_EVENTS = '/miracle/cognitive/knowledge_events';

/** /miracle/cognitive/global_model  -- FederatedModel */
export const GLOBAL_MODEL = '/miracle/cognitive/global_model';

/** /miracle/cognitive/model_updates  -- ModelUpdate */
export const MODEL_UPDATES = '/miracle/cognitive/model_updates';

// ---------------------------------------------------------------------------
// Resiliency topics
// ---------------------------------------------------------------------------

/** /miracle/resiliency/node_failure  -- NodeFailure */
export const NODE_FAILURE = '/miracle/resiliency/node_failure';

/** /miracle/resiliency/recovery_request  -- RecoveryRequest */
export const RECOVERY_REQUEST = '/miracle/resiliency/recovery_request';

// ---------------------------------------------------------------------------
// Service names
// ---------------------------------------------------------------------------

/** /miracle/mes/submit_task */
export const MES_SUBMIT_TASK = '/miracle/mes/submit_task';

/** /miracle/mes/get_fleet_status */
export const GET_FLEET_STATUS = '/miracle/mes/get_fleet_status';

/** /miracle/security/isolate_node */
export const ISOLATE_NODE = '/miracle/security/isolate_node';

// ---------------------------------------------------------------------------
// Default machine IDs (for demo / development)
// ---------------------------------------------------------------------------

export const DEFAULT_MACHINE_IDS = ['cnc1', 'cnc2', 'cnc3'];

// ---------------------------------------------------------------------------
// Rosbridge WebSocket defaults
// ---------------------------------------------------------------------------

export const ROSBRIDGE_URL = 'ws://localhost:9090';
