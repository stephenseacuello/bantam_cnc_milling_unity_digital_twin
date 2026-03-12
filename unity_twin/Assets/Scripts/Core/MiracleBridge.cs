using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Miracle;
using RosMessageTypes.Std;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Main bridge connecting Unity to the MIRACLE ROS2 system.
    /// Manages all subscriptions, publications, and service registrations.
    /// Uses ScriptableObject event channels for decoupled communication.
    ///
    /// Connection lifecycle is delegated to ROSConnection (ROS-TCP-Connector).
    /// MiracleBridge registers topics once and tracks health via ROSConnection state.
    /// </summary>
    public class MiracleBridge : MonoBehaviour
    {
        public static MiracleBridge Instance { get; private set; }

        [Header("Connection Settings")]
        [SerializeField] private string rosBridgeIP = "127.0.0.1";
        [SerializeField] private int rosBridgePort = 10000;
        [SerializeField] private string machineId = "cnc1";
        [Tooltip("Additional machine IDs to subscribe to for multi-machine monitoring.")]
        [SerializeField] private string[] additionalMachineIds = new string[0];
        [Tooltip("Enable heartbeat publishing. Disable if ros_tcp_endpoint doesn't have miracle_msgs.")]
        [SerializeField] private bool enableHeartbeat = false;
        [Tooltip("Enable ROS service registration (E-Stop, ValidateGCode, FleetStatus). Disable if no ROS2 service servers are running.")]
        [SerializeField] private bool enableServiceRegistration = false;

        [Header("TLS Settings")]
        [Tooltip("Require TLS for ROS bridge connection. Falls back to plaintext with warning if disabled.")]
        [SerializeField] private bool requireTLS = false;
        [Tooltip("Path to CA certificate for TLS verification (PEM format).")]
        [SerializeField] private string caCertPath = "";
        [Tooltip("Path to client certificate for mutual TLS (PEM format).")]
        [SerializeField] private string clientCertPath = "";

        [Header("Event Channels")]
        [SerializeField] private MachineStateEventSO onMachineState;
        [SerializeField] private AnomalyAlertEventSO onAnomalyAlert;
        [SerializeField] private ToolWearEventSO onToolWear;
        [SerializeField] private TwinSyncEventSO onTwinSync;
        [SerializeField] private SystemKPIsEventSO onSystemKPIs;
        [SerializeField] private JobStatusEventSO onJobStatus;
        [SerializeField] private TaskAwardEventSO onTaskAward;
        [SerializeField] private SecurityAlertEventSO onSecurityAlert;
        [SerializeField] private CuttingStateEventSO onCuttingState;
        [SerializeField] private RobotJointStateEventSO onRobotJointState;
        [SerializeField] private FeedOverrideEventSO onFeedOverride;
        [SerializeField] private StabilityRecommendationEventSO onStabilityRecommendation;

        /// <summary>True when the ROS TCP bridge connection is established and healthy.</summary>
        public bool IsConnected { get; private set; }
        public string MachineId => machineId;

        /// <summary>
        /// Fired whenever the connection status transitions (connected/disconnected).
        /// Parameter: true = connected, false = disconnected.
        /// </summary>
        public event Action<bool> ConnectionStatusChanged;

        /// <summary>
        /// Fired when a feed override message is received from the adaptive controller.
        /// </summary>
        public event Action<float, float, string> FeedOverrideReceived;

        private ROSConnection ros;
        private float lastHeartbeatTime;
        private bool registered;
        private const float HEARTBEAT_INTERVAL = 1f;
        private DataRecorder dataRecorder;

        void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;
            DontDestroyOnLoad(transform.root.gameObject);
        }

        void Start()
        {
            // Always attempt ROS connection — LocalCNCTestDriver provides fallback CNC data,
            // but real ROS connection is still needed for robot joint commands, sensor data,
            // and bidirectional communication when available.
            ConnectToROS();
            dataRecorder = Object.FindFirstObjectByType<DataRecorder>();
        }

        void Update()
        {
            // Track connection status from ROSConnection's internal state
            if (ros != null)
            {
                bool tcpHealthy = ros.HasConnectionThread && !ros.HasConnectionError;
                if (tcpHealthy != IsConnected)
                    SetConnectionStatus(tcpHealthy);
            }

            // Periodic heartbeat publishing (disabled by default until ROS2 side has miracle_msgs)
            if (enableHeartbeat && IsConnected && Time.time - lastHeartbeatTime >= HEARTBEAT_INTERVAL)
            {
                PublishHeartbeat();
                lastHeartbeatTime = Time.time;
            }
        }

        private void ConnectToROS()
        {
            try
            {
                ros = ROSConnection.GetOrCreateInstance();
                ros.Connect(rosBridgeIP, rosBridgePort);

                // TLS note: When requireTLS is enabled, connect to stunnel port (10001)
                // instead of direct ros_tcp_endpoint port (10000).
                // Stunnel handles TLS termination transparently — no SslStream needed
                // in Unity since ROS-TCP-Connector uses its own TCP layer.
                if (requireTLS)
                {
                    // Stunnel port for TLS-terminated connections
                    if (rosBridgePort == 10000)
                    {
                        Debug.LogWarning("[MiracleBridge] requireTLS is enabled but port is 10000. " +
                            "Set port to 10001 to use TLS bridge (stunnel).");
                    }
                    Debug.Log($"[MiracleBridge] TLS mode: connecting via stunnel on port {rosBridgePort}");
                }

                // Only register once — ROSConnection persists subscriptions across reconnects
                if (!registered)
                {
                    RegisterSubscriptions();
                    RegisterPublishers();
                    if (enableServiceRegistration)
                        RegisterServices();
                    registered = true;
                }

                Debug.Log($"[MiracleBridge] ROS2 bridge initialized at {rosBridgeIP}:{rosBridgePort} " +
                    "(connection will establish when ros_tcp_endpoint is running).");
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[MiracleBridge] ROS2 bridge init failed (non-fatal): {ex.Message}. " +
                    "LocalCNCTestDriver will provide fallback data.");
            }
        }

        private void SetConnectionStatus(bool connected)
        {
            if (IsConnected == connected) return;
            IsConnected = connected;
            ConnectionStatusChanged?.Invoke(connected);
            Debug.Log($"[MiracleBridge] Connection status: {(connected ? "CONNECTED" : "DISCONNECTED")}");
        }

        /// <summary>FNV-1a 32-bit hash for topic name recording.</summary>
        private static uint HashTopic(string topic)
        {
            uint hash = 2166136261u;
            foreach (char c in topic)
            {
                hash ^= (byte)c;
                hash *= 16777619u;
            }
            return hash;
        }

        private void TryRecord(string topic, Unity.Robotics.ROSTCPConnector.MessageGeneration.Message msg)
        {
            if (dataRecorder == null || !dataRecorder.IsRecording) return;
            try
            {
                byte[] data = msg.Serialize();
                dataRecorder.RecordMessage(HashTopic(topic), data);
            }
            catch (System.Exception ex)
            {
                Debug.LogWarning($"[MiracleBridge] Recording failed for {topic}: {ex.Message}");
            }
        }

        private void RegisterSubscriptions()
        {
            // Per-machine subscriptions (primary machine)
            SubscribeMachineTopics(machineId);

            // Subscribe to additional machine IDs for multi-machine monitoring
            foreach (string additionalId in additionalMachineIds)
            {
                if (!string.IsNullOrEmpty(additionalId) && additionalId != machineId)
                {
                    SubscribeMachineTopics(additionalId);
                    Debug.Log($"[MiracleBridge] Subscribed to additional machine: {additionalId}");
                }
            }

            // System-wide subscriptions
            ros.Subscribe<TwinSyncStatusMsg>(
                "/miracle/twin/sync_status", OnTwinSync);
            ros.Subscribe<SystemKPIsMsg>(
                "/miracle/system_kpis", OnKPIs);
            ros.Subscribe<TaskAwardMsg>(
                "/miracle/cognitive/task_awards", OnTaskAward);
            ros.Subscribe<SecurityAlertMsg>(
                "/miracle/security/alerts", OnSecurityAlert);
            ros.Subscribe<RobotJointStateMsg>(
                "/miracle/robots/joint_states", OnRobotJointState);

            Debug.Log("[MiracleBridge] All subscriptions registered");
        }

        private void SubscribeMachineTopics(string targetMachineId)
        {
            ros.Subscribe<MachineStateMsg>(
                $"/miracle/{targetMachineId}/state", OnMachineState);
            ros.Subscribe<AnomalyAlertMsg>(
                $"/miracle/{targetMachineId}/anomaly", OnAnomaly);
            ros.Subscribe<ToolWearEstimateMsg>(
                $"/miracle/{targetMachineId}/tool_wear", OnToolWear);
            ros.Subscribe<JobStatusMsg>(
                $"/miracle/{targetMachineId}/job_status", OnJobStatus);
            ros.Subscribe<FeedOverrideMsg>(
                $"/miracle/{targetMachineId}/feed_override", OnFeedOverride);
            ros.Subscribe<CuttingStateMsg>(
                $"/miracle/{targetMachineId}/cutting_state", OnCuttingState);
        }

        private void RegisterPublishers()
        {
            if (enableHeartbeat)
                ros.RegisterPublisher<HeartbeatMsg>("/miracle/unity/heartbeat");
            ros.RegisterPublisher<RobotJointStateMsg>("/miracle/robots/joint_states");
            ros.RegisterPublisher<FeedOverrideMsg>(
                $"/miracle/{machineId}/feed_override_cmd");
            ros.RegisterPublisher<StabilityRecommendationMsg>(
                $"/miracle/{machineId}/stability_recommendation");
        }

        private void RegisterServices()
        {
            ros.RegisterRosService<TriggerEStopRequest, TriggerEStopResponse>(
                $"/miracle/{machineId}/trigger_estop");
            ros.RegisterRosService<ValidateGCodeRequest, ValidateGCodeResponse>(
                "/miracle/mes/validate_gcode");
            ros.RegisterRosService<GetFleetStatusRequest, GetFleetStatusResponse>(
                "/miracle/fleet/get_status");
            ros.RegisterRosService<RunPredictionRequest, RunPredictionResponse>(
                "/miracle/twin/run_prediction");
        }

        // --- Subscription Callbacks ---
        /// <summary>True once any ROS MachineState message has been received.
        /// LocalCNCTestDriver checks this to stop publishing fake data.</summary>
        public static bool RosMachineStateActive { get; private set; }

        private void OnMachineState(MachineStateMsg msg)
        {
            if (!RosMachineStateActive)
            {
                RosMachineStateActive = true;
                Debug.Log("[MiracleBridge] ROS MachineState received — fake CNC data will stop.");
            }
            TryRecord($"/miracle/{msg.machine_id}/state", msg);
            onMachineState?.Raise(msg);
        }
        private void OnAnomaly(AnomalyAlertMsg msg)
        {
            TryRecord($"/miracle/{machineId}/anomaly", msg);
            onAnomalyAlert?.Raise(msg);
        }
        private void OnToolWear(ToolWearEstimateMsg msg) => onToolWear?.Raise(msg);
        private void OnTwinSync(TwinSyncStatusMsg msg) => onTwinSync?.Raise(msg);
        private void OnKPIs(SystemKPIsMsg msg)
        {
            TryRecord("/miracle/system_kpis", msg);
            onSystemKPIs?.Raise(msg);
        }
        private void OnJobStatus(JobStatusMsg msg)
        {
            TryRecord($"/miracle/{machineId}/job_status", msg);
            onJobStatus?.Raise(msg);
        }
        private void OnTaskAward(TaskAwardMsg msg) => onTaskAward?.Raise(msg);
        private void OnSecurityAlert(SecurityAlertMsg msg)
        {
            TryRecord("/miracle/security/alerts", msg);
            onSecurityAlert?.Raise(msg);
        }

        private void OnRobotJointState(RobotJointStateMsg msg)
        {
            onRobotJointState?.Raise(new RobotJointStateData
            {
                robotId = msg.robot_id ?? "",
                positions = msg.positions ?? new double[6],
                velocities = msg.velocities ?? new double[6],
                gripperState = msg.gripper_state ?? "OPEN"
            });
        }

        private void OnFeedOverride(FeedOverrideMsg msg)
        {
            TryRecord($"/miracle/{machineId}/feed_override", msg);
            onFeedOverride?.Raise(msg);
            FeedOverrideReceived?.Invoke(
                (float)msg.feed_override_pct,
                (float)msg.spindle_override_pct,
                msg.reason ?? "");
        }

        private void OnCuttingState(CuttingStateMsg msg)
        {
            TryRecord($"/miracle/{machineId}/cutting_state", msg);
            var pos = msg.tool_position ?? new double[3];
            var dir = msg.tool_direction ?? new double[3];
            onCuttingState?.Raise(new CuttingStateData
            {
                spindleRPM = (float)msg.spindle_rpm,
                feedRate = (float)msg.feed_rate,
                toolPosition = new UnityEngine.Vector3((float)pos[0], (float)pos[1], (float)pos[2]),
                toolDirection = new UnityEngine.Vector3((float)dir[0], (float)dir[1], (float)dir[2]),
                axialDepth = (float)msg.axial_depth,
                radialDepth = (float)msg.radial_depth,
                toolDiameter = (float)msg.tool_diameter,
                isCutting = msg.is_cutting,
                currentGCodeLine = (int)msg.current_gcode_line,
                forceFx = (float)msg.force_fx,
                forceFy = (float)msg.force_fy,
                forceFz = (float)msg.force_fz,
                powerWatts = (float)msg.power_watts,
                torqueNm = (float)msg.torque_nm,
                mrr = (float)msg.mrr,
                toolTemperature = (float)msg.tool_temperature,
                interfaceTemperature = (float)msg.interface_temperature,
                flankWearVB = (float)msg.flank_wear_vb,
                wearPercentage = (float)msg.wear_percentage,
                chipThicknessRatio = (float)msg.chip_thickness_ratio,
                shearAngleDeg = (float)msg.shear_angle_deg,
                chipCurlRadius = (float)msg.chip_curl_radius,
                chipVelocity = (float)msg.chip_velocity,
                surfaceRoughnessRa = (float)msg.surface_roughness_ra,
                surfaceRoughnessRz = (float)msg.surface_roughness_rz,
                surfaceGrade = msg.surface_grade ?? ""
            });
        }

        // --- Publishers ---
        private void PublishHeartbeat()
        {
            if (ros == null) return;

            var msg = new HeartbeatMsg
            {
                header = new HeaderMsg(),
                node_name = "unity_digital_twin",
                criticality = "LOW",
                lifecycle_state = "ACTIVE",
                cpu_usage = 0f,
                memory_usage = 0f,
            };

            ros.Publish("/miracle/unity/heartbeat", msg);
        }

        /// <summary>Publish robot joint state to ROS2.</summary>
        public void PublishRobotJointState(RobotJointStateData data)
        {
            if (ros == null) return;

            var msg = new RobotJointStateMsg
            {
                timestamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg(),
                robot_id = data.robotId ?? "",
                positions = data.positions ?? new double[6],
                velocities = data.velocities ?? new double[6],
                efforts = new double[6],
                gripper_state = data.gripperState ?? "OPEN",
                task_state = "ACTIVE"
            };

            ros.Publish("/miracle/robots/joint_states", msg);
        }

        /// <summary>Publish a stability recommendation to ROS2 and raise the event channel.</summary>
        public void PublishStabilityRecommendation(StabilityRecommendationMsg msg)
        {
            if (ros == null) return;

            string topic = $"/miracle/{msg.machine_id}/stability_recommendation";
            ros.Publish(topic, msg);
            TryRecord(topic, msg);
            onStabilityRecommendation?.Raise(msg);
        }

        // --- Service Calls ---
        public void CallEStop(string reason, Action<TriggerEStopResponse> callback)
        {
            if (!IsConnected)
            {
                Debug.LogWarning("[MiracleBridge] Cannot call E-Stop: not connected.");
                return;
            }

            var request = new TriggerEStopRequest
            {
                machine_id = machineId,
                reason = reason,
                requesting_node = "unity_digital_twin"
            };
            ros.SendServiceMessage<TriggerEStopResponse>(
                $"/miracle/{machineId}/trigger_estop", request, callback);
        }

        public void CallValidateGCode(string program, Action<ValidateGCodeResponse> callback)
        {
            if (!IsConnected)
            {
                Debug.LogWarning("[MiracleBridge] Cannot validate G-Code: not connected.");
                return;
            }

            var request = new ValidateGCodeRequest
            {
                program_content = program,
                machine_id = machineId
            };
            ros.SendServiceMessage<ValidateGCodeResponse>(
                "/miracle/mes/validate_gcode", request, callback);
        }

        // --- What-If Prediction ---

        /// <summary>
        /// Result of a what-if prediction, returned from the prediction service
        /// or computed locally as a fallback.
        /// </summary>
        public class PredictionResult
        {
            public float BaselineForceN;
            public float PredictedForceN;
            public float ForceDeltaPct;
            public float BaselineToolLifeMin;
            public float PredictedToolLifeMin;
            public float ToolLifeDeltaPct;
            public float BaselineRa;
            public float PredictedRa;
            public string QualityChange;
            public string BaselineChatterRisk;
            public string PredictedChatterRisk;
            public float Confidence;
            public bool FromService;
        }

        /// <summary>
        /// Request a what-if prediction via the ROS prediction service.
        /// Falls back to null callback if service unavailable.
        /// </summary>
        /// <param name="feedOverridePct">Feed override percentage (e.g. 80 = 80%).</param>
        /// <param name="spindleOverridePct">Spindle RPM override percentage.</param>
        /// <param name="callback">Callback invoked on main thread with result, or null if service unavailable.</param>
        public void RequestPrediction(float feedOverridePct, float spindleOverridePct, Action<PredictionResult> callback)
        {
            if (!IsConnected || !enableServiceRegistration)
            {
                Debug.Log("[MiracleBridge] Prediction service unavailable, returning null for fallback.");
                callback?.Invoke(null);
                return;
            }

            var request = new RunPredictionRequest
            {
                machine_id = machineId,
                scenario_type = $"what_if:feed={feedOverridePct},spindle={spindleOverridePct}",
                prediction_horizon_hours = 1.0
            };

            try
            {
                ros.SendServiceMessage<RunPredictionResponse>(
                    "/miracle/twin/run_prediction", request, response =>
                    {
                        if (response == null || !response.success)
                        {
                            callback?.Invoke(null);
                            return;
                        }

                        // Parse the detailed_report_json for prediction values
                        var result = ParsePredictionResponse(response, feedOverridePct, spindleOverridePct);
                        callback?.Invoke(result);
                    });
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[MiracleBridge] Prediction service call failed: {ex.Message}");
                callback?.Invoke(null);
            }
        }

        private PredictionResult ParsePredictionResponse(RunPredictionResponse response, float feedPct, float spindlePct)
        {
            var result = new PredictionResult
            {
                Confidence = (float)response.confidence,
                FromService = true
            };

            // Try to parse JSON report for detailed values
            try
            {
                string json = response.detailed_report_json ?? "";
                // Simple JSON value extraction (avoids JsonUtility dependency for flat dicts)
                result.BaselineForceN = ExtractJsonFloat(json, "baseline_force", 145f);
                result.PredictedForceN = ExtractJsonFloat(json, "predicted_force", 145f * feedPct / 100f);
                result.BaselineToolLifeMin = ExtractJsonFloat(json, "baseline_life_min", 32f);
                result.PredictedToolLifeMin = ExtractJsonFloat(json, "predicted_life_min", 32f);
                result.BaselineRa = ExtractJsonFloat(json, "baseline_ra", 0.8f);
                result.PredictedRa = ExtractJsonFloat(json, "predicted_ra", 0.8f);
                result.BaselineChatterRisk = ExtractJsonString(json, "baseline_chatter", "LOW");
                result.PredictedChatterRisk = ExtractJsonString(json, "predicted_chatter", "LOW");
            }
            catch
            {
                // If JSON parsing fails, set reasonable defaults
                result.BaselineForceN = 145f;
                result.PredictedForceN = 145f * feedPct / 100f;
                result.BaselineToolLifeMin = 32f;
                result.PredictedToolLifeMin = 32f;
                result.BaselineRa = 0.8f;
                result.PredictedRa = 0.8f;
                result.BaselineChatterRisk = "LOW";
                result.PredictedChatterRisk = "LOW";
            }

            // Compute deltas
            if (result.BaselineForceN > 0)
                result.ForceDeltaPct = (result.PredictedForceN - result.BaselineForceN) / result.BaselineForceN * 100f;
            if (result.BaselineToolLifeMin > 0)
                result.ToolLifeDeltaPct = (result.PredictedToolLifeMin - result.BaselineToolLifeMin) / result.BaselineToolLifeMin * 100f;

            result.QualityChange = result.PredictedRa < result.BaselineRa ? "improved"
                : result.PredictedRa > result.BaselineRa ? "degraded"
                : "unchanged";

            return result;
        }

        private static float ExtractJsonFloat(string json, string key, float fallback)
        {
            string pattern = $"\"{key}\"";
            int idx = json.IndexOf(pattern, StringComparison.Ordinal);
            if (idx < 0) return fallback;
            idx = json.IndexOf(':', idx + pattern.Length);
            if (idx < 0) return fallback;
            int start = idx + 1;
            int end = start;
            while (end < json.Length && json[end] != ',' && json[end] != '}') end++;
            if (float.TryParse(json.Substring(start, end - start).Trim(), System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out float val))
                return val;
            return fallback;
        }

        private static string ExtractJsonString(string json, string key, string fallback)
        {
            string pattern = $"\"{key}\"";
            int idx = json.IndexOf(pattern, StringComparison.Ordinal);
            if (idx < 0) return fallback;
            idx = json.IndexOf(':', idx + pattern.Length);
            if (idx < 0) return fallback;
            int qStart = json.IndexOf('"', idx + 1);
            if (qStart < 0) return fallback;
            int qEnd = json.IndexOf('"', qStart + 1);
            if (qEnd < 0) return fallback;
            return json.Substring(qStart + 1, qEnd - qStart - 1);
        }

        /// <summary>
        /// Publish a feed/spindle override command to the adaptive controller.
        /// </summary>
        /// <param name="feedPct">Feed override percentage (100 = programmed).</param>
        /// <param name="spindlePct">Spindle RPM override percentage (100 = programmed).</param>
        /// <param name="reason">Human-readable reason for the override.</param>
        public void PublishFeedOverride(float feedPct, float spindlePct, string reason)
        {
            if (ros == null)
            {
                Debug.LogWarning("[MiracleBridge] Cannot publish feed override: no ROS connection.");
                return;
            }

            var msg = new FeedOverrideMsg
            {
                timestamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg(),
                machine_id = machineId,
                feed_override_pct = feedPct,
                spindle_override_pct = spindlePct,
                reason = reason ?? "",
                confidence = 1.0,
                revert_after_sec = 0.0
            };

            string topic = $"/miracle/{machineId}/feed_override_cmd";
            ros.Publish(topic, msg);
            TryRecord(topic, msg);
            Debug.Log($"[MiracleBridge] Published feed override: feed={feedPct}%, spindle={spindlePct}%, reason={reason}");
        }

#if UNITY_EDITOR
        void OnValidate()
        {
            if (onMachineState == null) Debug.LogWarning("[MiracleBridge] onMachineState event channel not assigned.", this);
            if (onAnomalyAlert == null) Debug.LogWarning("[MiracleBridge] onAnomalyAlert event channel not assigned.", this);
            if (onToolWear == null) Debug.LogWarning("[MiracleBridge] onToolWear event channel not assigned.", this);
            if (onTwinSync == null) Debug.LogWarning("[MiracleBridge] onTwinSync event channel not assigned.", this);
            if (onSystemKPIs == null) Debug.LogWarning("[MiracleBridge] onSystemKPIs event channel not assigned.", this);
            if (onJobStatus == null) Debug.LogWarning("[MiracleBridge] onJobStatus event channel not assigned.", this);
            if (onTaskAward == null) Debug.LogWarning("[MiracleBridge] onTaskAward event channel not assigned.", this);
            if (onSecurityAlert == null) Debug.LogWarning("[MiracleBridge] onSecurityAlert event channel not assigned.", this);
            if (onCuttingState == null) Debug.LogWarning("[MiracleBridge] onCuttingState event channel not assigned.", this);
            if (onRobotJointState == null) Debug.LogWarning("[MiracleBridge] onRobotJointState event channel not assigned.", this);
            if (onFeedOverride == null) Debug.LogWarning("[MiracleBridge] onFeedOverride event channel not assigned.", this);
            if (onStabilityRecommendation == null) Debug.LogWarning("[MiracleBridge] onStabilityRecommendation event channel not assigned.", this);
            if (requireTLS && rosBridgePort == 10000)
                Debug.LogWarning("[MiracleBridge] TLS enabled but port is 10000 (plaintext). Use 10001 for TLS.", this);
        }
#endif

        void OnDestroy()
        {
            if (Instance == this) Instance = null;
            RosMachineStateActive = false;
        }
    }
}
