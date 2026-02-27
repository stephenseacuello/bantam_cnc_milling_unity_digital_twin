using System;
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

        /// <summary>True when the ROS TCP bridge connection is established and healthy.</summary>
        public bool IsConnected { get; private set; }
        public string MachineId => machineId;

        /// <summary>
        /// Fired whenever the connection status transitions (connected/disconnected).
        /// Parameter: true = connected, false = disconnected.
        /// </summary>
        public event Action<bool> ConnectionStatusChanged;

        private ROSConnection ros;
        private float lastHeartbeatTime;
        private bool registered;
        private const float HEARTBEAT_INTERVAL = 1f;

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
        }

        private void RegisterPublishers()
        {
            if (enableHeartbeat)
                ros.RegisterPublisher<HeartbeatMsg>("/miracle/unity/heartbeat");
            ros.RegisterPublisher<RobotJointStateMsg>("/miracle/robots/joint_states");
        }

        private void RegisterServices()
        {
            ros.RegisterRosService<TriggerEStopRequest, TriggerEStopResponse>(
                $"/miracle/{machineId}/trigger_estop");
            ros.RegisterRosService<ValidateGCodeRequest, ValidateGCodeResponse>(
                "/miracle/mes/validate_gcode");
            ros.RegisterRosService<GetFleetStatusRequest, GetFleetStatusResponse>(
                "/miracle/fleet/get_status");
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
            onMachineState?.Raise(msg);
        }
        private void OnAnomaly(AnomalyAlertMsg msg) => onAnomalyAlert?.Raise(msg);
        private void OnToolWear(ToolWearEstimateMsg msg) => onToolWear?.Raise(msg);
        private void OnTwinSync(TwinSyncStatusMsg msg) => onTwinSync?.Raise(msg);
        private void OnKPIs(SystemKPIsMsg msg) => onSystemKPIs?.Raise(msg);
        private void OnJobStatus(JobStatusMsg msg) => onJobStatus?.Raise(msg);
        private void OnTaskAward(TaskAwardMsg msg) => onTaskAward?.Raise(msg);
        private void OnSecurityAlert(SecurityAlertMsg msg) => onSecurityAlert?.Raise(msg);

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

        void OnDestroy()
        {
            if (Instance == this) Instance = null;
            RosMachineStateActive = false;
        }
    }
}
