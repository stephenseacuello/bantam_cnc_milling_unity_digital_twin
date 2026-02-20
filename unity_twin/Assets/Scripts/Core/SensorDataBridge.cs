using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Subscribes to raw micro-ROS sensor topics published by ESP32 and STM32
    /// microcontrollers via the micro-ROS agent → ROS2 DDS → ros_tcp_endpoint pipeline.
    ///
    /// Sensor topics (per machine):
    ///   ESP32 (WiFi/UDP):
    ///     /miracle/{machine_id}/raw_sensor/vibration   Float32MultiArray [x,y,z] 1000Hz
    ///     /miracle/{machine_id}/raw_sensor/current      Float32           500Hz
    ///   STM32 (Serial/UART):
    ///     /miracle/{machine_id}/raw_sensor/temperature   Float32          10Hz
    ///     /miracle/{machine_id}/raw_sensor/acoustic      Float32          1000Hz
    ///     /miracle/{machine_id}/raw_sensor/coolant_flow  Float32          10Hz
    ///
    /// Stores latest readings as public properties. Other components (CuttingSimulationManager,
    /// DashboardOverlay) can poll these values or subscribe to the OnSensorUpdate event.
    /// </summary>
    public class SensorDataBridge : MonoBehaviour
    {
        [Header("Machine")]
        [SerializeField] private string machineId = "cnc1";

        [Header("Connection")]
        [Tooltip("Subscribe to raw sensor topics. Disable if no micro-ROS agent is running.")]
        [SerializeField] private bool enableSubscriptions = true;

        // --- Latest Sensor Readings ---

        /// <summary>3-axis vibration acceleration (mm/s²). Updated at ~1000Hz from ESP32.</summary>
        public Vector3 Vibration { get; private set; }

        /// <summary>Spindle motor current (Amps). Updated at ~500Hz from ESP32 ACS712.</summary>
        public float SpindleCurrent { get; private set; }

        /// <summary>Workpiece/tool temperature (°C). Updated at ~10Hz from STM32 thermocouple.</summary>
        public float SensorTemperature { get; private set; }

        /// <summary>Acoustic emission RMS level (arbitrary units). Updated at ~1000Hz from STM32.</summary>
        public float AcousticEmission { get; private set; }

        /// <summary>Coolant flow rate (L/min). Updated at ~10Hz from STM32 flow sensor.</summary>
        public float CoolantFlow { get; private set; }

        /// <summary>True when at least one sensor topic has received data.</summary>
        public bool HasSensorData { get; private set; }

        /// <summary>Time since last sensor message on any topic.</summary>
        public float TimeSinceLastMessage { get; private set; }

        /// <summary>Bitmask of which sensors have sent data. Bits: 0=vibration, 1=current, 2=temp, 3=acoustic, 4=coolant.</summary>
        public byte SensorHealthMask { get; private set; }

        /// <summary>Fired whenever any sensor reading is updated. Parameter: this bridge instance.</summary>
        public event Action<SensorDataBridge> OnSensorUpdate;

        // Timestamps for health tracking
        private float lastVibrationTime;
        private float lastCurrentTime;
        private float lastTemperatureTime;
        private float lastAcousticTime;
        private float lastCoolantTime;
        private float lastAnyMessageTime;

        private ROSConnection ros;
        private bool subscribed;

        void Start()
        {
            if (!enableSubscriptions) return;

            // Check for local test mode
            bool localTestActive = false;
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (mb.enabled && mb.GetType().Name == "LocalCNCTestDriver")
                {
                    localTestActive = true;
                    break;
                }
            }

            if (localTestActive)
            {
                Debug.Log("[SensorDataBridge] LocalCNCTestDriver detected — sensor subscriptions deferred until ROS connected.");
            }

            TrySubscribe();
        }

        private void TrySubscribe()
        {
            if (subscribed) return;

            ros = ROSConnection.GetOrCreateInstance();
            if (ros == null) return;

            string prefix = $"/miracle/{machineId}/raw_sensor";

            // ESP32 sensors
            ros.Subscribe<Float32MultiArrayMsg>($"{prefix}/vibration", OnVibration);
            ros.Subscribe<Float32Msg>($"{prefix}/current", OnCurrent);

            // STM32 sensors
            ros.Subscribe<Float32Msg>($"{prefix}/temperature", OnTemperature);
            ros.Subscribe<Float32Msg>($"{prefix}/acoustic", OnAcoustic);
            ros.Subscribe<Float32Msg>($"{prefix}/coolant_flow", OnCoolantFlow);

            subscribed = true;
            Debug.Log($"[SensorDataBridge] Subscribed to 5 raw sensor topics for machine '{machineId}'.");
        }

        void Update()
        {
            if (HasSensorData)
                TimeSinceLastMessage = Time.time - lastAnyMessageTime;

            // Update health mask (sensor considered healthy if data within last 2 seconds)
            float now = Time.time;
            byte mask = 0;
            if (now - lastVibrationTime < 2f) mask |= 0x01;
            if (now - lastCurrentTime < 2f) mask |= 0x02;
            if (now - lastTemperatureTime < 2f) mask |= 0x04;
            if (now - lastAcousticTime < 2f) mask |= 0x08;
            if (now - lastCoolantTime < 2f) mask |= 0x10;
            SensorHealthMask = mask;
        }

        // --- ROS Callbacks ---

        private void OnVibration(Float32MultiArrayMsg msg)
        {
            if (msg.data != null && msg.data.Length >= 3)
                Vibration = new Vector3(msg.data[0], msg.data[1], msg.data[2]);
            lastVibrationTime = Time.time;
            MarkReceived();
        }

        private void OnCurrent(Float32Msg msg)
        {
            SpindleCurrent = msg.data;
            lastCurrentTime = Time.time;
            MarkReceived();
        }

        private void OnTemperature(Float32Msg msg)
        {
            SensorTemperature = msg.data;
            lastTemperatureTime = Time.time;
            MarkReceived();
        }

        private void OnAcoustic(Float32Msg msg)
        {
            AcousticEmission = msg.data;
            lastAcousticTime = Time.time;
            MarkReceived();
        }

        private void OnCoolantFlow(Float32Msg msg)
        {
            CoolantFlow = msg.data;
            lastCoolantTime = Time.time;
            MarkReceived();
        }

        private void MarkReceived()
        {
            HasSensorData = true;
            lastAnyMessageTime = Time.time;
            OnSensorUpdate?.Invoke(this);
        }

        /// <summary>
        /// Compute vibration magnitude (RMS of 3-axis acceleration).
        /// Useful for chatter detection threshold comparison.
        /// </summary>
        public float VibrationMagnitude => Vibration.magnitude;

        /// <summary>
        /// Estimate cutting power from spindle current (simplified: P = V*I*efficiency).
        /// Uses typical Bantam/CR-1 48V BLDC motor characteristics.
        /// </summary>
        public float EstimatedPowerFromCurrent(float motorVoltage = 48f, float efficiency = 0.85f)
        {
            return SpindleCurrent * motorVoltage * efficiency;
        }

        /// <summary>
        /// Check if vibration magnitude exceeds chatter threshold.
        /// Default threshold calibrated for desktop CNC + accelerometer on spindle housing.
        /// </summary>
        public bool IsChatterDetected(float thresholdMmPerS2 = 500f)
        {
            return VibrationMagnitude > thresholdMmPerS2;
        }
    }
}
