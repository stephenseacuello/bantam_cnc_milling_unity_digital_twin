using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;
using MiracleTwin.CNC;
using MiracleTwin.Cutting;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Main dashboard HUD overlay using UI Toolkit.
    /// Shows machine status, forces, tool wear, temperature, OEE, connection state,
    /// FPS counter, and total cutting time.
    /// </summary>
    public class DashboardOverlay : MonoBehaviour
    {
        [Header("Event Channels")]
        [SerializeField] private MachineStateEventSO machineStateEvent;
        [SerializeField] private SystemKPIsEventSO systemKPIsEvent;
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;

        [Header("UI Document")]
        [SerializeField] private UIDocument uiDocument;

        [Header("References")]
        [SerializeField] private CuttingSimulationManager cuttingSimManager;

        private VisualElement root;
        private Label statusLabel, rpmLabel, feedLabel, lineLabel;
        private Label programLabel, loadLabel;
        private Label posXLabel, posYLabel, posZLabel;
        private Label fxLabel, fyLabel, fzLabel;
        private Label wearLabel, vbLabel, wearLifeLabel;
        private Label toolTempLabel, interfaceTempLabel;
        private Label oeeLabel, availLabel, perfLabel, qualLabel;
        private Label roughnessRaLabel, roughnessGradeLabel;
        private Label simTimeLabel, connectionLabel;
        private Label fpsLabel, cuttingTimeLabel;
        private Label sensorVibLabel, sensorCurrentLabel, sensorTempLabel;
        private Label sensorAcousticLabel, sensorCoolantLabel, sensorStatusLabel;
        private VisualElement connectionDot;
        private ProgressBar wearBar;

        // Sensor data bridge (auto-discovered at runtime)
        private SensorDataBridge sensorBridge;

        // Machine selector (auto-discovered) — used to filter MachineState messages
        private CNCMachineSelector machineSelector;

        private bool isVisible = true;
        private float updateTimer;
        private const float UI_UPDATE_INTERVAL = 1f / 15f; // 15 Hz

        // Cached state
        private string machineStatus = "IDLE";
        private string currentProgram = "--";
        private float spindleRPM, feedRate, spindleLoad;
        private float posX, posY, posZ;
        private uint currentLine;
        private float fx, fy, fz;
        private float wearPct, vb;
        private float toolTemp, interfaceTemp;
        private float oee, availability, performance, quality;
        private float roughnessRa;
        private string roughnessGrade = "--";

        // FPS tracking
        private float fpsAccumulator;
        private int fpsFrameCount;
        private float fpsDisplayValue;
        private float fpsUpdateTimer;
        private const float FPS_UPDATE_INTERVAL = 0.5f;

        void OnEnable()
        {
            if (machineStateEvent != null) machineStateEvent.Register(OnMachineState);
            if (systemKPIsEvent != null) systemKPIsEvent.Register(OnKPIs);
            if (cuttingStateEvent != null) cuttingStateEvent.Register(OnCuttingState);

            if (InputManager.Instance != null)
                InputManager.Instance.OnToggleHUD += ToggleVisibility;
        }

        void OnDisable()
        {
            if (machineStateEvent != null) machineStateEvent.Unregister(OnMachineState);
            if (systemKPIsEvent != null) systemKPIsEvent.Unregister(OnKPIs);
            if (cuttingStateEvent != null) cuttingStateEvent.Unregister(OnCuttingState);

            if (InputManager.Instance != null)
                InputManager.Instance.OnToggleHUD -= ToggleVisibility;
        }

        void Start()
        {
            if (uiDocument == null) return;
            root = uiDocument.rootVisualElement;
            BindElements();
            CreateDynamicElements();

            // Auto-discover sensor bridge and machine selector
            sensorBridge = FindFirstObjectByType<SensorDataBridge>();
            machineSelector = FindFirstObjectByType<CNCMachineSelector>();
        }

        private void BindElements()
        {
            statusLabel = root.Q<Label>("status-value");
            programLabel = root.Q<Label>("program-value");
            rpmLabel = root.Q<Label>("rpm-value");
            feedLabel = root.Q<Label>("feed-value");
            loadLabel = root.Q<Label>("load-value");
            lineLabel = root.Q<Label>("line-value");
            posXLabel = root.Q<Label>("pos-x-value");
            posYLabel = root.Q<Label>("pos-y-value");
            posZLabel = root.Q<Label>("pos-z-value");
            fxLabel = root.Q<Label>("force-x-value");
            fyLabel = root.Q<Label>("force-y-value");
            fzLabel = root.Q<Label>("force-z-value");
            wearLabel = root.Q<Label>("wear-pct-value");
            vbLabel = root.Q<Label>("wear-vb-value");
            wearLifeLabel = root.Q<Label>("wear-life-value");
            wearBar = root.Q<ProgressBar>("wear-bar");
            toolTempLabel = root.Q<Label>("tool-temp-value");
            interfaceTempLabel = root.Q<Label>("interface-temp-value");
            oeeLabel = root.Q<Label>("oee-value");
            availLabel = root.Q<Label>("availability-value");
            perfLabel = root.Q<Label>("performance-value");
            qualLabel = root.Q<Label>("quality-value");
            roughnessRaLabel = root.Q<Label>("roughness-ra-value");
            roughnessGradeLabel = root.Q<Label>("roughness-grade-value");
            simTimeLabel = root.Q<Label>("sim-time-value");
            connectionLabel = root.Q<Label>("connection-status");

            // Sensor data labels (micro-ROS)
            sensorVibLabel = root.Q<Label>("sensor-vibration-value");
            sensorCurrentLabel = root.Q<Label>("sensor-current-value");
            sensorTempLabel = root.Q<Label>("sensor-temp-value");
            sensorAcousticLabel = root.Q<Label>("sensor-acoustic-value");
            sensorCoolantLabel = root.Q<Label>("sensor-coolant-value");
            sensorStatusLabel = root.Q<Label>("sensor-status-value");

            // Try to find elements that may already exist in the UXML
            fpsLabel = root.Q<Label>("fps-value");
            cuttingTimeLabel = root.Q<Label>("cutting-time-value");
            connectionDot = root.Q<VisualElement>("connection-dot");
        }

        /// <summary>
        /// Create dynamic UI elements (FPS counter, connection dot, cutting time)
        /// if they are not already defined in the UXML template.
        /// </summary>
        private void CreateDynamicElements()
        {
            if (root == null) return;

            // Connection status dot (green/red circle)
            if (connectionDot == null)
            {
                connectionDot = new VisualElement();
                connectionDot.name = "connection-dot";
                connectionDot.style.width = 10;
                connectionDot.style.height = 10;
                connectionDot.style.borderTopLeftRadius = 5;
                connectionDot.style.borderTopRightRadius = 5;
                connectionDot.style.borderBottomLeftRadius = 5;
                connectionDot.style.borderBottomRightRadius = 5;
                connectionDot.style.backgroundColor = Color.red;
                connectionDot.style.marginRight = 6;

                // Insert next to connection label if it exists
                if (connectionLabel != null && connectionLabel.parent != null)
                {
                    connectionLabel.parent.Insert(
                        connectionLabel.parent.IndexOf(connectionLabel), connectionDot);
                }
                else
                {
                    root.Add(connectionDot);
                }
            }

            // FPS counter label (top-right corner)
            if (fpsLabel == null)
            {
                fpsLabel = new Label("FPS: --");
                fpsLabel.name = "fps-value";
                fpsLabel.style.position = Position.Absolute;
                fpsLabel.style.top = 8;
                fpsLabel.style.right = 8;
                fpsLabel.style.fontSize = 12;
                fpsLabel.style.color = Color.white;
                fpsLabel.style.unityFontStyleAndWeight = FontStyle.Bold;
                root.Add(fpsLabel);
            }

            // Total cutting time label
            if (cuttingTimeLabel == null)
            {
                cuttingTimeLabel = new Label("Cut: 00:00:00");
                cuttingTimeLabel.name = "cutting-time-value";
                cuttingTimeLabel.style.position = Position.Absolute;
                cuttingTimeLabel.style.top = 24;
                cuttingTimeLabel.style.right = 8;
                cuttingTimeLabel.style.fontSize = 11;
                cuttingTimeLabel.style.color = new Color(0.8f, 0.9f, 1f, 1f);
                root.Add(cuttingTimeLabel);
            }
        }

        void Update()
        {
            // FPS calculation (every frame, displayed at lower rate)
            fpsAccumulator += Time.unscaledDeltaTime;
            fpsFrameCount++;
            fpsUpdateTimer += Time.unscaledDeltaTime;
            if (fpsUpdateTimer >= FPS_UPDATE_INTERVAL)
            {
                fpsDisplayValue = fpsFrameCount / fpsAccumulator;
                fpsAccumulator = 0;
                fpsFrameCount = 0;
                fpsUpdateTimer = 0;
            }

            // Throttled UI update
            updateTimer += Time.unscaledDeltaTime;
            if (updateTimer < UI_UPDATE_INTERVAL) return;
            updateTimer = 0;

            UpdateUI();
        }

        private void UpdateUI()
        {
            if (root == null) return;

            statusLabel?.SetText(machineStatus);
            programLabel?.SetText(currentProgram);
            rpmLabel?.SetText($"{spindleRPM:F0}");
            feedLabel?.SetText($"{feedRate:F0}");
            loadLabel?.SetText($"{spindleLoad:F1}%");
            lineLabel?.SetText($"{currentLine}");
            posXLabel?.SetText($"{posX:F1} mm");
            posYLabel?.SetText($"{posY:F1} mm");
            posZLabel?.SetText($"{posZ:F1} mm");

            fxLabel?.SetText($"{fx:F1} N");
            fyLabel?.SetText($"{fy:F1} N");
            fzLabel?.SetText($"{fz:F1} N");

            wearLabel?.SetText($"{wearPct:F1}%");
            vbLabel?.SetText($"{vb:F3} mm");
            if (wearBar != null) wearBar.value = wearPct;

            toolTempLabel?.SetText($"{toolTemp:F1} °C");
            interfaceTempLabel?.SetText($"{interfaceTemp:F1} °C");

            roughnessRaLabel?.SetText($"{roughnessRa:F2} µm");
            roughnessGradeLabel?.SetText(roughnessGrade);

            oeeLabel?.SetText($"{oee:F1}%");
            availLabel?.SetText($"{availability:F1}%");
            perfLabel?.SetText($"{performance:F1}%");
            qualLabel?.SetText($"{quality:F1}%");

            // Simulation time
            if (SimulationClock.Instance != null)
                simTimeLabel?.SetText(SimulationClock.Instance.FormatSimTime());

            // Connection status indicator (green/red dot + text)
            bool connected = MiracleBridge.Instance != null && MiracleBridge.Instance.IsConnected;
            connectionLabel?.SetText(connected ? "ROS Connected" : "Disconnected");
            if (connectionDot != null)
            {
                connectionDot.style.backgroundColor = connected
                    ? new Color(0.2f, 0.9f, 0.2f, 1f)  // green
                    : new Color(0.9f, 0.2f, 0.2f, 1f);  // red
            }

            // Raw sensor data (micro-ROS)
            if (sensorBridge != null && sensorBridge.HasSensorData)
            {
                sensorVibLabel?.SetText($"{sensorBridge.VibrationMagnitude:F0} mm/s²");
                sensorCurrentLabel?.SetText($"{sensorBridge.SpindleCurrent:F2} A");
                sensorTempLabel?.SetText($"{sensorBridge.SensorTemperature:F1} °C");
                sensorAcousticLabel?.SetText($"{sensorBridge.AcousticEmission:F0}");
                sensorCoolantLabel?.SetText($"{sensorBridge.CoolantFlow:F1} L/min");

                int activeSensors = 0;
                byte mask = sensorBridge.SensorHealthMask;
                for (int i = 0; i < 5; i++)
                    if ((mask & (1 << i)) != 0) activeSensors++;
                sensorStatusLabel?.SetText($"{activeSensors}/5 Online");
                if (sensorStatusLabel != null)
                    sensorStatusLabel.style.color = activeSensors >= 3
                        ? new Color(0.4f, 1f, 0.4f)
                        : new Color(1f, 1f, 0.4f);
            }
            else
            {
                sensorStatusLabel?.SetText("No Data");
                if (sensorStatusLabel != null)
                    sensorStatusLabel.style.color = new Color(0.6f, 0.6f, 0.6f);
            }

            // FPS counter
            if (fpsLabel != null)
            {
                fpsLabel.text = $"FPS: {fpsDisplayValue:F0}";
                // Color-code: green > 60, yellow 30-60, red < 30
                if (fpsDisplayValue >= 60f)
                    fpsLabel.style.color = new Color(0.4f, 1f, 0.4f);
                else if (fpsDisplayValue >= 30f)
                    fpsLabel.style.color = new Color(1f, 1f, 0.4f);
                else
                    fpsLabel.style.color = new Color(1f, 0.3f, 0.3f);
            }

            // Total cutting time display
            if (cuttingTimeLabel != null && cuttingSimManager != null)
            {
                float totalSec = cuttingSimManager.TotalCuttingTime;
                var ts = System.TimeSpan.FromSeconds(totalSec);
                cuttingTimeLabel.text = $"Cut: {ts.Hours:D2}:{ts.Minutes:D2}:{ts.Seconds:D2}";
            }
        }

        private void OnMachineState(RosMessageTypes.Miracle.MachineStateMsg msg)
        {
            // Only display data for the currently selected machine
            if (machineSelector != null && machineSelector.ActiveMachine != null
                && !string.IsNullOrEmpty(msg.machine_id)
                && msg.machine_id != machineSelector.ActiveMachine.MachineId)
                return;

            machineStatus = msg.status ?? "UNKNOWN";
            currentProgram = msg.current_program ?? "--";
            spindleRPM = (float)msg.spindle_speed;
            feedRate = (float)msg.feed_rate;
            spindleLoad = (float)msg.spindle_load;
            currentLine = msg.current_line;

            if (msg.axis_positions != null && msg.axis_positions.Length >= 3)
            {
                posX = (float)msg.axis_positions[0];
                posY = (float)msg.axis_positions[1];
                posZ = (float)msg.axis_positions[2];
            }
        }

        private void OnKPIs(RosMessageTypes.Miracle.SystemKPIsMsg msg)
        {
            oee = (float)msg.oee;
            availability = (float)msg.availability;
            performance = (float)msg.performance;
            quality = (float)msg.quality;
        }

        private void OnCuttingState(CuttingStateData state)
        {
            fx = state.forceFx;
            fy = state.forceFy;
            fz = state.forceFz;
            wearPct = state.wearPercentage;
            vb = state.flankWearVB;
            toolTemp = state.toolTemperature;
            interfaceTemp = state.interfaceTemperature;
            roughnessRa = state.surfaceRoughnessRa;
            roughnessGrade = state.surfaceGrade ?? "--";
        }

        public void ToggleVisibility()
        {
            isVisible = !isVisible;
            if (root != null)
                root.style.display = isVisible ? DisplayStyle.Flex : DisplayStyle.None;
        }

        /// <summary>
        /// Called by CNCMachineSelector when the active machine changes.
        /// Swaps the CuttingSimulationManager reference so dashboard displays
        /// the correct machine's cutting data.
        /// </summary>
        public void SetActiveCuttingSimManager(CuttingSimulationManager newManager)
        {
            cuttingSimManager = newManager;
        }
    }
}
