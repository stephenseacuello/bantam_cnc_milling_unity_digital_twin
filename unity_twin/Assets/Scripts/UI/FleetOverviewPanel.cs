using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Fleet overview panel showing a grid of machine cards.
    /// Each card displays: status indicator, multi-metric sparklines (load, temp, power, wear),
    /// wear bar, program progress, and alert badge count.
    /// Click a card to switch the dashboard to that machine's detailed view.
    /// </summary>
    public class FleetOverviewPanel : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private MachineStateEventSO machineStateEvent;

        [Header("Event Channels")]
        [SerializeField] private AnomalyAlertEventSO onAnomalyAlert;
        [SerializeField] private ToolWearEventSO onToolWear;
        [SerializeField] private JobStatusEventSO onJobStatus;
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;

        [Header("Fleet Settings")]
        [SerializeField] private string[] machineIds = { "cnc1", "cnc2", "cnc3" };
        [SerializeField] private int sparklineSamples = 60;

        [Header("Severity Thresholds (0-1 normalized)")]
        [SerializeField] private float warningThreshold = 0.70f;
        [SerializeField] private float criticalThreshold = 0.90f;

        /// <summary>Fired when user clicks a machine card. Parameter is machine_id.</summary>
        public event Action<string> OnMachineSelected;

        private VisualElement fleetPanel;
        private bool isVisible;
        private readonly Dictionary<string, MachineCardData> cardData = new();
        private readonly Dictionary<string, VisualElement> cardElements = new();

        /// <summary>Which metric(s) to display on the sparkline.</summary>
        public enum SparklineMode
        {
            SpindleLoad,
            Temperature,
            Power,
            Wear,
            All
        }

        private static readonly Color SpindleLoadColor = new(0.3f, 0.9f, 1f, 0.9f);   // cyan
        private static readonly Color TemperatureColor = new(1f, 0.65f, 0.15f, 0.9f);  // orange
        private static readonly Color PowerColor = new(1f, 0.95f, 0.2f, 0.9f);         // yellow
        private static readonly Color WearColor = new(0.95f, 0.25f, 0.25f, 0.9f);      // red

        private static readonly Color StatusGreen = new(0.2f, 0.8f, 0.3f);
        private static readonly Color StatusYellow = new(0.9f, 0.7f, 0.1f);
        private static readonly Color StatusRed = new(0.9f, 0.2f, 0.2f);
        private static readonly Color StatusGray = new(0.5f, 0.5f, 0.6f);

        private class MachineCardData
        {
            public string machineId;
            public string status = "IDLE";
            public float spindleLoad;
            public float wearPercent;
            public float programProgress;
            public int alertCount;
            public bool hasActiveAnomaly;
            public SparklineMode sparklineMode = SparklineMode.SpindleLoad;
            public Color statusColor = new(0.5f, 0.5f, 0.6f);

            // Multi-metric history arrays (ring buffers)
            public readonly float[] spindleLoadHistory;
            public readonly float[] temperatureHistory;
            public readonly float[] powerHistory;
            public readonly float[] wearHistory;
            public int historyIndex;
            public int historyCount;

            // Latest metric values for severity calculation
            public float latestTemperature;
            public float latestPower;
            public float latestWear;

            public MachineCardData(int samples)
            {
                spindleLoadHistory = new float[samples];
                temperatureHistory = new float[samples];
                powerHistory = new float[samples];
                wearHistory = new float[samples];
            }

            public void PushSample(float load, float temp, float power, float wear)
            {
                spindleLoadHistory[historyIndex] = load;
                temperatureHistory[historyIndex] = temp;
                powerHistory[historyIndex] = power;
                wearHistory[historyIndex] = wear;
                historyIndex = (historyIndex + 1) % spindleLoadHistory.Length;
                if (historyCount < spindleLoadHistory.Length) historyCount++;
            }

            public void PushLoadSample(float load)
            {
                spindleLoadHistory[historyIndex] = load;
                temperatureHistory[historyIndex] = latestTemperature;
                powerHistory[historyIndex] = latestPower;
                wearHistory[historyIndex] = latestWear;
                historyIndex = (historyIndex + 1) % spindleLoadHistory.Length;
                if (historyCount < spindleLoadHistory.Length) historyCount++;
            }

            /// <summary>Get ordered samples from the ring buffer for the given metric.</summary>
            public float[] GetOrdered(float[] ring)
            {
                if (historyCount == 0) return Array.Empty<float>();
                var result = new float[historyCount];
                int start = (historyCount < ring.Length) ? 0 : historyIndex;
                for (int i = 0; i < historyCount; i++)
                    result[i] = ring[(start + i) % ring.Length];
                return result;
            }
        }

        void Start()
        {
            if (uiDocument == null) return;

            fleetPanel = uiDocument.rootVisualElement.Q<VisualElement>("fleet-overview-panel");
            if (fleetPanel == null) return;

            fleetPanel.style.display = DisplayStyle.None;
            isVisible = false;

            // Initialize card data for each machine
            foreach (var id in machineIds)
            {
                cardData[id] = new MachineCardData(sparklineSamples) { machineId = id };
            }

            BuildCards();
        }

        void OnEnable()
        {
            if (machineStateEvent != null)
                machineStateEvent.Register(OnMachineStateUpdate);
            if (onAnomalyAlert != null)
                onAnomalyAlert.Register(OnAnomalyAlert);
            if (onToolWear != null)
                onToolWear.Register(OnToolWearUpdate);
            if (onJobStatus != null)
                onJobStatus.Register(OnJobStatusUpdate);
            if (cuttingStateEvent != null)
                cuttingStateEvent.Register(OnCuttingStateUpdate);
        }

        void OnDisable()
        {
            if (machineStateEvent != null)
                machineStateEvent.Unregister(OnMachineStateUpdate);
            if (onAnomalyAlert != null)
                onAnomalyAlert.Unregister(OnAnomalyAlert);
            if (onToolWear != null)
                onToolWear.Unregister(OnToolWearUpdate);
            if (onJobStatus != null)
                onJobStatus.Unregister(OnJobStatusUpdate);
            if (cuttingStateEvent != null)
                cuttingStateEvent.Unregister(OnCuttingStateUpdate);
        }

        /// <summary>Toggle fleet panel visibility.</summary>
        public void ToggleVisibility()
        {
            isVisible = !isVisible;
            if (fleetPanel != null)
                fleetPanel.style.display = isVisible ? DisplayStyle.Flex : DisplayStyle.None;
        }

        /// <summary>Show fleet panel.</summary>
        public void Show()
        {
            isVisible = true;
            if (fleetPanel != null)
                fleetPanel.style.display = DisplayStyle.Flex;
        }

        /// <summary>Hide fleet panel.</summary>
        public void Hide()
        {
            isVisible = false;
            if (fleetPanel != null)
                fleetPanel.style.display = DisplayStyle.None;
        }

        private void BuildCards()
        {
            var grid = fleetPanel.Q<VisualElement>("fleet-grid");
            if (grid == null) return;

            grid.Clear();

            foreach (var id in machineIds)
            {
                var card = CreateMachineCard(id);
                cardElements[id] = card;
                grid.Add(card);
            }

            // Fleet metrics row
            var metricsRow = new VisualElement();
            metricsRow.AddToClassList("fleet-metrics-row");

            var avgOEE = new Label("Avg OEE: --");
            avgOEE.name = "fleet-avg-oee";
            avgOEE.AddToClassList("fleet-metric-label");
            metricsRow.Add(avgOEE);

            var worstPerf = new Label("Worst: --");
            worstPerf.name = "fleet-worst-performer";
            worstPerf.AddToClassList("fleet-metric-label");
            metricsRow.Add(worstPerf);

            grid.Add(metricsRow);
        }

        private VisualElement CreateMachineCard(string machineId)
        {
            var card = new VisualElement();
            card.AddToClassList("fleet-card");
            card.name = $"fleet-card-{machineId}";

            // Header with machine ID and status dot
            var header = new VisualElement();
            header.AddToClassList("fleet-card-header");

            var statusDot = new VisualElement();
            statusDot.name = $"fleet-status-{machineId}";
            statusDot.AddToClassList("fleet-status-dot");
            header.Add(statusDot);

            var idLabel = new Label(machineId.ToUpper());
            idLabel.AddToClassList("fleet-card-id");
            header.Add(idLabel);

            var alertBadge = new Label("0");
            alertBadge.name = $"fleet-alerts-{machineId}";
            alertBadge.AddToClassList("fleet-alert-badge");
            header.Add(alertBadge);

            card.Add(header);

            // Status label
            var statusLabel = new Label("IDLE");
            statusLabel.name = $"fleet-machine-status-{machineId}";
            statusLabel.AddToClassList("fleet-status-label");
            card.Add(statusLabel);

            // Sparkline container
            var sparkline = new VisualElement();
            sparkline.name = $"fleet-sparkline-{machineId}";
            sparkline.AddToClassList("fleet-sparkline");
            sparkline.generateVisualContent += (ctx) => DrawSparkline(ctx, machineId);
            card.Add(sparkline);

            // Sparkline mode toggle button row
            var modeRow = new VisualElement();
            modeRow.AddToClassList("fleet-sparkline-mode-row");
            string[] modeLabels = { "L", "T", "P", "W", "A" };
            SparklineMode[] modes = {
                SparklineMode.SpindleLoad, SparklineMode.Temperature,
                SparklineMode.Power, SparklineMode.Wear, SparklineMode.All
            };
            Color[] modeColors = { SpindleLoadColor, TemperatureColor, PowerColor, WearColor, new Color(0.7f, 0.7f, 0.8f) };

            for (int i = 0; i < modeLabels.Length; i++)
            {
                var btn = new Button();
                btn.text = modeLabels[i];
                btn.name = $"fleet-sparkmode-{machineId}-{i}";
                btn.AddToClassList("fleet-sparkline-mode-btn");
                btn.style.color = modeColors[i];
                var capturedMode = modes[i];
                var capturedId = machineId;
                btn.clicked += () =>
                {
                    if (cardData.TryGetValue(capturedId, out var d))
                    {
                        d.sparklineMode = capturedMode;
                        UpdateModeButtonHighlight(capturedId);
                    }
                };
                modeRow.Add(btn);
            }
            card.Add(modeRow);

            // Wear bar
            var wearRow = new VisualElement();
            wearRow.AddToClassList("fleet-data-row");
            wearRow.Add(new Label("Wear:") { pickingMode = PickingMode.Ignore });
            var wearBar = new ProgressBar();
            wearBar.name = $"fleet-wear-{machineId}";
            wearBar.AddToClassList("fleet-wear-bar");
            wearRow.Add(wearBar);
            card.Add(wearRow);

            // Progress bar
            var progRow = new VisualElement();
            progRow.AddToClassList("fleet-data-row");
            progRow.Add(new Label("Prog:") { pickingMode = PickingMode.Ignore });
            var progBar = new ProgressBar();
            progBar.name = $"fleet-progress-{machineId}";
            progBar.AddToClassList("fleet-progress-bar");
            progRow.Add(progBar);
            card.Add(progRow);

            // Click handler
            card.RegisterCallback<ClickEvent>(evt =>
            {
                OnMachineSelected?.Invoke(machineId);
                Debug.Log($"[FleetOverview] Selected machine: {machineId}");
            });

            return card;
        }

        private void UpdateModeButtonHighlight(string machineId)
        {
            if (!cardElements.TryGetValue(machineId, out var card)) return;
            if (!cardData.TryGetValue(machineId, out var data)) return;

            for (int i = 0; i < 5; i++)
            {
                var btn = card.Q<Button>($"fleet-sparkmode-{machineId}-{i}");
                if (btn == null) continue;
                bool active = (int)data.sparklineMode == i;
                if (active)
                    btn.AddToClassList("fleet-sparkline-mode-btn-active");
                else
                    btn.RemoveFromClassList("fleet-sparkline-mode-btn-active");
            }

            card.Q<VisualElement>($"fleet-sparkline-{machineId}")?.MarkDirtyRepaint();
        }

        private void DrawSparkline(MeshGenerationContext ctx, string machineId)
        {
            if (!cardData.TryGetValue(machineId, out var data)) return;
            if (data.historyCount < 2) return;

            var sparklineEl = cardElements.TryGetValue(machineId, out var card)
                ? card.Q<VisualElement>($"fleet-sparkline-{machineId}")
                : null;
            if (sparklineEl == null) return;

            Rect rect = sparklineEl.contentRect;
            if (rect.width < 5 || rect.height < 5) return;

            var painter = ctx.painter2D;

            if (data.sparklineMode == SparklineMode.All)
            {
                // Draw all 4 metrics overlaid, each auto-scaled to its own range
                DrawSingleMetricLine(painter, data.GetOrdered(data.spindleLoadHistory), rect, SpindleLoadColor);
                DrawSingleMetricLine(painter, data.GetOrdered(data.temperatureHistory), rect, TemperatureColor);
                DrawSingleMetricLine(painter, data.GetOrdered(data.powerHistory), rect, PowerColor);
                DrawSingleMetricLine(painter, data.GetOrdered(data.wearHistory), rect, WearColor);
            }
            else
            {
                float[] samples;
                Color color;
                switch (data.sparklineMode)
                {
                    case SparklineMode.Temperature:
                        samples = data.GetOrdered(data.temperatureHistory);
                        color = TemperatureColor;
                        break;
                    case SparklineMode.Power:
                        samples = data.GetOrdered(data.powerHistory);
                        color = PowerColor;
                        break;
                    case SparklineMode.Wear:
                        samples = data.GetOrdered(data.wearHistory);
                        color = WearColor;
                        break;
                    default:
                        samples = data.GetOrdered(data.spindleLoadHistory);
                        color = SpindleLoadColor;
                        break;
                }

                DrawSingleMetricLine(painter, samples, rect, color);
            }
        }

        /// <summary>Draw a single sparkline series auto-scaled to its own min/max range.</summary>
        private void DrawSingleMetricLine(Painter2D painter, float[] samples, Rect rect, Color color)
        {
            if (samples == null || samples.Length < 2) return;

            // Find auto-scale range
            float minVal = float.MaxValue, maxVal = float.MinValue;
            for (int i = 0; i < samples.Length; i++)
            {
                if (samples[i] < minVal) minVal = samples[i];
                if (samples[i] > maxVal) maxVal = samples[i];
            }
            float range = maxVal - minVal;
            if (range < 0.001f) range = 1f; // avoid division by zero for flat lines

            painter.strokeColor = color;
            painter.lineWidth = 1.5f;
            painter.BeginPath();

            for (int i = 0; i < samples.Length; i++)
            {
                float x = (i / (float)(samples.Length - 1)) * rect.width;
                float normalized = (samples[i] - minVal) / range;
                float y = rect.height - normalized * rect.height;
                y = Mathf.Clamp(y, 0, rect.height);

                if (i == 0) painter.MoveTo(new Vector2(x, y));
                else painter.LineTo(new Vector2(x, y));
            }

            painter.Stroke();
        }

        /// <summary>Compute severity color from worst metric. Green = normal, Yellow = warning, Red = critical.</summary>
        private Color ComputeSeverityColor(MachineCardData data)
        {
            // Normalize each metric to 0-1 range against reasonable maxes
            float loadNorm = Mathf.Clamp01(data.spindleLoad / 100f);
            float tempNorm = Mathf.Clamp01(data.latestTemperature / 100f);
            float powerNorm = Mathf.Clamp01(data.latestPower / 1000f);
            float wearNorm = Mathf.Clamp01(data.latestWear / 100f);

            float worst = Mathf.Max(loadNorm, Mathf.Max(tempNorm, Mathf.Max(powerNorm, wearNorm)));

            if (data.hasActiveAnomaly || worst >= criticalThreshold)
                return StatusRed;
            if (worst >= warningThreshold)
                return StatusYellow;
            return StatusGreen;
        }

        private void OnMachineStateUpdate(RosMessageTypes.Miracle.MachineStateMsg msg)
        {
            string id = msg.machine_id;
            if (!cardData.ContainsKey(id)) return;

            var data = cardData[id];
            data.status = msg.status ?? "UNKNOWN";
            data.spindleLoad = (float)msg.spindle_load;

            // Derive temperature from spindle_speed as proxy (no spindle_temp field on ROS msg)
            // and power from spindle_load * 10
            data.latestPower = data.spindleLoad * 10f;

            // Push history sample with latest known values
            data.PushLoadSample(data.spindleLoad);

            // Update status color based on worst metric severity
            data.statusColor = ComputeSeverityColor(data);

            UpdateCardUI(id, data);
        }

        private void OnCuttingStateUpdate(CuttingStateData cuttingData)
        {
            // CuttingStateData is not machine-keyed; apply to all machines or first active
            // For fleet view, apply to all cards (the simulation drives all machines)
            foreach (var kvp in cardData)
            {
                var data = kvp.Value;
                data.latestTemperature = cuttingData.toolTemperature;
                data.latestPower = cuttingData.powerWatts;
                data.latestWear = cuttingData.wearPercentage;

                // Re-derive severity
                data.statusColor = ComputeSeverityColor(data);
                UpdateCardUI(kvp.Key, data);
            }
        }

        private void OnAnomalyAlert(RosMessageTypes.Miracle.AnomalyAlertMsg msg)
        {
            string id = msg.machine_id;
            if (!cardData.ContainsKey(id)) return;

            var data = cardData[id];
            data.alertCount++;
            data.hasActiveAnomaly = true;
            data.statusColor = ComputeSeverityColor(data);
            UpdateCardUI(id, data);

            Debug.Log($"[FleetOverview] Anomaly alert for {id}: {msg.anomaly_type} " +
                     $"(severity={msg.severity:F2}). Total alerts: {data.alertCount}");
        }

        private void OnToolWearUpdate(RosMessageTypes.Miracle.ToolWearEstimateMsg msg)
        {
            string id = msg.machine_id;
            if (!cardData.ContainsKey(id)) return;

            var data = cardData[id];
            data.wearPercent = (float)msg.wear_percentage;
            data.latestWear = data.wearPercent;
            data.statusColor = ComputeSeverityColor(data);
            UpdateCardUI(id, data);
        }

        private void OnJobStatusUpdate(RosMessageTypes.Miracle.JobStatusMsg msg)
        {
            string id = msg.machine_id;
            if (!cardData.ContainsKey(id)) return;

            var data = cardData[id];
            data.programProgress = (float)msg.progress;
            UpdateCardUI(id, data);
        }

        /// <summary>Update alert count for a specific machine.</summary>
        public void SetAlertCount(string machineId, int count)
        {
            if (cardData.TryGetValue(machineId, out var data))
            {
                data.alertCount = count;
                data.hasActiveAnomaly = count > 0;
                data.statusColor = ComputeSeverityColor(data);
                UpdateCardUI(machineId, data);
            }
        }

        /// <summary>Update wear percentage for a specific machine.</summary>
        public void SetWearPercent(string machineId, float percent)
        {
            if (cardData.TryGetValue(machineId, out var data))
            {
                data.wearPercent = percent;
                data.latestWear = percent;
                data.statusColor = ComputeSeverityColor(data);
                UpdateCardUI(machineId, data);
            }
        }

        /// <summary>Update program progress for a specific machine.</summary>
        public void SetProgramProgress(string machineId, float progress)
        {
            if (cardData.TryGetValue(machineId, out var data))
            {
                data.programProgress = progress;
                UpdateCardUI(machineId, data);
            }
        }

        /// <summary>Clear active anomaly flag for a machine (e.g. after operator acknowledges).</summary>
        public void ClearAnomaly(string machineId)
        {
            if (cardData.TryGetValue(machineId, out var data))
            {
                data.hasActiveAnomaly = false;
                data.statusColor = ComputeSeverityColor(data);
                UpdateCardUI(machineId, data);
            }
        }

        private void UpdateCardUI(string machineId, MachineCardData data)
        {
            if (!cardElements.TryGetValue(machineId, out var card)) return;

            // Status dot color
            var dot = card.Q<VisualElement>($"fleet-status-{machineId}");
            if (dot != null)
                dot.style.backgroundColor = data.statusColor;

            // Status label
            var statusLabel = card.Q<Label>($"fleet-machine-status-{machineId}");
            if (statusLabel != null)
                statusLabel.text = data.status;

            // Alert badge
            var badge = card.Q<Label>($"fleet-alerts-{machineId}");
            if (badge != null)
            {
                badge.text = data.alertCount.ToString();
                badge.style.display = data.alertCount > 0 ? DisplayStyle.Flex : DisplayStyle.None;
            }

            // Wear bar
            var wearBar = card.Q<ProgressBar>($"fleet-wear-{machineId}");
            if (wearBar != null)
                wearBar.value = data.wearPercent;

            // Progress bar
            var progBar = card.Q<ProgressBar>($"fleet-progress-{machineId}");
            if (progBar != null)
                progBar.value = data.programProgress * 100f;

            // Trigger sparkline repaint
            var sparkline = card.Q<VisualElement>($"fleet-sparkline-{machineId}");
            sparkline?.MarkDirtyRepaint();
        }

        void Update()
        {
            if (!isVisible) return;
            // Repaint sparklines
            foreach (var id in machineIds)
            {
                if (cardElements.TryGetValue(id, out var card))
                {
                    card.Q<VisualElement>($"fleet-sparkline-{id}")?.MarkDirtyRepaint();
                }
            }
        }
    }
}
