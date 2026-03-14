using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Predictive health data for a single machine, populated from
    /// the anomaly-prediction and tool-wear-estimation pipelines.
    /// </summary>
    [System.Serializable]
    public class PredictiveHealthData
    {
        public float toolRUL_minutes;        // Remaining useful life
        public float nextAnomalyETA_seconds; // Time until predicted anomaly
        public string nextAnomalyType;       // "FORCE_WARNING", "CHATTER_RISK", etc.
        public int nextAnomalyBlockIndex;    // Which G-code block
        public float overallHealthScore;     // 0.0 (critical) to 1.0 (perfect)
        public float oeeScore;              // Current OEE 0-100
        public string trendDirection;       // "improving", "stable", "degrading"
    }

    // ─── Machine Utilization Types ──────────────────────────────────

    /// <summary>Discrete machine operating states for utilization tracking.</summary>
    public enum MachineState
    {
        RUNNING,
        IDLE,
        SETUP,
        MAINTENANCE,
        ALARM,
        OFFLINE
    }

    /// <summary>
    /// A single timestamped snapshot of machine state, used to build
    /// utilization heatmaps and state-distribution analyses.
    /// </summary>
    [System.Serializable]
    public class MachineUtilizationRecord
    {
        public double timestamp;       // Time.realtimeSinceStartupAsDouble
        public string machineId;
        public MachineState state;
        public string programName;
        public float spindleLoad;      // 0-100 %
        public float feedOverride;     // 0-200 %
    }

    /// <summary>
    /// Pre-computed heatmap data for one machine: an array of per-slot
    /// utilization percentages plus summary statistics.
    /// </summary>
    [System.Serializable]
    public class UtilizationHeatmapData
    {
        public string machineId;
        public List<float> timeSlots;          // utilization % per slot
        public float slotDurationMinutes;
        public float totalHours;
        public float averageUtilization;
        public float peakUtilization;
        public float idleTimeMinutes;
    }

    /// <summary>
    /// Idle-analysis result tuple returned by
    /// <see cref="UtilizationTracker.GetIdleAnalysis"/>.
    /// </summary>
    public struct IdleAnalysis
    {
        public float totalIdleMin;
        public float longestIdleMin;
        public int idleCount;
        public float avgIdleMin;
    }

    /// <summary>
    /// Records machine state transitions over time and computes
    /// utilization heatmaps, state distributions, and idle analyses.
    /// </summary>
    public class UtilizationTracker
    {
        /// <summary>Per-machine ring buffer of utilization records.</summary>
        public readonly Dictionary<string, List<MachineUtilizationRecord>> recordBuffer = new();

        /// <summary>
        /// Append a timestamped state record for the given machine.
        /// </summary>
        public void RecordState(string machineId, MachineState state, float spindleLoad, string program)
        {
            if (!recordBuffer.TryGetValue(machineId, out var list))
            {
                list = new List<MachineUtilizationRecord>();
                recordBuffer[machineId] = list;
            }

            list.Add(new MachineUtilizationRecord
            {
                timestamp = Time.realtimeSinceStartupAsDouble,
                machineId = machineId,
                state = state,
                programName = program ?? "",
                spindleLoad = Mathf.Clamp(spindleLoad, 0f, 100f),
                feedOverride = 100f
            });
        }

        /// <summary>
        /// Compute a heatmap for a single machine by dividing the last
        /// <paramref name="hoursBack"/> hours into slots of
        /// <paramref name="slotMinutes"/> minutes and calculating the
        /// percentage of each slot spent in <see cref="MachineState.RUNNING"/>.
        /// </summary>
        public UtilizationHeatmapData ComputeHeatmap(string machineId, float hoursBack, float slotMinutes)
        {
            var heatmap = new UtilizationHeatmapData
            {
                machineId = machineId,
                slotDurationMinutes = slotMinutes,
                totalHours = hoursBack,
                timeSlots = new List<float>()
            };

            double now = Time.realtimeSinceStartupAsDouble;
            double windowStart = now - hoursBack * 3600.0;
            double slotSeconds = slotMinutes * 60.0;
            int slotCount = Mathf.Max(1, Mathf.CeilToInt((float)(hoursBack * 60f / slotMinutes)));

            // Gather records in the time window
            List<MachineUtilizationRecord> records;
            if (!recordBuffer.TryGetValue(machineId, out records) || records.Count == 0)
            {
                for (int i = 0; i < slotCount; i++)
                    heatmap.timeSlots.Add(0f);
                heatmap.averageUtilization = 0f;
                heatmap.peakUtilization = 0f;
                heatmap.idleTimeMinutes = hoursBack * 60f;
                return heatmap;
            }

            // Filter to window and sort by timestamp
            var windowRecords = records
                .Where(r => r.timestamp >= windowStart)
                .OrderBy(r => r.timestamp)
                .ToList();

            float totalRunning = 0f;
            float totalIdle = 0f;
            float peak = 0f;

            for (int s = 0; s < slotCount; s++)
            {
                double slotStart = windowStart + s * slotSeconds;
                double slotEnd = slotStart + slotSeconds;

                // Find records that overlap this slot
                float runningSeconds = 0f;
                float slotDuration = (float)slotSeconds;

                // Get the state at slotStart (last record before slotStart)
                MachineState activeState = MachineState.IDLE;
                for (int r = windowRecords.Count - 1; r >= 0; r--)
                {
                    if (windowRecords[r].timestamp <= slotStart)
                    {
                        activeState = windowRecords[r].state;
                        break;
                    }
                }

                // Walk through transitions within the slot
                double cursor = slotStart;
                foreach (var rec in windowRecords)
                {
                    if (rec.timestamp >= slotEnd) break;
                    if (rec.timestamp <= slotStart) continue;

                    // Time from cursor to this transition
                    double segDuration = rec.timestamp - cursor;
                    if (activeState == MachineState.RUNNING)
                        runningSeconds += (float)segDuration;

                    activeState = rec.state;
                    cursor = rec.timestamp;
                }

                // Remaining time from last transition to slot end
                double remaining = slotEnd - cursor;
                if (remaining > 0 && activeState == MachineState.RUNNING)
                    runningSeconds += (float)remaining;

                float utilPct = (slotDuration > 0f) ? (runningSeconds / slotDuration) * 100f : 0f;
                utilPct = Mathf.Clamp(utilPct, 0f, 100f);
                heatmap.timeSlots.Add(utilPct);

                totalRunning += runningSeconds;
                float idleSec = slotDuration - runningSeconds;
                totalIdle += Mathf.Max(0f, idleSec);
                if (utilPct > peak) peak = utilPct;
            }

            float totalSeconds = slotCount * (float)slotSeconds;
            heatmap.averageUtilization = (totalSeconds > 0f) ? (totalRunning / totalSeconds) * 100f : 0f;
            heatmap.peakUtilization = peak;
            heatmap.idleTimeMinutes = totalIdle / 60f;

            return heatmap;
        }

        /// <summary>
        /// Compute heatmaps for every tracked machine.
        /// </summary>
        public List<UtilizationHeatmapData> ComputeFleetHeatmap(float hoursBack, float slotMinutes)
        {
            var result = new List<UtilizationHeatmapData>();
            foreach (var machineId in recordBuffer.Keys)
                result.Add(ComputeHeatmap(machineId, hoursBack, slotMinutes));
            return result;
        }

        /// <summary>
        /// Return the percentage of time spent in each <see cref="MachineState"/>
        /// over the last <paramref name="hoursBack"/> hours.
        /// </summary>
        public Dictionary<MachineState, float> GetStateDistribution(string machineId, float hoursBack)
        {
            var distribution = new Dictionary<MachineState, float>();
            foreach (MachineState s in Enum.GetValues(typeof(MachineState)))
                distribution[s] = 0f;

            double now = Time.realtimeSinceStartupAsDouble;
            double windowStart = now - hoursBack * 3600.0;
            double windowEnd = now;

            if (!recordBuffer.TryGetValue(machineId, out var records) || records.Count == 0)
            {
                distribution[MachineState.IDLE] = 100f;
                return distribution;
            }

            var windowRecords = records
                .Where(r => r.timestamp >= windowStart)
                .OrderBy(r => r.timestamp)
                .ToList();

            // Determine state at window start
            MachineState activeState = MachineState.IDLE;
            for (int r = records.Count - 1; r >= 0; r--)
            {
                if (records[r].timestamp <= windowStart)
                {
                    activeState = records[r].state;
                    break;
                }
            }

            var durations = new Dictionary<MachineState, double>();
            foreach (MachineState s in Enum.GetValues(typeof(MachineState)))
                durations[s] = 0.0;

            double cursor = windowStart;
            foreach (var rec in windowRecords)
            {
                if (rec.timestamp > windowEnd) break;
                double segDuration = rec.timestamp - cursor;
                if (segDuration > 0)
                    durations[activeState] += segDuration;
                activeState = rec.state;
                cursor = rec.timestamp;
            }

            // Trailing segment
            double trailing = windowEnd - cursor;
            if (trailing > 0)
                durations[activeState] += trailing;

            double totalDuration = windowEnd - windowStart;
            if (totalDuration > 0)
            {
                foreach (MachineState s in Enum.GetValues(typeof(MachineState)))
                    distribution[s] = (float)(durations[s] / totalDuration * 100.0);
            }

            return distribution;
        }

        /// <summary>
        /// Analyse idle periods for a machine over the last
        /// <paramref name="hoursBack"/> hours.
        /// </summary>
        public IdleAnalysis GetIdleAnalysis(string machineId, float hoursBack)
        {
            var result = new IdleAnalysis();

            double now = Time.realtimeSinceStartupAsDouble;
            double windowStart = now - hoursBack * 3600.0;
            double windowEnd = now;

            if (!recordBuffer.TryGetValue(machineId, out var records) || records.Count == 0)
            {
                result.totalIdleMin = hoursBack * 60f;
                result.longestIdleMin = hoursBack * 60f;
                result.idleCount = 1;
                result.avgIdleMin = hoursBack * 60f;
                return result;
            }

            var windowRecords = records
                .Where(r => r.timestamp >= windowStart)
                .OrderBy(r => r.timestamp)
                .ToList();

            // State at window start
            MachineState activeState = MachineState.IDLE;
            for (int r = records.Count - 1; r >= 0; r--)
            {
                if (records[r].timestamp <= windowStart)
                {
                    activeState = records[r].state;
                    break;
                }
            }

            float totalIdleSec = 0f;
            float longestIdleSec = 0f;
            float currentIdleSec = 0f;
            int idleCount = 0;
            bool wasIdle = (activeState == MachineState.IDLE);
            if (wasIdle) idleCount = 1;

            double cursor = windowStart;
            foreach (var rec in windowRecords)
            {
                if (rec.timestamp > windowEnd) break;
                float seg = (float)(rec.timestamp - cursor);

                if (activeState == MachineState.IDLE)
                {
                    currentIdleSec += seg;
                    totalIdleSec += seg;
                }
                else
                {
                    if (currentIdleSec > longestIdleSec)
                        longestIdleSec = currentIdleSec;
                    currentIdleSec = 0f;
                }

                if (rec.state == MachineState.IDLE && activeState != MachineState.IDLE)
                    idleCount++;

                activeState = rec.state;
                cursor = rec.timestamp;
            }

            // Trailing segment
            float trailingSeg = (float)(windowEnd - cursor);
            if (activeState == MachineState.IDLE)
            {
                currentIdleSec += trailingSeg;
                totalIdleSec += trailingSeg;
            }
            if (currentIdleSec > longestIdleSec)
                longestIdleSec = currentIdleSec;

            result.totalIdleMin = totalIdleSec / 60f;
            result.longestIdleMin = longestIdleSec / 60f;
            result.idleCount = idleCount;
            result.avgIdleMin = (idleCount > 0) ? (totalIdleSec / 60f) / idleCount : 0f;
            return result;
        }
    }

    /// <summary>
    /// Fleet overview panel showing a grid of machine cards.
    /// Each card displays: status indicator, multi-metric sparklines (load, temp, power, wear),
    /// wear bar, program progress, alert badge count, and predictive health indicators
    /// (RUL countdown, health ring, anomaly ETA badge, OEE trend arrow).
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
        private readonly Dictionary<string, PredictiveHealthData> _machineHealth = new();

        /// <summary>Tracks machine state transitions for utilization heatmap and timeline.</summary>
        private readonly UtilizationTracker _utilizationTracker = new();

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

            // Fleet health summary row
            var healthRow = new VisualElement();
            healthRow.AddToClassList("fleet-health-summary-row");

            var healthScore = new Label("Fleet Health: --%");
            healthScore.name = "fleet-health-score";
            healthScore.AddToClassList("fleet-metric-label");
            healthRow.Add(healthScore);

            var rulCritical = new Label("RUL Critical: 0");
            rulCritical.name = "fleet-rul-critical-count";
            rulCritical.AddToClassList("fleet-metric-label");
            healthRow.Add(rulCritical);

            var anomalyCount = new Label("Anomalies <5m: 0");
            anomalyCount.name = "fleet-anomaly-count";
            anomalyCount.AddToClassList("fleet-metric-label");
            healthRow.Add(anomalyCount);

            grid.Add(healthRow);
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

            // --- Predictive Health Indicators ---

            // Health ring (circular progress indicator)
            var healthRing = CreateHealthRing(1.0f);
            healthRing.name = $"fleet-health-ring-{machineId}";
            card.Add(healthRing);

            // RUL countdown bar
            var rulBar = CreateRULBar(float.PositiveInfinity);
            rulBar.name = $"fleet-rul-bar-{machineId}";
            card.Add(rulBar);

            // Anomaly countdown badge
            var anomalyBadge = CreateAnomalyCountdown("", float.PositiveInfinity);
            anomalyBadge.name = $"fleet-anomaly-badge-{machineId}";
            anomalyBadge.style.display = DisplayStyle.None;
            card.Add(anomalyBadge);

            // OEE trend arrow
            var trendLabel = new Label("");
            trendLabel.name = $"fleet-trend-{machineId}";
            trendLabel.AddToClassList("fleet-trend-arrow");
            card.Add(trendLabel);

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

            // Update predictive health indicators if data exists
            if (_machineHealth.TryGetValue(machineId, out var healthData))
                ApplyPredictiveHealthUI(card, machineId, healthData);
        }

        // ─── Predictive Health Indicator Methods ───────────────────────

        /// <summary>Update predictive health data for a machine and refresh UI.</summary>
        public void UpdatePredictiveHealth(string machineId, PredictiveHealthData data)
        {
            _machineHealth[machineId] = data;

            if (cardElements.TryGetValue(machineId, out var card))
                ApplyPredictiveHealthUI(card, machineId, data);

            UpdateFleetHealthSummary();
        }

        /// <summary>Apply predictive health data to a card's UI elements.</summary>
        private void ApplyPredictiveHealthUI(VisualElement card, string machineId, PredictiveHealthData data)
        {
            // Health ring
            var healthRing = card.Q<VisualElement>($"fleet-health-ring-{machineId}");
            if (healthRing != null)
            {
                var fill = healthRing.Q<VisualElement>("health-ring-fill");
                var label = healthRing.Q<Label>("health-ring-label");
                Color hc = GetHealthColor(data.overallHealthScore);

                if (fill != null)
                {
                    float pct = Mathf.Clamp01(data.overallHealthScore) * 100f;
                    fill.style.width = new Length(pct, LengthUnit.Percent);
                    fill.style.backgroundColor = hc;
                }
                if (label != null)
                    label.text = $"{Mathf.RoundToInt(data.overallHealthScore * 100)}%";
            }

            // RUL bar
            var rulBar = card.Q<VisualElement>($"fleet-rul-bar-{machineId}");
            if (rulBar != null)
            {
                var fill = rulBar.Q<VisualElement>("rul-bar-fill");
                var label = rulBar.Q<Label>("rul-bar-label");

                Color rulColor;
                if (data.toolRUL_minutes > 60f)
                    rulColor = StatusGreen;
                else if (data.toolRUL_minutes > 30f)
                    rulColor = StatusYellow;
                else
                    rulColor = StatusRed;

                if (fill != null)
                {
                    float pct = Mathf.Clamp01(data.toolRUL_minutes / 120f) * 100f;
                    fill.style.width = new Length(pct, LengthUnit.Percent);
                    fill.style.backgroundColor = rulColor;
                }
                if (label != null)
                    label.text = float.IsInfinity(data.toolRUL_minutes)
                        ? "RUL: --"
                        : $"RUL: {data.toolRUL_minutes:F0}m";
            }

            // Anomaly countdown badge
            var anomalyBadge = card.Q<VisualElement>($"fleet-anomaly-badge-{machineId}");
            if (anomalyBadge != null)
            {
                bool hasAnomaly = !float.IsInfinity(data.nextAnomalyETA_seconds)
                               && !string.IsNullOrEmpty(data.nextAnomalyType)
                               && data.nextAnomalyETA_seconds >= 0;
                anomalyBadge.style.display = hasAnomaly ? DisplayStyle.Flex : DisplayStyle.None;

                if (hasAnomaly)
                {
                    var label = anomalyBadge.Q<Label>("anomaly-badge-label");
                    if (label != null)
                    {
                        int totalSec = Mathf.RoundToInt(data.nextAnomalyETA_seconds);
                        int min = totalSec / 60;
                        int sec = totalSec % 60;
                        label.text = $"{data.nextAnomalyType} in {min}:{sec:D2}";
                    }

                    // Pulse class for critical (< 60s)
                    if (data.nextAnomalyETA_seconds < 60f)
                        anomalyBadge.AddToClassList("anomaly-badge-critical");
                    else
                        anomalyBadge.RemoveFromClassList("anomaly-badge-critical");
                }
            }

            // OEE trend arrow
            var trendLabel = card.Q<Label>($"fleet-trend-{machineId}");
            if (trendLabel != null)
            {
                switch (data.trendDirection)
                {
                    case "improving":
                        trendLabel.text = $"OEE {data.oeeScore:F0}% \u2191";
                        trendLabel.RemoveFromClassList("trend-stable");
                        trendLabel.RemoveFromClassList("trend-degrading");
                        trendLabel.AddToClassList("trend-improving");
                        break;
                    case "degrading":
                        trendLabel.text = $"OEE {data.oeeScore:F0}% \u2193";
                        trendLabel.RemoveFromClassList("trend-stable");
                        trendLabel.RemoveFromClassList("trend-improving");
                        trendLabel.AddToClassList("trend-degrading");
                        break;
                    default: // "stable"
                        trendLabel.text = $"OEE {data.oeeScore:F0}% \u2192";
                        trendLabel.RemoveFromClassList("trend-improving");
                        trendLabel.RemoveFromClassList("trend-degrading");
                        trendLabel.AddToClassList("trend-stable");
                        break;
                }
            }
        }

        /// <summary>Create a health ring indicator element.</summary>
        private VisualElement CreateHealthRing(float score)
        {
            var container = new VisualElement();
            container.AddToClassList("fleet-health-ring");

            var track = new VisualElement();
            track.AddToClassList("health-ring-track");

            var fill = new VisualElement();
            fill.name = "health-ring-fill";
            fill.AddToClassList("health-ring-fill");
            float pct = Mathf.Clamp01(score) * 100f;
            fill.style.width = new Length(pct, LengthUnit.Percent);
            fill.style.backgroundColor = GetHealthColor(score);
            track.Add(fill);

            container.Add(track);

            var label = new Label($"{Mathf.RoundToInt(score * 100)}%");
            label.name = "health-ring-label";
            label.AddToClassList("health-ring-label");
            container.Add(label);

            return container;
        }

        /// <summary>Create a RUL countdown bar element.</summary>
        private VisualElement CreateRULBar(float rul_minutes)
        {
            var container = new VisualElement();
            container.AddToClassList("fleet-rul-container");

            var track = new VisualElement();
            track.AddToClassList("rul-bar-track");

            var fill = new VisualElement();
            fill.name = "rul-bar-fill";
            fill.AddToClassList("rul-bar-fill");

            Color rulColor;
            if (float.IsInfinity(rul_minutes) || rul_minutes > 60f)
                rulColor = StatusGreen;
            else if (rul_minutes > 30f)
                rulColor = StatusYellow;
            else
                rulColor = StatusRed;

            float pct = float.IsInfinity(rul_minutes) ? 100f : Mathf.Clamp01(rul_minutes / 120f) * 100f;
            fill.style.width = new Length(pct, LengthUnit.Percent);
            fill.style.backgroundColor = rulColor;
            track.Add(fill);

            container.Add(track);

            var label = new Label(float.IsInfinity(rul_minutes) ? "RUL: --" : $"RUL: {rul_minutes:F0}m");
            label.name = "rul-bar-label";
            label.AddToClassList("rul-bar-label");
            container.Add(label);

            return container;
        }

        /// <summary>Create an anomaly countdown badge element.</summary>
        private VisualElement CreateAnomalyCountdown(string type, float eta_seconds)
        {
            var badge = new VisualElement();
            badge.AddToClassList("fleet-anomaly-badge");

            var label = new Label("");
            label.name = "anomaly-badge-label";
            label.AddToClassList("anomaly-badge-label");

            if (!string.IsNullOrEmpty(type) && !float.IsInfinity(eta_seconds))
            {
                int totalSec = Mathf.RoundToInt(eta_seconds);
                int min = totalSec / 60;
                int sec = totalSec % 60;
                label.text = $"{type} in {min}:{sec:D2}";
            }

            badge.Add(label);
            return badge;
        }

        /// <summary>Map a 0-1 health score to a color (red-yellow-green gradient).</summary>
        private Color GetHealthColor(float score)
        {
            score = Mathf.Clamp01(score);
            if (score >= 0.7f)
                return Color.Lerp(StatusYellow, StatusGreen, (score - 0.7f) / 0.3f);
            if (score >= 0.3f)
                return Color.Lerp(StatusRed, StatusYellow, (score - 0.3f) / 0.4f);
            return StatusRed;
        }

        /// <summary>Update the fleet-level health summary labels.</summary>
        private void UpdateFleetHealthSummary()
        {
            if (fleetPanel == null) return;

            float totalHealth = 0f;
            int healthCount = 0;
            int rulCriticalCount = 0;
            int upcomingAnomalies = 0;

            foreach (var kvp in _machineHealth)
            {
                var h = kvp.Value;
                totalHealth += h.overallHealthScore;
                healthCount++;

                if (h.toolRUL_minutes < 30f)
                    rulCriticalCount++;

                if (!float.IsInfinity(h.nextAnomalyETA_seconds) && h.nextAnomalyETA_seconds <= 300f)
                    upcomingAnomalies++;
            }

            float avgHealth = healthCount > 0 ? totalHealth / healthCount : 1f;

            var healthLabel = fleetPanel.Q<Label>("fleet-health-score");
            if (healthLabel != null)
                healthLabel.text = $"Fleet Health: {Mathf.RoundToInt(avgHealth * 100)}%";

            var rulLabel = fleetPanel.Q<Label>("fleet-rul-critical-count");
            if (rulLabel != null)
            {
                rulLabel.text = $"RUL Critical: {rulCriticalCount}";
                rulLabel.style.color = rulCriticalCount > 0 ? StatusRed : StatusGreen;
            }

            var anomalyLabel = fleetPanel.Q<Label>("fleet-anomaly-count");
            if (anomalyLabel != null)
            {
                anomalyLabel.text = $"Anomalies <5m: {upcomingAnomalies}";
                anomalyLabel.style.color = upcomingAnomalies > 0 ? StatusYellow : StatusGreen;
            }
        }

        // ─── Utilization Heatmap & Timeline Rendering ─────────────────

        /// <summary>Color mapping for each MachineState used in timeline bars.</summary>
        private static readonly Dictionary<MachineState, Color> StateColors = new()
        {
            { MachineState.RUNNING,     new Color(0.2f, 0.8f, 0.3f) },     // green
            { MachineState.IDLE,        new Color(0.5f, 0.5f, 0.6f) },     // gray
            { MachineState.SETUP,       new Color(0.3f, 0.6f, 1.0f) },     // blue
            { MachineState.MAINTENANCE, new Color(0.9f, 0.7f, 0.1f) },     // yellow
            { MachineState.ALARM,       new Color(0.9f, 0.2f, 0.2f) },     // red
            { MachineState.OFFLINE,     new Color(0.3f, 0.3f, 0.35f) }     // dark gray
        };

        /// <summary>Expose the utilization tracker so external systems can record states.</summary>
        public UtilizationTracker UtilizationTracker => _utilizationTracker;

        /// <summary>
        /// Map a utilization percentage (0-100) to a color on a red-yellow-green
        /// gradient.  Red = low/idle, yellow = medium, green = high utilization.
        /// </summary>
        public static Color GetUtilizationColor(float pct)
        {
            pct = Mathf.Clamp(pct, 0f, 100f);
            // 0-40 → red to yellow, 40-75 → yellow to green, 75-100 → green
            if (pct <= 40f)
                return Color.Lerp(
                    new Color(0.9f, 0.2f, 0.2f),   // red
                    new Color(0.9f, 0.7f, 0.1f),   // yellow
                    pct / 40f);
            if (pct <= 75f)
                return Color.Lerp(
                    new Color(0.9f, 0.7f, 0.1f),   // yellow
                    new Color(0.2f, 0.8f, 0.3f),   // green
                    (pct - 40f) / 35f);
            return new Color(0.2f, 0.8f, 0.3f);    // green
        }

        /// <summary>
        /// Build a color-coded heatmap grid from fleet utilization data and
        /// insert it into the fleet panel.  Each row is a machine, each cell
        /// is a time slot colored by utilization percentage.
        /// </summary>
        public void DrawUtilizationHeatmap(List<UtilizationHeatmapData> data)
        {
            if (fleetPanel == null || data == null) return;

            // Remove any previous heatmap
            var existing = fleetPanel.Q<VisualElement>("utilization-heatmap-container");
            existing?.RemoveFromHierarchy();

            var container = new VisualElement();
            container.name = "utilization-heatmap-container";
            container.AddToClassList("heatmap-container");

            // Title
            var title = new Label("Utilization Heatmap");
            title.AddToClassList("panel-title");
            container.Add(title);

            foreach (var machineData in data)
            {
                var row = new VisualElement();
                row.AddToClassList("heatmap-row");

                // Machine label
                var idLabel = new Label(machineData.machineId.ToUpper());
                idLabel.AddToClassList("heatmap-machine-label");
                row.Add(idLabel);

                // Cells container
                var cellsContainer = new VisualElement();
                cellsContainer.AddToClassList("heatmap-cells");

                foreach (float slotPct in machineData.timeSlots)
                {
                    var cell = new VisualElement();
                    cell.AddToClassList("heatmap-cell");
                    cell.style.backgroundColor = GetUtilizationColor(slotPct);
                    cell.tooltip = $"{slotPct:F1}%";
                    cellsContainer.Add(cell);
                }

                row.Add(cellsContainer);

                // Summary label
                var summary = new Label($"Avg: {machineData.averageUtilization:F1}%  Peak: {machineData.peakUtilization:F1}%");
                summary.AddToClassList("heatmap-summary-label");
                row.Add(summary);

                container.Add(row);
            }

            // Legend
            var legend = new VisualElement();
            legend.AddToClassList("utilization-legend");

            var legendTitle = new Label("Legend:");
            legendTitle.AddToClassList("heatmap-legend-title");
            legend.Add(legendTitle);

            float[] legendPcts = { 0f, 25f, 50f, 75f, 100f };
            foreach (float p in legendPcts)
            {
                var item = new VisualElement();
                item.AddToClassList("heatmap-legend-item");

                var swatch = new VisualElement();
                swatch.AddToClassList("heatmap-cell");
                swatch.style.backgroundColor = GetUtilizationColor(p);
                item.Add(swatch);

                var lbl = new Label($"{p:F0}%");
                lbl.AddToClassList("heatmap-legend-label");
                item.Add(lbl);

                legend.Add(item);
            }

            container.Add(legend);
            fleetPanel.Add(container);
        }

        /// <summary>
        /// Draw a horizontal state-timeline bar for a single machine
        /// showing colored segments for each state transition over time.
        /// </summary>
        public void DrawStateTimeline(string machineId)
        {
            if (fleetPanel == null) return;

            // Remove previous timeline for this machine
            string timelineName = $"state-timeline-{machineId}";
            var existing = fleetPanel.Q<VisualElement>(timelineName);
            existing?.RemoveFromHierarchy();

            if (!_utilizationTracker.recordBuffer.TryGetValue(machineId, out var records)
                || records.Count == 0)
                return;

            var sorted = records.OrderBy(r => r.timestamp).ToList();

            double earliest = sorted[0].timestamp;
            double latest = Time.realtimeSinceStartupAsDouble;
            double totalSpan = latest - earliest;
            if (totalSpan <= 0) return;

            var container = new VisualElement();
            container.name = timelineName;
            container.AddToClassList("state-timeline");

            // Machine label
            var idLabel = new Label(machineId.ToUpper());
            idLabel.AddToClassList("state-timeline-label");
            container.Add(idLabel);

            // Timeline bar
            var bar = new VisualElement();
            bar.AddToClassList("state-timeline-bar");

            for (int i = 0; i < sorted.Count; i++)
            {
                double segStart = sorted[i].timestamp;
                double segEnd = (i + 1 < sorted.Count) ? sorted[i + 1].timestamp : latest;
                float pct = (float)((segEnd - segStart) / totalSpan) * 100f;
                if (pct < 0.1f) continue;

                var segment = new VisualElement();
                segment.AddToClassList("state-timeline-segment");
                segment.style.width = new Length(pct, LengthUnit.Percent);

                Color segColor;
                if (!StateColors.TryGetValue(sorted[i].state, out segColor))
                    segColor = new Color(0.5f, 0.5f, 0.6f);
                segment.style.backgroundColor = segColor;
                segment.tooltip = $"{sorted[i].state}";

                bar.Add(segment);
            }

            container.Add(bar);

            // State legend
            var legend = new VisualElement();
            legend.AddToClassList("utilization-legend");
            foreach (var kvp in StateColors)
            {
                var item = new VisualElement();
                item.AddToClassList("heatmap-legend-item");

                var swatch = new VisualElement();
                swatch.AddToClassList("heatmap-cell");
                swatch.style.backgroundColor = kvp.Value;
                item.Add(swatch);

                var lbl = new Label(kvp.Key.ToString());
                lbl.AddToClassList("heatmap-legend-label");
                item.Add(lbl);

                legend.Add(item);
            }
            container.Add(legend);

            fleetPanel.Add(container);
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
