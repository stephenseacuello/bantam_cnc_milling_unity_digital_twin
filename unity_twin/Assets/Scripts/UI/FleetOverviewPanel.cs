using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Fleet overview panel showing a grid of machine cards.
    /// Each card displays: status indicator, sparkline (60s spindle load),
    /// wear bar, program progress, and alert badge count.
    /// Click a card to switch the dashboard to that machine's detailed view.
    /// </summary>
    public class FleetOverviewPanel : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private MachineStateEventSO machineStateEvent;

        [Header("Fleet Settings")]
        [SerializeField] private string[] machineIds = { "cnc1", "cnc2", "cnc3" };
        [SerializeField] private int sparklineSamples = 60;

        /// <summary>Fired when user clicks a machine card. Parameter is machine_id.</summary>
        public event Action<string> OnMachineSelected;

        private VisualElement fleetPanel;
        private bool isVisible;
        private readonly Dictionary<string, MachineCardData> cardData = new();
        private readonly Dictionary<string, VisualElement> cardElements = new();

        private class MachineCardData
        {
            public string machineId;
            public string status = "IDLE";
            public float spindleLoad;
            public float wearPercent;
            public float programProgress;
            public int alertCount;
            public readonly Queue<float> sparklineData = new();
            public Color statusColor = new(0.5f, 0.5f, 0.6f);
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
                cardData[id] = new MachineCardData { machineId = id };
            }

            BuildCards();

            if (machineStateEvent != null)
                machineStateEvent.Register(OnMachineStateUpdate);
        }

        void OnDestroy()
        {
            if (machineStateEvent != null)
                machineStateEvent.Unregister(OnMachineStateUpdate);
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

        private void DrawSparkline(MeshGenerationContext ctx, string machineId)
        {
            if (!cardData.TryGetValue(machineId, out var data)) return;
            if (data.sparklineData.Count < 2) return;

            var painter = ctx.painter2D;
            var samples = data.sparklineData.ToArray();

            var sparklineEl = cardElements.TryGetValue(machineId, out var card)
                ? card.Q<VisualElement>($"fleet-sparkline-{machineId}")
                : null;
            if (sparklineEl == null) return;

            Rect rect = sparklineEl.contentRect;
            if (rect.width < 5 || rect.height < 5) return;

            painter.strokeColor = new Color(0.3f, 0.7f, 1f, 0.8f);
            painter.lineWidth = 1.5f;
            painter.BeginPath();

            float maxVal = 100f;
            for (int i = 0; i < samples.Length; i++)
            {
                float x = (i / (float)(sparklineSamples - 1)) * rect.width;
                float y = rect.height - (samples[i] / maxVal) * rect.height;
                y = Mathf.Clamp(y, 0, rect.height);

                if (i == 0) painter.MoveTo(new Vector2(x, y));
                else painter.LineTo(new Vector2(x, y));
            }

            painter.Stroke();
        }

        private void OnMachineStateUpdate(RosMessageTypes.Miracle.MachineStateMsg msg)
        {
            string id = msg.machine_id;
            if (!cardData.ContainsKey(id)) return;

            var data = cardData[id];
            data.status = msg.machine_status ?? "UNKNOWN";
            data.spindleLoad = (float)msg.spindle_load;

            // Add to sparkline
            data.sparklineData.Enqueue(data.spindleLoad);
            while (data.sparklineData.Count > sparklineSamples)
                data.sparklineData.Dequeue();

            // Update status color
            data.statusColor = data.status switch
            {
                "RUNNING" => new Color(0.2f, 0.8f, 0.3f),
                "IDLE" => new Color(0.5f, 0.5f, 0.6f),
                "ERROR" or "FAULT" => new Color(0.9f, 0.2f, 0.2f),
                "MAINTENANCE" => new Color(0.9f, 0.7f, 0.1f),
                _ => new Color(0.5f, 0.5f, 0.6f),
            };

            UpdateCardUI(id, data);
        }

        /// <summary>Update alert count for a specific machine.</summary>
        public void SetAlertCount(string machineId, int count)
        {
            if (cardData.TryGetValue(machineId, out var data))
            {
                data.alertCount = count;
                UpdateCardUI(machineId, data);
            }
        }

        /// <summary>Update wear percentage for a specific machine.</summary>
        public void SetWearPercent(string machineId, float percent)
        {
            if (cardData.TryGetValue(machineId, out var data))
            {
                data.wearPercent = percent;
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
