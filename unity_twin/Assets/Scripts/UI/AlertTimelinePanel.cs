using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Scrolling timeline of alerts with severity color-coding.
    /// Subscribes to AnomalyAlertEventSO, CorrelatedAlertEventSO, and SecurityAlertEventSO
    /// and displays a filterable, color-coded list of recent alerts (newest first).
    /// </summary>
    public class AlertTimelinePanel : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;

        [Header("Event Channels")]
        [SerializeField] private AnomalyAlertEventSO onAnomalyAlert;
        [SerializeField] private CorrelatedAlertEventSO onCorrelatedAlert;
        [SerializeField] private SecurityAlertEventSO onSecurityAlert;

        [Header("Settings")]
        [SerializeField] private int maxAlerts = 100;

        public enum AlertCategory
        {
            Anomaly,
            Correlated,
            Security
        }

        public struct AlertEntry
        {
            public DateTime Timestamp;
            public string MachineId;
            public string AlertType;
            public string Message;
            public float Severity;
            public AlertCategory Category;
        }

        /// <summary>Total number of alerts currently stored.</summary>
        public int AlertCount => alerts.Count;

        private readonly List<AlertEntry> alerts = new();
        private AlertCategory? activeFilter;

        // UI elements
        private VisualElement timelinePanel;
        private Label alertCountLabel;
        private ScrollView alertScrollView;
        private readonly List<Button> filterButtons = new();

        void Start()
        {
            if (uiDocument == null) return;

            var root = uiDocument.rootVisualElement;
            timelinePanel = root.Q<VisualElement>("alert-timeline-panel");

            if (timelinePanel == null)
            {
                BuildDynamicUI(root);
            }
            else
            {
                BindExistingUI();
            }
        }

        void OnEnable()
        {
            if (onAnomalyAlert != null)
                onAnomalyAlert.Register(OnAnomalyAlert);
            if (onCorrelatedAlert != null)
                onCorrelatedAlert.Register(OnCorrelatedAlert);
            if (onSecurityAlert != null)
                onSecurityAlert.Register(OnSecurityAlert);
        }

        void OnDisable()
        {
            if (onAnomalyAlert != null)
                onAnomalyAlert.Unregister(OnAnomalyAlert);
            if (onCorrelatedAlert != null)
                onCorrelatedAlert.Unregister(OnCorrelatedAlert);
            if (onSecurityAlert != null)
                onSecurityAlert.Unregister(OnSecurityAlert);
        }

        /// <summary>Clear all stored alerts and refresh the UI.</summary>
        public void ClearAlerts()
        {
            alerts.Clear();
            RefreshUI();
        }

        // ── Event handlers ──────────────────────────────────────────────

        private void OnAnomalyAlert(RosMessageTypes.Miracle.AnomalyAlertMsg msg)
        {
            AddAlert(new AlertEntry
            {
                Timestamp = DateTime.Now,
                MachineId = msg.machine_id,
                AlertType = msg.anomaly_type,
                Message = msg.recommended_action,
                Severity = (float)msg.severity,
                Category = AlertCategory.Anomaly
            });
        }

        private void OnCorrelatedAlert(RosMessageTypes.Miracle.CorrelatedAlertMsg msg)
        {
            // Use the max severity from all correlated anomalies
            float maxSeverity = msg.anomaly_severities != null && msg.anomaly_severities.Length > 0
                ? (float)msg.anomaly_severities.Max()
                : (float)msg.estimated_impact;

            AddAlert(new AlertEntry
            {
                Timestamp = DateTime.Now,
                MachineId = msg.machine_id,
                AlertType = msg.summary,
                Message = msg.root_cause_hypothesis,
                Severity = maxSeverity,
                Category = AlertCategory.Correlated
            });
        }

        private void OnSecurityAlert(RosMessageTypes.Miracle.SecurityAlertMsg msg)
        {
            // SecurityAlertMsg.severity is a string (e.g. "HIGH", "MEDIUM", "LOW")
            float severityValue = msg.severity?.ToUpperInvariant() switch
            {
                "CRITICAL" => 1.0f,
                "HIGH" => 0.9f,
                "MEDIUM" => 0.6f,
                "LOW" => 0.3f,
                _ => (float)msg.confidence
            };

            AddAlert(new AlertEntry
            {
                Timestamp = DateTime.Now,
                MachineId = msg.source_node,
                AlertType = msg.category,
                Message = msg.description,
                Severity = severityValue,
                Category = AlertCategory.Security
            });
        }

        // ── Core logic ──────────────────────────────────────────────────

        private void AddAlert(AlertEntry entry)
        {
            alerts.Insert(0, entry);

            // Trim to max
            while (alerts.Count > maxAlerts)
                alerts.RemoveAt(alerts.Count - 1);

            RefreshUI();
        }

        private void RefreshUI()
        {
            if (alertScrollView == null) return;

            alertScrollView.Clear();

            var filtered = activeFilter.HasValue
                ? alerts.Where(a => a.Category == activeFilter.Value)
                : alerts;

            foreach (var alert in filtered)
            {
                alertScrollView.Add(CreateAlertCard(alert));
            }

            if (alertCountLabel != null)
                alertCountLabel.text = $"{alerts.Count} alert{(alerts.Count != 1 ? "s" : "")}";
        }

        // ── Alert card construction ─────────────────────────────────────

        private VisualElement CreateAlertCard(AlertEntry alert)
        {
            var card = new VisualElement();
            card.AddToClassList("alert-card");

            // Severity color bar
            var severityBar = new VisualElement();
            severityBar.AddToClassList("alert-severity-bar");
            severityBar.style.backgroundColor = GetSeverityColor(alert.Severity);
            card.Add(severityBar);

            // Card content
            var content = new VisualElement();
            content.AddToClassList("alert-card-content");

            // Header row: alert type + timestamp
            var header = new VisualElement();
            header.AddToClassList("alert-card-header");

            var typeLabel = new Label(TruncateText(alert.AlertType, 24));
            typeLabel.AddToClassList("alert-type-label");
            header.Add(typeLabel);

            var timeLabel = new Label(alert.Timestamp.ToString("HH:mm:ss"));
            timeLabel.AddToClassList("alert-time-label");
            header.Add(timeLabel);

            content.Add(header);

            // Detail row: machine label + severity %
            var detail = new VisualElement();
            detail.AddToClassList("alert-card-detail");

            var machineLabel = new Label(alert.MachineId);
            machineLabel.AddToClassList("alert-machine-label");
            detail.Add(machineLabel);

            var severityLabel = new Label($"{alert.Severity * 100f:F0}%");
            severityLabel.AddToClassList("alert-severity-label");
            severityLabel.style.color = GetSeverityColor(alert.Severity);
            detail.Add(severityLabel);

            content.Add(detail);

            // Category badge
            var badge = new Label(alert.Category.ToString());
            badge.AddToClassList("alert-category-badge");
            badge.AddToClassList(alert.Category switch
            {
                AlertCategory.Anomaly => "alert-badge-anomaly",
                AlertCategory.Correlated => "alert-badge-correlated",
                AlertCategory.Security => "alert-badge-security",
                _ => "alert-badge-anomaly"
            });
            content.Add(badge);

            card.Add(content);
            return card;
        }

        private static Color GetSeverityColor(float severity)
        {
            if (severity >= 0.8f)
                return new Color(0.93f, 0.27f, 0.27f); // red
            if (severity >= 0.5f)
                return new Color(0.92f, 0.80f, 0.13f); // yellow
            return new Color(0.30f, 0.80f, 0.40f);     // green
        }

        private static string TruncateText(string text, int max)
        {
            if (string.IsNullOrEmpty(text)) return "--";
            return text.Length <= max ? text : text.Substring(0, max - 1) + "\u2026";
        }

        // ── Filter handling ─────────────────────────────────────────────

        private void SetFilter(AlertCategory? category)
        {
            activeFilter = category;

            // Update active button styling
            for (int i = 0; i < filterButtons.Count; i++)
            {
                var btn = filterButtons[i];
                bool isActive = i switch
                {
                    0 => !category.HasValue,               // All
                    1 => category == AlertCategory.Anomaly,
                    2 => category == AlertCategory.Correlated,
                    3 => category == AlertCategory.Security,
                    _ => false
                };

                if (isActive)
                    btn.AddToClassList("alert-filter-active");
                else
                    btn.RemoveFromClassList("alert-filter-active");
            }

            RefreshUI();
        }

        // ── UI binding ──────────────────────────────────────────────────

        private void BindExistingUI()
        {
            alertCountLabel = timelinePanel.Q<Label>("alert-count");
            alertScrollView = timelinePanel.Q<ScrollView>("alert-scroll-view");

            // Bind filter buttons
            var filtersContainer = timelinePanel.Q<VisualElement>(className: "alert-timeline-filters");
            if (filtersContainer != null)
            {
                var buttons = filtersContainer.Query<Button>().ToList();
                filterButtons.Clear();
                filterButtons.AddRange(buttons);

                if (buttons.Count >= 4)
                {
                    buttons[0].clicked += () => SetFilter(null);
                    buttons[1].clicked += () => SetFilter(AlertCategory.Anomaly);
                    buttons[2].clicked += () => SetFilter(AlertCategory.Correlated);
                    buttons[3].clicked += () => SetFilter(AlertCategory.Security);
                }
            }
        }

        // ── Dynamic UI creation (fallback) ──────────────────────────────

        private void BuildDynamicUI(VisualElement root)
        {
            var mainContent = root.Q<VisualElement>("main-content");
            if (mainContent == null)
            {
                Debug.LogWarning("[AlertTimelinePanel] Could not find main-content element.");
                return;
            }

            timelinePanel = new VisualElement();
            timelinePanel.name = "alert-timeline-panel";
            timelinePanel.AddToClassList("alert-timeline-panel");

            // Header
            var headerRow = new VisualElement();
            headerRow.AddToClassList("alert-timeline-header");

            var title = new Label("Alert Timeline");
            title.AddToClassList("alert-timeline-title");
            headerRow.Add(title);

            alertCountLabel = new Label("0 alerts");
            alertCountLabel.name = "alert-count";
            alertCountLabel.AddToClassList("alert-timeline-count");
            headerRow.Add(alertCountLabel);

            timelinePanel.Add(headerRow);

            // Filter buttons
            var filtersRow = new VisualElement();
            filtersRow.AddToClassList("alert-timeline-filters");

            string[] filterLabels = { "All", "Anomaly", "Correlated", "Security" };
            AlertCategory?[] filterValues = { null, AlertCategory.Anomaly, AlertCategory.Correlated, AlertCategory.Security };

            for (int i = 0; i < filterLabels.Length; i++)
            {
                var btn = new Button { text = filterLabels[i] };
                btn.AddToClassList("alert-filter-btn");
                if (i == 0) btn.AddToClassList("alert-filter-active");

                var filterVal = filterValues[i];
                btn.clicked += () => SetFilter(filterVal);

                filterButtons.Add(btn);
                filtersRow.Add(btn);
            }

            timelinePanel.Add(filtersRow);

            // Scroll view
            alertScrollView = new ScrollView();
            alertScrollView.name = "alert-scroll-view";
            alertScrollView.AddToClassList("alert-timeline-scroll");
            timelinePanel.Add(alertScrollView);

            mainContent.Add(timelinePanel);
        }
    }
}
