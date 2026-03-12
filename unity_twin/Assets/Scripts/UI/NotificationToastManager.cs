using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;
using RosMessageTypes.Miracle;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Priority-sorted notification toast system with severity-based routing,
    /// audio feedback for critical alerts, and inline action buttons.
    /// Replaces basic AlertNotification with full UI Toolkit toast rendering.
    /// </summary>
    public class NotificationToastManager : MonoBehaviour
    {
        [Header("Event Sources")]
        [SerializeField] private AnomalyAlertEventSO anomalyAlertEvent;
        [SerializeField] private CorrelatedAlertEventSO correlatedAlertEvent;
        [SerializeField] private SecurityAlertEventSO securityAlertEvent;

        [Header("UI")]
        [SerializeField] private UIDocument uiDocument;

        [Header("Audio")]
        [SerializeField] private AudioSource audioSource;
        [SerializeField] private AudioClip warningSound;
        [SerializeField] private AudioClip criticalSound;
        [SerializeField] private AudioClip securitySound;

        [Header("Configuration")]
        [SerializeField] private int maxVisibleToasts = 4;
        [SerializeField] private float normalDuration = 5f;
        [SerializeField] private float warningDuration = 8f;
        [SerializeField] private float criticalDuration = 15f;
        [SerializeField] private bool criticalRequiresDismiss = true;

        private VisualElement _toastContainer;
        private readonly List<ToastInstance> _activeToasts = new List<ToastInstance>();
        private readonly Queue<ToastData> _pendingQueue = new Queue<ToastData>();

        // ── Data types ───────────────────────────────────────────────────

        public enum ToastPriority { Info, Warning, Critical, Security }

        public struct ToastData
        {
            public string title;
            public string message;
            public string machineId;
            public float severity;
            public ToastPriority priority;
            public string actionLabel;
            public Action onAction;
        }

        private class ToastInstance
        {
            public VisualElement element;
            public float spawnTime;
            public float duration;
            public bool requiresDismiss;
            public bool dismissed;
        }

        // ── Lifecycle ────────────────────────────────────────────────────

        void OnEnable()
        {
            var root = uiDocument != null ? uiDocument.rootVisualElement : null;
            _toastContainer = root?.Q("toast-container");
            if (_toastContainer == null && root != null)
            {
                _toastContainer = new VisualElement();
                _toastContainer.name = "toast-container";
                _toastContainer.AddToClassList("toast-container");
                root.Add(_toastContainer);
            }

            if (anomalyAlertEvent != null) anomalyAlertEvent.Register(OnAnomalyAlert);
            if (correlatedAlertEvent != null) correlatedAlertEvent.Register(OnCorrelatedAlert);
            if (securityAlertEvent != null) securityAlertEvent.Register(OnSecurityAlert);
        }

        void OnDisable()
        {
            if (anomalyAlertEvent != null) anomalyAlertEvent.Unregister(OnAnomalyAlert);
            if (correlatedAlertEvent != null) correlatedAlertEvent.Unregister(OnCorrelatedAlert);
            if (securityAlertEvent != null) securityAlertEvent.Unregister(OnSecurityAlert);
        }

        void Update()
        {
            // Remove expired toasts (iterate backwards for safe removal)
            for (int i = _activeToasts.Count - 1; i >= 0; i--)
            {
                var toast = _activeToasts[i];
                if (toast.dismissed ||
                    (!toast.requiresDismiss && Time.time - toast.spawnTime > toast.duration))
                {
                    if (toast.element != null)
                    {
                        toast.element.AddToClassList("toast-exit");
                        var el = toast.element;
                        el.schedule.Execute(() => el.RemoveFromHierarchy()).ExecuteLater(300);
                    }
                    _activeToasts.RemoveAt(i);
                }
            }

            // Show pending toasts when space is available
            while (_pendingQueue.Count > 0 && _activeToasts.Count < maxVisibleToasts)
            {
                ShowToast(_pendingQueue.Dequeue());
            }
        }

        // ── Event handlers ───────────────────────────────────────────────

        private void OnAnomalyAlert(AnomalyAlertMsg msg)
        {
            float sev = (float)msg.severity;
            var priority = sev >= 0.8f ? ToastPriority.Critical
                         : sev >= 0.5f ? ToastPriority.Warning
                         : ToastPriority.Info;

            string actionLabel = null;
            if (msg.anomaly_type != null)
            {
                if (msg.anomaly_type.Contains("wear"))
                    actionLabel = "Inspect Tool";
                else if (msg.anomaly_type.Contains("vibration") || msg.anomaly_type.Contains("chatter"))
                    actionLabel = "Check Spindle";
            }

            if (msg.requires_immediate_stop)
                priority = ToastPriority.Critical;

            EnqueueToast(new ToastData
            {
                title = (msg.anomaly_type ?? "anomaly").Replace('_', ' '),
                message = $"Severity: {sev:P0} on {msg.machine_id}",
                machineId = msg.machine_id,
                severity = sev,
                priority = priority,
                actionLabel = actionLabel,
            });
        }

        private void OnCorrelatedAlert(CorrelatedAlertMsg msg)
        {
            float maxSeverity = msg.anomaly_severities != null && msg.anomaly_severities.Length > 0
                ? (float)msg.anomaly_severities.Max()
                : (float)msg.estimated_impact;

            EnqueueToast(new ToastData
            {
                title = "Correlated Alert",
                message = msg.root_cause_hypothesis,
                machineId = msg.machine_id,
                severity = maxSeverity,
                priority = maxSeverity >= 0.8f ? ToastPriority.Critical : ToastPriority.Warning,
                actionLabel = "View Details",
            });
        }

        private void OnSecurityAlert(SecurityAlertMsg msg)
        {
            float severityValue = msg.severity?.ToUpperInvariant() switch
            {
                "CRITICAL" => 1.0f,
                "HIGH" => 0.9f,
                "MEDIUM" => 0.6f,
                "LOW" => 0.3f,
                _ => (float)msg.confidence
            };

            EnqueueToast(new ToastData
            {
                title = "Security Alert",
                message = msg.description,
                severity = severityValue,
                machineId = msg.source_node,
                priority = ToastPriority.Security,
                actionLabel = msg.requires_isolation ? "Isolate Node" : "Investigate",
            });
        }

        // ── Enqueueing & routing ─────────────────────────────────────────

        private void EnqueueToast(ToastData data)
        {
            // Play audio feedback based on priority
            if (audioSource != null)
            {
                var clip = data.priority switch
                {
                    ToastPriority.Critical => criticalSound,
                    ToastPriority.Security => securitySound,
                    ToastPriority.Warning  => warningSound,
                    _ => null
                };
                if (clip != null)
                    audioSource.PlayOneShot(clip);
            }

            // Critical and security toasts bypass the queue
            if (data.priority == ToastPriority.Critical || data.priority == ToastPriority.Security)
            {
                ShowToast(data);
            }
            else
            {
                _pendingQueue.Enqueue(data);
            }
        }

        // ── Toast rendering ──────────────────────────────────────────────

        private void ShowToast(ToastData data)
        {
            if (_toastContainer == null) return;

            var toast = new VisualElement();
            toast.AddToClassList("toast");
            toast.AddToClassList($"toast-{data.priority.ToString().ToLower()}");

            // Severity color bar (left edge)
            var bar = new VisualElement();
            bar.AddToClassList("toast-severity-bar");
            bar.style.backgroundColor = GetSeverityColor(data.severity);
            toast.Add(bar);

            // Content area
            var content = new VisualElement();
            content.AddToClassList("toast-content");

            // Title row with dismiss button
            var titleRow = new VisualElement();
            titleRow.AddToClassList("toast-title-row");

            var title = new Label(data.title);
            title.AddToClassList("toast-title");
            titleRow.Add(title);

            var dismissBtn = new Button(() =>
            {
                toast.AddToClassList("toast-exit");
                var instance = _activeToasts.Find(t => t.element == toast);
                if (instance != null) instance.dismissed = true;
            });
            dismissBtn.text = "\u00d7";
            dismissBtn.AddToClassList("toast-dismiss");
            titleRow.Add(dismissBtn);

            content.Add(titleRow);

            // Message body
            var msg = new Label(data.message);
            msg.AddToClassList("toast-message");
            content.Add(msg);

            // Action button (optional)
            if (!string.IsNullOrEmpty(data.actionLabel))
            {
                var onAction = data.onAction;
                var actionBtn = new Button(() => onAction?.Invoke());
                actionBtn.text = data.actionLabel;
                actionBtn.AddToClassList("toast-action-btn");
                content.Add(actionBtn);
            }

            toast.Add(content);

            // Insert at top so newest toast appears first
            _toastContainer.Insert(0, toast);

            float duration = data.priority switch
            {
                ToastPriority.Critical => criticalDuration,
                ToastPriority.Warning  => warningDuration,
                _ => normalDuration,
            };

            _activeToasts.Add(new ToastInstance
            {
                element = toast,
                spawnTime = Time.time,
                duration = duration,
                requiresDismiss = criticalRequiresDismiss &&
                                  (data.priority == ToastPriority.Critical ||
                                   data.priority == ToastPriority.Security),
            });
        }

        // ── Helpers ──────────────────────────────────────────────────────

        private static Color GetSeverityColor(float severity)
        {
            if (severity >= 0.8f) return new Color(0.9f, 0.2f, 0.2f);
            if (severity >= 0.5f) return new Color(0.9f, 0.7f, 0.1f);
            return new Color(0.2f, 0.8f, 0.3f);
        }

        // ── Public API ───────────────────────────────────────────────────

        /// <summary>Number of toasts currently visible on screen.</summary>
        public int ActiveToastCount => _activeToasts.Count;

        /// <summary>Number of toasts waiting in the queue.</summary>
        public int PendingCount => _pendingQueue.Count;

        /// <summary>Dismiss all active toasts and clear the pending queue.</summary>
        public void DismissAll()
        {
            foreach (var toast in _activeToasts)
                toast.dismissed = true;
            _pendingQueue.Clear();
        }

        /// <summary>
        /// Programmatically enqueue a toast from external code.
        /// </summary>
        public void ShowCustomToast(string title, string message, float severity,
                                     ToastPriority priority = ToastPriority.Info,
                                     string actionLabel = null, Action onAction = null)
        {
            EnqueueToast(new ToastData
            {
                title = title,
                message = message,
                severity = severity,
                priority = priority,
                actionLabel = actionLabel,
                onAction = onAction,
            });
        }
    }
}
