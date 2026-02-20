using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.Core;
using RosMessageTypes.Miracle;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Toast popup notifications for anomaly and security alerts.
    /// </summary>
    public class AlertNotification : MonoBehaviour
    {
        [SerializeField] private AnomalyAlertEventSO anomalyEvent;
        [SerializeField] private SecurityAlertEventSO securityEvent;

        [Header("Settings")]
        [SerializeField] private float displayDuration = 5f;
        [SerializeField] private int maxVisibleAlerts = 3;

        public struct AlertData
        {
            public string title;
            public string message;
            public AlertSeverity severity;
            public float timestamp;
        }

        public enum AlertSeverity { Info, Warning, Critical, Security }

        private readonly Queue<AlertData> activeAlerts = new();
        private readonly List<AlertData> alertHistory = new();

        void OnEnable()
        {
            if (anomalyEvent != null) anomalyEvent.Register(OnAnomaly);
            if (securityEvent != null) securityEvent.Register(OnSecurity);
        }

        void OnDisable()
        {
            if (anomalyEvent != null) anomalyEvent.Unregister(OnAnomaly);
            if (securityEvent != null) securityEvent.Unregister(OnSecurity);
        }

        private void OnAnomaly(AnomalyAlertMsg msg)
        {
            var severity = msg.confidence > 0.8f ? AlertSeverity.Critical :
                          msg.confidence > 0.5f ? AlertSeverity.Warning : AlertSeverity.Info;

            ShowAlert(new AlertData
            {
                title = $"Anomaly: {msg.anomaly_type}",
                message = $"Confidence: {msg.confidence:P0}",
                severity = severity,
                timestamp = Time.time
            });
        }

        private void OnSecurity(SecurityAlertMsg msg)
        {
            ShowAlert(new AlertData
            {
                title = $"Security: {msg.category}",
                message = msg.description,
                severity = AlertSeverity.Security,
                timestamp = Time.time
            });
        }

        private void ShowAlert(AlertData alert)
        {
            activeAlerts.Enqueue(alert);
            alertHistory.Add(alert);

            while (activeAlerts.Count > maxVisibleAlerts)
                activeAlerts.Dequeue();

            Debug.Log($"[Alert] [{alert.severity}] {alert.title}: {alert.message}");
        }

        void Update()
        {
            // Remove expired alerts
            while (activeAlerts.Count > 0 &&
                   Time.time - activeAlerts.Peek().timestamp > displayDuration)
            {
                activeAlerts.Dequeue();
            }
        }

        public AlertData[] GetActiveAlerts()
        {
            return activeAlerts.ToArray();
        }

        public int AlertCount => alertHistory.Count;
    }
}
