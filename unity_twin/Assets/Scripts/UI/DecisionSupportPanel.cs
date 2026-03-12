using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;
using RosMessageTypes.Miracle;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Operator Decision Support Panel.
    /// Slide-out panel that displays correlated alert details, root cause hypothesis,
    /// ranked recommended actions, and a what-if feed override slider.
    /// Subscribes to CorrelatedAlertEventSO and shows/hides on demand.
    /// </summary>
    public class DecisionSupportPanel : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private CorrelatedAlertEventSO onCorrelatedAlert;

        private VisualElement panelRoot;
        private Label situationLabel;
        private Label rootCauseLabel;
        private VisualElement actionsListContainer;
        private Slider whatIfSlider;
        private Label whatIfResultLabel;
        private Button closeButton;

        private bool isVisible;
        private CorrelatedAlertMsg currentAlert;

        // Contextual action mapping: anomaly type keywords -> recommended action text
        private static readonly Dictionary<string, string> ContextualActionMap = new()
        {
            { "wear+vibration", "Replace tool at next program break" },
            { "thermal+chatter", "Reduce RPM to recommended stable value" },
            { "force anomaly", "Reduce feed rate by 20%" },
        };

        void OnEnable()
        {
            if (onCorrelatedAlert != null)
                onCorrelatedAlert.Register(OnCorrelatedAlertReceived);
        }

        void OnDisable()
        {
            if (onCorrelatedAlert != null)
                onCorrelatedAlert.Unregister(OnCorrelatedAlertReceived);
        }

        void Start()
        {
            if (uiDocument == null) return;

            var root = uiDocument.rootVisualElement;

            panelRoot = root.Q<VisualElement>("decision-support-panel");
            if (panelRoot == null) return;

            situationLabel = root.Q<Label>("decision-situation");
            rootCauseLabel = root.Q<Label>("decision-root-cause");
            actionsListContainer = root.Q<VisualElement>("decision-actions-list");
            whatIfSlider = root.Q<Slider>("whatif-slider");
            whatIfResultLabel = root.Q<Label>("whatif-result");
            closeButton = root.Q<Button>("decision-close-btn");

            if (closeButton != null)
                closeButton.clicked += Hide;

            if (whatIfSlider != null)
                whatIfSlider.RegisterValueChangedCallback(OnWhatIfSliderChanged);

            // Start hidden
            isVisible = false;
            panelRoot.RemoveFromClassList("visible");
        }

        /// <summary>Show the decision support panel with slide-in animation.</summary>
        public void Show()
        {
            if (panelRoot == null) return;
            isVisible = true;
            panelRoot.AddToClassList("visible");
        }

        /// <summary>Hide the decision support panel with slide-out animation.</summary>
        public void Hide()
        {
            if (panelRoot == null) return;
            isVisible = false;
            panelRoot.RemoveFromClassList("visible");
        }

        /// <summary>Toggle panel visibility.</summary>
        public void Toggle()
        {
            if (isVisible)
                Hide();
            else
                Show();
        }

        private void OnCorrelatedAlertReceived(CorrelatedAlertMsg msg)
        {
            currentAlert = msg;
            PopulatePanel(msg);
            Show();
        }

        private void PopulatePanel(CorrelatedAlertMsg msg)
        {
            // Current Situation
            if (situationLabel != null)
            {
                string situation = !string.IsNullOrEmpty(msg.summary)
                    ? msg.summary
                    : $"Correlated alert on {msg.machine_id}: {string.Join(", ", msg.correlated_anomaly_types ?? Array.Empty<string>())}";
                situationLabel.text = situation;
            }

            // Root Cause
            if (rootCauseLabel != null)
            {
                string rootCause = !string.IsNullOrEmpty(msg.root_cause_hypothesis)
                    ? $"{msg.root_cause_hypothesis} (confidence: {msg.hypothesis_confidence:P0})"
                    : "--";
                rootCauseLabel.text = rootCause;
            }

            // Recommended Actions
            PopulateActions(msg);

            // Reset what-if slider
            if (whatIfSlider != null)
                whatIfSlider.value = 100f;

            if (whatIfResultLabel != null)
                whatIfResultLabel.text = "";
        }

        private void PopulateActions(CorrelatedAlertMsg msg)
        {
            if (actionsListContainer == null) return;
            actionsListContainer.Clear();

            var actions = new List<string>();

            // Add message-provided recommended actions
            if (msg.recommended_actions != null)
            {
                actions.AddRange(msg.recommended_actions);
            }

            // Add contextual actions based on anomaly type correlation
            string anomalySignature = string.Join("+",
                msg.correlated_anomaly_types ?? Array.Empty<string>()).ToLowerInvariant();

            foreach (var mapping in ContextualActionMap)
            {
                if (MatchesAnomalyPattern(anomalySignature, msg.correlated_anomaly_types, mapping.Key))
                {
                    if (!actions.Contains(mapping.Value))
                        actions.Add(mapping.Value);
                }
            }

            // Build ranked action items
            for (int i = 0; i < actions.Count; i++)
            {
                var actionItem = new VisualElement();
                actionItem.AddToClassList("decision-action-item");

                var rankLabel = new Label($"#{i + 1}");
                rankLabel.AddToClassList("decision-action-rank");
                actionItem.Add(rankLabel);

                var textLabel = new Label(actions[i]);
                textLabel.AddToClassList("decision-action-text");
                actionItem.Add(textLabel);

                actionsListContainer.Add(actionItem);
            }
        }

        /// <summary>
        /// Check if the anomaly types match a contextual pattern.
        /// Supports patterns like "wear+vibration", "thermal+chatter", "force anomaly".
        /// </summary>
        private static bool MatchesAnomalyPattern(string signature, string[] anomalyTypes, string pattern)
        {
            if (anomalyTypes == null || anomalyTypes.Length == 0)
                return false;

            // Split pattern by '+' or space for multi-keyword matching
            string[] keywords = pattern.Replace("+", " ").Split(' ', StringSplitOptions.RemoveEmptyEntries);

            foreach (string keyword in keywords)
            {
                bool found = false;
                foreach (string anomaly in anomalyTypes)
                {
                    if (anomaly != null && anomaly.ToLowerInvariant().Contains(keyword))
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                    return false;
            }

            return true;
        }

        private void OnWhatIfSliderChanged(ChangeEvent<float> evt)
        {
            if (whatIfResultLabel == null || currentAlert == null) return;

            float feedOverride = evt.newValue;
            float baseImpact = (float)currentAlert.estimated_impact;

            // Simulate: lower feed override reduces force proportionally, reducing anomaly impact
            float adjustedImpact = baseImpact * (feedOverride / 100f);
            float reduction = baseImpact - adjustedImpact;

            string result;
            if (feedOverride < 100f)
            {
                result = $"At {feedOverride:F0}% feed: estimated impact drops from " +
                         $"{baseImpact:F1} to {adjustedImpact:F1} " +
                         $"(reduction: {reduction:F1})";
            }
            else
            {
                result = "Current feed rate: no override applied.";
            }

            whatIfResultLabel.text = result;
        }

        void OnDestroy()
        {
            if (closeButton != null)
                closeButton.clicked -= Hide;

            if (whatIfSlider != null)
                whatIfSlider.UnregisterValueChangedCallback(OnWhatIfSliderChanged);
        }
    }
}
