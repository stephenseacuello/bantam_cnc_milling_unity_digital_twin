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
    /// ranked recommended actions, and what-if override sliders (feed + spindle RPM).
    /// Uses CuttingSimProxy predictions via MiracleBridge when available, with an
    /// improved analytical fallback model (non-linear force, Taylor tool life, chatter risk).
    /// </summary>
    public class DecisionSupportPanel : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private CorrelatedAlertEventSO onCorrelatedAlert;
        [SerializeField] private MiracleBridge miracleBridge;

        private VisualElement panelRoot;
        private Label situationLabel;
        private Label rootCauseLabel;
        private VisualElement actionsListContainer;

        // Feed override slider
        private Slider whatIfSlider;
        private Label whatIfFeedValueLabel;

        // Spindle RPM override slider
        private Slider spindleSlider;
        private Label spindleValueLabel;

        // Impact preview section
        private VisualElement impactPreviewContainer;
        private Label impactForceLabel;
        private Label impactToolLifeLabel;
        private Label impactSurfaceLabel;
        private Label impactChatterLabel;
        private Label computingIndicator;

        // Action buttons
        private Button applyOverrideButton;
        private Button revertButton;
        private Button closeButton;

        // Operator feedback UI
        private VisualElement feedbackContainer;
        private VisualElement starRatingContainer;
        private Button acceptButton;
        private Button rejectButton;
        private Label feedbackStatusLabel;
        private VisualElement postActionContainer;
        private Button actionConfirmedButton;
        private Button actionFailedButton;

        // Feedback state
        private string currentReferenceId;
        private string currentAnomalyType;
        private float overrideAppliedTime;
        private bool awaitingPostActionFeedback;
        private const float POST_ACTION_DELAY_SEC = 30f;

        private bool isVisible;
        private CorrelatedAlertMsg currentAlert;

        // Debounce state for slider changes
        private float lastSliderChangeTime;
        private const float SLIDER_DEBOUNCE_SEC = 0.3f;
        private bool predictionPending;
        private float pendingFeedPct;
        private float pendingSpindlePct;

        // Baseline cutting state cached from alert context
        private float baselineForceN = 145f;
        private float baselineToolLifeMin = 32f;
        private float baselineRa = 0.8f;
        private float baselineSpindleRPM = 8000f;
        private float baselineFeedRate = 500f;

        // Known unstable RPM zones (percentage of baseline) for chatter detection
        private static readonly (float min, float max)[] UnstableZones = new[]
        {
            (0.72f, 0.78f),  // 3rd harmonic of natural frequency
            (1.15f, 1.22f),  // near-resonance zone
        };

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

            // Feed override slider
            whatIfSlider = root.Q<Slider>("whatif-slider");
            whatIfFeedValueLabel = root.Q<Label>("whatif-feed-value");

            // Spindle RPM override slider
            spindleSlider = root.Q<Slider>("whatif-spindle-slider");
            spindleValueLabel = root.Q<Label>("whatif-spindle-value");

            // Impact preview labels
            impactPreviewContainer = root.Q<VisualElement>("whatif-impact-preview");
            impactForceLabel = root.Q<Label>("whatif-impact-force");
            impactToolLifeLabel = root.Q<Label>("whatif-impact-toollife");
            impactSurfaceLabel = root.Q<Label>("whatif-impact-surface");
            impactChatterLabel = root.Q<Label>("whatif-impact-chatter");
            computingIndicator = root.Q<Label>("whatif-computing");

            // Action buttons
            applyOverrideButton = root.Q<Button>("whatif-apply-btn");
            revertButton = root.Q<Button>("whatif-revert-btn");
            closeButton = root.Q<Button>("decision-close-btn");

            if (closeButton != null)
                closeButton.clicked += Hide;

            if (whatIfSlider != null)
            {
                whatIfSlider.lowValue = 50f;
                whatIfSlider.highValue = 120f;
                whatIfSlider.value = 100f;
                whatIfSlider.RegisterValueChangedCallback(OnWhatIfSliderChanged);
            }

            if (spindleSlider != null)
            {
                spindleSlider.lowValue = 50f;
                spindleSlider.highValue = 120f;
                spindleSlider.value = 100f;
                spindleSlider.label = "Spindle RPM Override";
                spindleSlider.RegisterValueChangedCallback(OnSpindleSliderChanged);
            }

            if (applyOverrideButton != null)
                applyOverrideButton.clicked += OnApplyOverride;

            if (revertButton != null)
                revertButton.clicked += OnRevertOverride;

            // Dynamically create UI elements that may not exist in UXML
            EnsurePreviewUIExists();

            // Start hidden
            isVisible = false;
            panelRoot.RemoveFromClassList("visible");

            // Auto-resolve MiracleBridge if not assigned
            if (miracleBridge == null)
                miracleBridge = MiracleBridge.Instance;
        }

        /// <summary>
        /// Create impact preview UI elements programmatically if they are not defined
        /// in the UXML template. This ensures the panel works even without UXML updates.
        /// </summary>
        private void EnsurePreviewUIExists()
        {
            if (panelRoot == null) return;

            // Create spindle slider if not in UXML
            if (spindleSlider == null && whatIfSlider != null)
            {
                spindleSlider = new Slider("Spindle RPM Override", 50f, 120f)
                {
                    name = "whatif-spindle-slider",
                    value = 100f
                };
                spindleSlider.AddToClassList("whatif-slider");
                spindleSlider.RegisterValueChangedCallback(OnSpindleSliderChanged);

                spindleValueLabel = new Label("RPM: 100%");
                spindleValueLabel.name = "whatif-spindle-value";
                spindleValueLabel.AddToClassList("whatif-value-label");

                // Insert after feed slider
                int feedIdx = panelRoot.IndexOf(whatIfSlider.parent ?? whatIfSlider);
                if (feedIdx >= 0)
                {
                    panelRoot.Insert(feedIdx + 1, spindleSlider);
                    panelRoot.Insert(feedIdx + 2, spindleValueLabel);
                }
                else
                {
                    panelRoot.Add(spindleSlider);
                    panelRoot.Add(spindleValueLabel);
                }
            }

            // Create impact preview container if not in UXML
            if (impactPreviewContainer == null)
            {
                impactPreviewContainer = new VisualElement { name = "whatif-impact-preview" };
                impactPreviewContainer.AddToClassList("whatif-impact-preview");

                var header = new Label("Preview Impact");
                header.AddToClassList("whatif-impact-header");
                impactPreviewContainer.Add(header);

                computingIndicator = new Label("") { name = "whatif-computing" };
                computingIndicator.AddToClassList("whatif-computing");
                impactPreviewContainer.Add(computingIndicator);

                impactForceLabel = new Label("Force:     -- -> --") { name = "whatif-impact-force" };
                impactForceLabel.AddToClassList("whatif-impact-row");
                impactPreviewContainer.Add(impactForceLabel);

                impactToolLifeLabel = new Label("Tool Life: -- -> --") { name = "whatif-impact-toollife" };
                impactToolLifeLabel.AddToClassList("whatif-impact-row");
                impactPreviewContainer.Add(impactToolLifeLabel);

                impactSurfaceLabel = new Label("Surface:   -- -> --") { name = "whatif-impact-surface" };
                impactSurfaceLabel.AddToClassList("whatif-impact-row");
                impactPreviewContainer.Add(impactSurfaceLabel);

                impactChatterLabel = new Label("Chatter:   -- -> --") { name = "whatif-impact-chatter" };
                impactChatterLabel.AddToClassList("whatif-impact-row");
                impactPreviewContainer.Add(impactChatterLabel);

                panelRoot.Add(impactPreviewContainer);
            }

            // Create operator feedback section
            if (feedbackContainer == null)
            {
                feedbackContainer = new VisualElement { name = "feedback-container" };
                feedbackContainer.AddToClassList("feedback-container");

                var feedbackHeader = new Label("Operator Feedback");
                feedbackHeader.AddToClassList("feedback-header");
                feedbackContainer.Add(feedbackHeader);

                // Accept / Reject buttons for recommendations
                var recommendationRow = new VisualElement();
                recommendationRow.style.flexDirection = FlexDirection.Row;
                recommendationRow.AddToClassList("feedback-button-row");

                acceptButton = new Button(OnAcceptRecommendation) { text = "Accept", name = "feedback-accept-btn" };
                acceptButton.AddToClassList("feedback-accept-btn");
                recommendationRow.Add(acceptButton);

                rejectButton = new Button(OnRejectRecommendation) { text = "Reject", name = "feedback-reject-btn" };
                rejectButton.AddToClassList("feedback-reject-btn");
                recommendationRow.Add(rejectButton);

                feedbackContainer.Add(recommendationRow);

                // Star rating (1-5) for explanation quality
                starRatingContainer = new VisualElement { name = "star-rating-container" };
                starRatingContainer.style.flexDirection = FlexDirection.Row;
                starRatingContainer.AddToClassList("star-rating-container");
                var ratingLabel = new Label("Rate explanation:");
                ratingLabel.AddToClassList("star-rating-label");
                starRatingContainer.Add(ratingLabel);
                for (int star = 1; star <= 5; star++)
                {
                    int starValue = star;
                    var starBtn = new Button(() => OnRateExplanation(starValue))
                    {
                        text = "\u2605",
                        name = $"star-{star}"
                    };
                    starBtn.AddToClassList("star-btn");
                    starRatingContainer.Add(starBtn);
                }
                feedbackContainer.Add(starRatingContainer);

                // Post-action feedback (shown after override applied + delay)
                postActionContainer = new VisualElement { name = "post-action-feedback" };
                postActionContainer.AddToClassList("post-action-feedback");
                postActionContainer.style.display = DisplayStyle.None;

                var postLabel = new Label("Did this action help?");
                postLabel.AddToClassList("post-action-label");
                postActionContainer.Add(postLabel);

                var postRow = new VisualElement();
                postRow.style.flexDirection = FlexDirection.Row;

                actionConfirmedButton = new Button(OnActionConfirmed) { text = "Yes", name = "feedback-confirmed-btn" };
                actionConfirmedButton.AddToClassList("feedback-confirmed-btn");
                postRow.Add(actionConfirmedButton);

                actionFailedButton = new Button(OnActionFailed) { text = "No", name = "feedback-failed-btn" };
                actionFailedButton.AddToClassList("feedback-failed-btn");
                postRow.Add(actionFailedButton);

                postActionContainer.Add(postRow);
                feedbackContainer.Add(postActionContainer);

                feedbackStatusLabel = new Label("");
                feedbackStatusLabel.name = "feedback-status";
                feedbackStatusLabel.AddToClassList("feedback-status");
                feedbackContainer.Add(feedbackStatusLabel);

                panelRoot.Add(feedbackContainer);
            }

            // Create Apply/Revert buttons if not in UXML
            if (applyOverrideButton == null)
            {
                var buttonRow = new VisualElement();
                buttonRow.AddToClassList("whatif-button-row");
                buttonRow.style.flexDirection = FlexDirection.Row;

                applyOverrideButton = new Button(OnApplyOverride) { text = "Apply Override", name = "whatif-apply-btn" };
                applyOverrideButton.AddToClassList("whatif-apply-btn");

                revertButton = new Button(OnRevertOverride) { text = "Revert to Programmed", name = "whatif-revert-btn" };
                revertButton.AddToClassList("whatif-revert-btn");

                buttonRow.Add(applyOverrideButton);
                buttonRow.Add(revertButton);
                panelRoot.Add(buttonRow);
            }
        }

        void Update()
        {
            // Process debounced slider changes
            if (predictionPending && Time.time - lastSliderChangeTime >= SLIDER_DEBOUNCE_SEC)
            {
                predictionPending = false;
                RequestWhatIfPrediction(pendingFeedPct, pendingSpindlePct);
            }

            // Show post-action feedback prompt after delay
            if (awaitingPostActionFeedback && Time.time - overrideAppliedTime >= POST_ACTION_DELAY_SEC)
            {
                if (postActionContainer != null)
                    postActionContainer.style.display = DisplayStyle.Flex;
            }
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
            // Track alert context for feedback
            currentAnomalyType = (msg.correlated_anomaly_types != null && msg.correlated_anomaly_types.Length > 0)
                ? msg.correlated_anomaly_types[0]
                : "unknown";
            currentReferenceId = $"{msg.machine_id}_{currentAnomalyType}_{Time.time:F0}";
            awaitingPostActionFeedback = false;
            if (postActionContainer != null)
                postActionContainer.style.display = DisplayStyle.None;
            if (feedbackStatusLabel != null)
                feedbackStatusLabel.text = "";

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

            // Reset sliders
            if (whatIfSlider != null)
                whatIfSlider.value = 100f;
            if (spindleSlider != null)
                spindleSlider.value = 100f;

            // Clear impact preview
            ClearImpactPreview();
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

        // --- Slider Callbacks (debounced) ---

        private void OnWhatIfSliderChanged(ChangeEvent<float> evt)
        {
            float feedPct = evt.newValue;
            float spindlePct = spindleSlider != null ? spindleSlider.value : 100f;

            UpdateSliderValueLabels(feedPct, spindlePct);
            SchedulePrediction(feedPct, spindlePct);
        }

        private void OnSpindleSliderChanged(ChangeEvent<float> evt)
        {
            float feedPct = whatIfSlider != null ? whatIfSlider.value : 100f;
            float spindlePct = evt.newValue;

            UpdateSliderValueLabels(feedPct, spindlePct);
            SchedulePrediction(feedPct, spindlePct);
        }

        private void UpdateSliderValueLabels(float feedPct, float spindlePct)
        {
            if (whatIfFeedValueLabel != null)
                whatIfFeedValueLabel.text = $"Feed: {feedPct:F0}% ({baselineFeedRate * feedPct / 100f:F0} mm/min)";

            if (spindleValueLabel != null)
                spindleValueLabel.text = $"RPM: {spindlePct:F0}% ({baselineSpindleRPM * spindlePct / 100f:F0} RPM)";
        }

        private void SchedulePrediction(float feedPct, float spindlePct)
        {
            pendingFeedPct = feedPct;
            pendingSpindlePct = spindlePct;
            lastSliderChangeTime = Time.time;
            predictionPending = true;

            // Show computing indicator
            if (computingIndicator != null)
                computingIndicator.text = "Computing...";
        }

        // --- Prediction Request ---

        /// <summary>
        /// Request a what-if prediction. Tries MiracleBridge prediction service first,
        /// falls back to improved analytical model if unavailable.
        /// </summary>
        private void RequestWhatIfPrediction(float feedOverridePct, float spindleOverridePct)
        {
            if (currentAlert == null)
            {
                ApplyAnalyticalFallback(feedOverridePct, spindleOverridePct);
                return;
            }

            if (miracleBridge == null)
                miracleBridge = MiracleBridge.Instance;

            if (miracleBridge != null)
            {
                miracleBridge.RequestPrediction(feedOverridePct, spindleOverridePct, result =>
                {
                    if (result != null)
                    {
                        DisplayPredictionResult(result);
                    }
                    else
                    {
                        // Service unavailable — use analytical fallback
                        ApplyAnalyticalFallback(feedOverridePct, spindleOverridePct);
                    }
                });
            }
            else
            {
                ApplyAnalyticalFallback(feedOverridePct, spindleOverridePct);
            }
        }

        // --- Improved Analytical Fallback Model ---

        /// <summary>
        /// Analytical fallback when prediction service is unavailable.
        /// Uses non-linear models instead of simple linear scaling:
        ///   - Force proportional to feed^0.8 (chip load saturation)
        ///   - Tool life via Taylor equation ratio: life proportional to (1/speed)^(1/n)
        ///   - Surface roughness proportional to feed^2 / (nose_radius) approximation
        ///   - Chatter risk from known unstable RPM zones
        /// </summary>
        private void ApplyAnalyticalFallback(float feedPct, float spindlePct)
        {
            float feedRatio = feedPct / 100f;
            float spindleRatio = spindlePct / 100f;

            // Force: F proportional to feed^0.8 (non-linear chip load saturation)
            // At low feed, force doesn't drop linearly because edge forces dominate
            float forceMultiplier = Mathf.Pow(feedRatio, 0.8f);
            float predictedForce = baselineForceN * forceMultiplier;
            float forceDeltaPct = (predictedForce - baselineForceN) / baselineForceN * 100f;

            // Tool life: Taylor equation — T proportional to (1/V)^(1/n)
            // For HSS on 6061-T6: n ~ 0.125
            // Life ratio = (V_base / V_new)^(1/n) = (1/spindleRatio)^(1/0.125) = (1/spindleRatio)^8
            // Also account for feed effect: life proportional to (1/feed)^(a/n) where a~0.5
            // -> feed factor = (1/feedRatio)^(0.5/0.125) = (1/feedRatio)^4
            float taylorN = 0.125f;
            float taylorA = 0.5f;
            float speedLifeFactor = Mathf.Pow(1f / Mathf.Max(spindleRatio, 0.01f), 1f / taylorN);
            float feedLifeFactor = Mathf.Pow(1f / Mathf.Max(feedRatio, 0.01f), taylorA / taylorN);

            // Clamp life multiplier to avoid extreme values from Taylor exponents
            float lifeMultiplier = Mathf.Clamp(speedLifeFactor * feedLifeFactor, 0.01f, 100f);
            float predictedToolLife = baselineToolLifeMin * lifeMultiplier;
            float toolLifeDeltaPct = (predictedToolLife - baselineToolLifeMin) / baselineToolLifeMin * 100f;

            // Surface roughness: Ra proportional to f^2 / (8 * r) — theoretical finish
            // Ratio: Ra_new / Ra_base = (feed_new / feed_base)^2 = feedRatio^2
            // Spindle effect: higher RPM generally improves finish slightly
            float raMultiplier = Mathf.Pow(feedRatio, 2f) * Mathf.Pow(1f / Mathf.Max(spindleRatio, 0.01f), 0.1f);
            float predictedRa = baselineRa * raMultiplier;
            string qualityChange = predictedRa < baselineRa * 0.95f ? "improved"
                : predictedRa > baselineRa * 1.05f ? "degraded"
                : "unchanged";

            // Chatter risk: check if RPM override moves into known unstable zone
            string baseChatter = EvaluateChatterRisk(1.0f);
            string predictedChatter = EvaluateChatterRisk(spindleRatio);

            var result = new MiracleBridge.PredictionResult
            {
                BaselineForceN = baselineForceN,
                PredictedForceN = predictedForce,
                ForceDeltaPct = forceDeltaPct,
                BaselineToolLifeMin = baselineToolLifeMin,
                PredictedToolLifeMin = predictedToolLife,
                ToolLifeDeltaPct = toolLifeDeltaPct,
                BaselineRa = baselineRa,
                PredictedRa = predictedRa,
                QualityChange = qualityChange,
                BaselineChatterRisk = baseChatter,
                PredictedChatterRisk = predictedChatter,
                Confidence = 0.6f,  // Lower confidence for analytical fallback
                FromService = false
            };

            DisplayPredictionResult(result);
        }

        /// <summary>
        /// Evaluate chatter risk based on RPM ratio relative to known unstable zones.
        /// </summary>
        private static string EvaluateChatterRisk(float rpmRatio)
        {
            foreach (var zone in UnstableZones)
            {
                if (rpmRatio >= zone.min && rpmRatio <= zone.max)
                    return "HIGH";
                // Near an unstable zone boundary
                float distToZone = Mathf.Min(
                    Mathf.Abs(rpmRatio - zone.min),
                    Mathf.Abs(rpmRatio - zone.max));
                if (distToZone < 0.03f)
                    return "MEDIUM";
            }
            return "LOW";
        }

        // --- Display ---

        private void DisplayPredictionResult(MiracleBridge.PredictionResult result)
        {
            if (computingIndicator != null)
                computingIndicator.text = result.FromService ? "" : "(analytical estimate)";

            if (impactForceLabel != null)
            {
                string sign = result.ForceDeltaPct >= 0 ? "+" : "";
                impactForceLabel.text =
                    $"Force:     {result.BaselineForceN:F0}N -> {result.PredictedForceN:F0}N ({sign}{result.ForceDeltaPct:F0}%)";
            }

            if (impactToolLifeLabel != null)
            {
                string sign = result.ToolLifeDeltaPct >= 0 ? "+" : "";
                impactToolLifeLabel.text =
                    $"Tool Life: {result.BaselineToolLifeMin:F0}min -> {result.PredictedToolLifeMin:F0}min ({sign}{result.ToolLifeDeltaPct:F0}%)";
            }

            if (impactSurfaceLabel != null)
            {
                impactSurfaceLabel.text =
                    $"Surface:   Ra {result.BaselineRa:F1} -> Ra {result.PredictedRa:F1} ({result.QualityChange})";
            }

            if (impactChatterLabel != null)
            {
                string stability = result.PredictedChatterRisk == result.BaselineChatterRisk
                    ? "stable"
                    : "CHANGED";
                impactChatterLabel.text =
                    $"Chatter:   {result.BaselineChatterRisk} -> {result.PredictedChatterRisk} ({stability})";
            }
        }

        private void ClearImpactPreview()
        {
            if (computingIndicator != null) computingIndicator.text = "";
            if (impactForceLabel != null) impactForceLabel.text = "Force:     -- -> --";
            if (impactToolLifeLabel != null) impactToolLifeLabel.text = "Tool Life: -- -> --";
            if (impactSurfaceLabel != null) impactSurfaceLabel.text = "Surface:   -- -> --";
            if (impactChatterLabel != null) impactChatterLabel.text = "Chatter:   -- -> --";
        }

        // --- Override Application ---

        private void OnApplyOverride()
        {
            float feedPct = whatIfSlider != null ? whatIfSlider.value : 100f;
            float spindlePct = spindleSlider != null ? spindleSlider.value : 100f;

            if (miracleBridge == null)
                miracleBridge = MiracleBridge.Instance;

            if (miracleBridge != null)
            {
                string reason = $"Operator what-if override: feed={feedPct:F0}%, spindle={spindlePct:F0}%";
                miracleBridge.PublishFeedOverride(feedPct, spindlePct, reason);
                Debug.Log($"[DecisionSupportPanel] Applied override: feed={feedPct}%, spindle={spindlePct}%");

                // Start post-action feedback timer
                overrideAppliedTime = Time.time;
                awaitingPostActionFeedback = true;
            }
            else
            {
                Debug.LogWarning("[DecisionSupportPanel] Cannot apply override: MiracleBridge not available.");
            }
        }

        private void OnRevertOverride()
        {
            // Reset sliders to 100%
            if (whatIfSlider != null)
                whatIfSlider.value = 100f;
            if (spindleSlider != null)
                spindleSlider.value = 100f;

            if (miracleBridge == null)
                miracleBridge = MiracleBridge.Instance;

            if (miracleBridge != null)
            {
                miracleBridge.PublishFeedOverride(100f, 100f, "Operator reverted to programmed values");
                Debug.Log("[DecisionSupportPanel] Reverted to programmed values (100%/100%).");
            }

            ClearImpactPreview();
        }

        // --- Operator Feedback Handlers ---

        private void OnAcceptRecommendation()
        {
            PublishFeedback("RECOMMENDATION_ACCEPTED", 1.0f, "accepted");
            if (feedbackStatusLabel != null)
                feedbackStatusLabel.text = "Recommendation accepted.";
        }

        private void OnRejectRecommendation()
        {
            PublishFeedback("RECOMMENDATION_REJECTED", 0.0f, "rejected");
            if (feedbackStatusLabel != null)
                feedbackStatusLabel.text = "Recommendation rejected.";
        }

        private void OnRateExplanation(int stars)
        {
            PublishFeedback("EXPLANATION_RATED", (float)stars, "rated");
            if (feedbackStatusLabel != null)
                feedbackStatusLabel.text = $"Rated {stars}/5 stars.";
        }

        private void OnActionConfirmed()
        {
            PublishFeedback("ACTION_CONFIRMED", 1.0f, "confirmed effective");
            awaitingPostActionFeedback = false;
            if (postActionContainer != null)
                postActionContainer.style.display = DisplayStyle.None;
            if (feedbackStatusLabel != null)
                feedbackStatusLabel.text = "Action confirmed effective.";
        }

        private void OnActionFailed()
        {
            PublishFeedback("ACTION_FAILED", 0.0f, "confirmed ineffective");
            awaitingPostActionFeedback = false;
            if (postActionContainer != null)
                postActionContainer.style.display = DisplayStyle.None;
            if (feedbackStatusLabel != null)
                feedbackStatusLabel.text = "Action marked as ineffective.";
        }

        private void PublishFeedback(string feedbackType, float rating, string actionTaken)
        {
            if (miracleBridge == null)
                miracleBridge = MiracleBridge.Instance;

            if (miracleBridge != null)
            {
                miracleBridge.PublishOperatorFeedback(
                    feedbackType,
                    currentAnomalyType ?? "",
                    rating,
                    actionTaken,
                    currentReferenceId ?? ""
                );
            }
            else
            {
                Debug.LogWarning("[DecisionSupportPanel] Cannot publish feedback: MiracleBridge not available.");
            }
        }

        void OnDestroy()
        {
            if (closeButton != null)
                closeButton.clicked -= Hide;

            if (whatIfSlider != null)
                whatIfSlider.UnregisterValueChangedCallback(OnWhatIfSliderChanged);

            if (spindleSlider != null)
                spindleSlider.UnregisterValueChangedCallback(OnSpindleSliderChanged);

            if (applyOverrideButton != null)
                applyOverrideButton.clicked -= OnApplyOverride;

            if (revertButton != null)
                revertButton.clicked -= OnRevertOverride;
        }
    }
}
