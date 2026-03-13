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

        // Ranked action cards UI
        private VisualElement rankedActionsContainer;
        private Button compareToggleButton;
        private VisualElement comparisonTable;
        private VisualElement doNothingRiskBar;
        private Label doNothingRiskLabel;
        private bool compareMode;
        private int highlightedActionIndex;

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

            // Create ranked actions UI
            EnsureRankedActionsUIExists();

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
                RequestCausalPreview(feedOverridePct, spindleOverridePct);
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

            // Always request causal trajectory preview alongside the prediction
            RequestCausalPreview(feedOverridePct, spindleOverridePct);
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

            if (compareToggleButton != null)
                compareToggleButton.clicked -= OnToggleCompare;
        }

        // ========================================================================
        // Ranked Action Cards — Alternative Action Ranking Display
        // ========================================================================

        /// <summary>
        /// Data for a single ranked corrective action received from the cognitive layer.
        /// </summary>
        [Serializable]
        public class RankedActionData
        {
            public string actionType;
            public string description;
            public float feedOverridePct;
            public float speedOverridePct;
            public float forceReductionPct;
            public float rulExtensionMin;
            public float surfaceImprovementPct;
            public float cycleTimeImpactPct;
            public float riskReduction;
            public float confidence;
            public string reasoning;
            public float score;
        }

        /// <summary>
        /// Full ranked action result from the cognitive action ranker.
        /// </summary>
        [Serializable]
        public class RankedActionResult
        {
            public string situationSummary;
            public List<RankedActionData> rankedActions;
            public int recommendedIndex;
            public float doNothingRisk;
        }

        /// <summary>
        /// Build the ranked-actions UI section programmatically.
        /// Called once from EnsurePreviewUIExists or Start.
        /// </summary>
        private void EnsureRankedActionsUIExists()
        {
            if (panelRoot == null || rankedActionsContainer != null) return;

            rankedActionsContainer = new VisualElement { name = "ranked-actions-container" };
            rankedActionsContainer.AddToClassList("ranked-actions-container");

            var header = new Label("Alternative Actions (Top 3)");
            header.AddToClassList("ranked-actions-header");
            rankedActionsContainer.Add(header);

            // Compare toggle
            compareToggleButton = new Button(OnToggleCompare)
            {
                text = "Compare",
                name = "compare-toggle-btn"
            };
            compareToggleButton.AddToClassList("compare-toggle-btn");
            rankedActionsContainer.Add(compareToggleButton);

            // Comparison table (hidden by default)
            comparisonTable = new VisualElement { name = "comparison-table" };
            comparisonTable.AddToClassList("comparison-table");
            comparisonTable.style.display = DisplayStyle.None;
            rankedActionsContainer.Add(comparisonTable);

            // Do-nothing risk indicator
            var doNothingContainer = new VisualElement { name = "do-nothing-container" };
            doNothingContainer.AddToClassList("do-nothing-container");

            doNothingRiskLabel = new Label("Do Nothing Risk: --");
            doNothingRiskLabel.AddToClassList("do-nothing-label");
            doNothingContainer.Add(doNothingRiskLabel);

            doNothingRiskBar = new VisualElement { name = "do-nothing-risk-bar" };
            doNothingRiskBar.AddToClassList("do-nothing-risk-bar");
            var barFill = new VisualElement { name = "do-nothing-risk-fill" };
            barFill.AddToClassList("do-nothing-risk-fill");
            doNothingRiskBar.Add(barFill);
            doNothingContainer.Add(doNothingRiskBar);

            rankedActionsContainer.Add(doNothingContainer);

            // Insert before the feedback container if it exists, otherwise append
            if (feedbackContainer != null)
            {
                int idx = panelRoot.IndexOf(feedbackContainer);
                if (idx >= 0)
                    panelRoot.Insert(idx, rankedActionsContainer);
                else
                    panelRoot.Add(rankedActionsContainer);
            }
            else
            {
                panelRoot.Add(rankedActionsContainer);
            }

            compareMode = false;
        }

        /// <summary>
        /// Populate the ranked action cards with data from the cognitive layer.
        /// Shows top 3 actions as expandable cards with Apply buttons.
        /// </summary>
        public void DisplayRankedActions(RankedActionResult result)
        {
            if (rankedActionsContainer == null)
                EnsureRankedActionsUIExists();

            if (rankedActionsContainer == null) return;

            // Remove old action cards (keep header, toggle, table, do-nothing)
            var toRemove = new List<VisualElement>();
            foreach (var child in rankedActionsContainer.Children())
            {
                if (child.ClassListContains("ranked-action-card"))
                    toRemove.Add(child);
            }
            foreach (var el in toRemove)
                rankedActionsContainer.Remove(el);

            highlightedActionIndex = result.recommendedIndex;

            // Show top 3
            int count = Mathf.Min(result.rankedActions.Count, 3);
            for (int i = 0; i < count; i++)
            {
                var action = result.rankedActions[i];
                var card = BuildActionCard(action, i, i == result.recommendedIndex);

                // Insert after header (index 1 = after header label)
                int insertIdx = 1 + i;
                if (insertIdx < rankedActionsContainer.childCount)
                    rankedActionsContainer.Insert(insertIdx, card);
                else
                    rankedActionsContainer.Add(card);
            }

            // Update do-nothing risk
            UpdateDoNothingRisk(result.doNothingRisk);

            // Update comparison table
            if (compareMode)
                BuildComparisonTable(result);
        }

        /// <summary>
        /// Build a single expandable action card.
        /// </summary>
        private VisualElement BuildActionCard(RankedActionData action, int rank, bool isRecommended)
        {
            var card = new VisualElement();
            card.AddToClassList("ranked-action-card");
            if (isRecommended)
                card.AddToClassList("ranked-action-recommended");

            // Header row: rank + name + score
            var headerRow = new VisualElement();
            headerRow.style.flexDirection = FlexDirection.Row;
            headerRow.AddToClassList("ranked-action-header-row");

            var rankLabel = new Label($"#{rank + 1}");
            rankLabel.AddToClassList("ranked-action-rank");
            headerRow.Add(rankLabel);

            var nameLabel = new Label(action.description);
            nameLabel.AddToClassList("ranked-action-name");
            headerRow.Add(nameLabel);

            card.Add(headerRow);

            // Key benefit
            string benefit = GetKeyBenefit(action);
            var benefitLabel = new Label($"Benefit: {benefit}");
            benefitLabel.AddToClassList("ranked-action-benefit");
            card.Add(benefitLabel);

            // Key cost
            string cost = GetKeyCost(action);
            var costLabel = new Label($"Cost: {cost}");
            costLabel.AddToClassList("ranked-action-cost");
            card.Add(costLabel);

            // Confidence bar
            var confRow = new VisualElement();
            confRow.style.flexDirection = FlexDirection.Row;
            confRow.AddToClassList("ranked-action-conf-row");

            var confLabel = new Label($"Confidence: {action.confidence:P0}");
            confLabel.AddToClassList("ranked-action-conf-label");
            confRow.Add(confLabel);

            var confBar = new VisualElement();
            confBar.AddToClassList("ranked-action-conf-bar");
            var confFill = new VisualElement();
            confFill.AddToClassList("ranked-action-conf-fill");
            confFill.style.width = new StyleLength(new Length(action.confidence * 100f, LengthUnit.Percent));
            confBar.Add(confFill);
            confRow.Add(confBar);

            card.Add(confRow);

            // Expandable reasoning (collapsed by default)
            var reasoningLabel = new Label(action.reasoning);
            reasoningLabel.AddToClassList("ranked-action-reasoning");
            reasoningLabel.style.display = DisplayStyle.None;
            card.Add(reasoningLabel);

            // Expand/collapse toggle on card click
            card.RegisterCallback<ClickEvent>(evt =>
            {
                var style = reasoningLabel.style.display;
                reasoningLabel.style.display =
                    style == DisplayStyle.None ? DisplayStyle.Flex : DisplayStyle.None;
            });

            // Apply button
            float feedPct = action.feedOverridePct;
            float speedPct = action.speedOverridePct;
            string actionType = action.actionType;
            var applyBtn = new Button(() => OnApplyRankedAction(feedPct, speedPct, actionType))
            {
                text = "Apply"
            };
            applyBtn.AddToClassList("ranked-action-apply-btn");
            card.Add(applyBtn);

            return card;
        }

        /// <summary>Derive the key benefit string for an action card.</summary>
        private static string GetKeyBenefit(RankedActionData action)
        {
            if (action.riskReduction > 0.5f)
                return $"Risk -{action.riskReduction:P0}";
            if (action.forceReductionPct > 10f)
                return $"Force -{action.forceReductionPct:F0}%";
            if (action.rulExtensionMin > 10f)
                return $"Tool life +{action.rulExtensionMin:F0} min";
            if (action.surfaceImprovementPct > 15f)
                return $"Surface +{action.surfaceImprovementPct:F0}%";
            return $"Risk -{action.riskReduction:P0}";
        }

        /// <summary>Derive the key cost string for an action card.</summary>
        private static string GetKeyCost(RankedActionData action)
        {
            if (action.cycleTimeImpactPct > 0f)
                return $"Cycle time +{action.cycleTimeImpactPct:F0}%";
            return "No cycle time impact";
        }

        /// <summary>Apply a ranked action by publishing feed/speed overrides.</summary>
        private void OnApplyRankedAction(float feedPct, float speedPct, string actionType)
        {
            if (miracleBridge == null)
                miracleBridge = MiracleBridge.Instance;

            if (miracleBridge != null)
            {
                string reason = $"Operator selected ranked action: {actionType} (feed={feedPct:F0}%, spindle={speedPct:F0}%)";
                miracleBridge.PublishFeedOverride(feedPct, speedPct, reason);
                Debug.Log($"[DecisionSupportPanel] Applied ranked action {actionType}: feed={feedPct}%, spindle={speedPct}%");

                // Update sliders to reflect
                if (whatIfSlider != null) whatIfSlider.value = feedPct;
                if (spindleSlider != null) spindleSlider.value = speedPct;

                overrideAppliedTime = Time.time;
                awaitingPostActionFeedback = true;
            }
        }

        /// <summary>Toggle between card view and side-by-side comparison table.</summary>
        private void OnToggleCompare()
        {
            compareMode = !compareMode;
            if (comparisonTable != null)
            {
                comparisonTable.style.display = compareMode ? DisplayStyle.Flex : DisplayStyle.None;
            }
            if (compareToggleButton != null)
            {
                compareToggleButton.text = compareMode ? "Cards" : "Compare";
            }
        }

        /// <summary>Build a side-by-side comparison table for the ranked actions.</summary>
        private void BuildComparisonTable(RankedActionResult result)
        {
            if (comparisonTable == null) return;
            comparisonTable.Clear();

            // Header row
            var headerRow = new VisualElement();
            headerRow.style.flexDirection = FlexDirection.Row;
            headerRow.AddToClassList("comparison-header-row");

            headerRow.Add(MakeTableCell("Metric", isHeader: true));
            int count = Mathf.Min(result.rankedActions.Count, 3);
            for (int i = 0; i < count; i++)
            {
                headerRow.Add(MakeTableCell($"#{i + 1} {result.rankedActions[i].actionType}", isHeader: true));
            }
            comparisonTable.Add(headerRow);

            // Metric rows
            string[] metrics = { "Force Reduction", "RUL Extension", "Surface Improvement", "Cycle Time Impact", "Risk Reduction", "Confidence" };
            for (int m = 0; m < metrics.Length; m++)
            {
                var row = new VisualElement();
                row.style.flexDirection = FlexDirection.Row;
                row.AddToClassList("comparison-data-row");
                row.Add(MakeTableCell(metrics[m], isHeader: false));

                for (int i = 0; i < count; i++)
                {
                    var a = result.rankedActions[i];
                    string val = m switch
                    {
                        0 => $"{a.forceReductionPct:F1}%",
                        1 => $"{a.rulExtensionMin:F1} min",
                        2 => $"{a.surfaceImprovementPct:F1}%",
                        3 => $"+{a.cycleTimeImpactPct:F1}%",
                        4 => $"{a.riskReduction:P0}",
                        5 => $"{a.confidence:P0}",
                        _ => "--"
                    };
                    row.Add(MakeTableCell(val, isHeader: false));
                }
                comparisonTable.Add(row);
            }
        }

        /// <summary>Helper to create a table cell label.</summary>
        private static Label MakeTableCell(string text, bool isHeader)
        {
            var label = new Label(text);
            label.AddToClassList(isHeader ? "comparison-cell-header" : "comparison-cell");
            label.style.width = new StyleLength(new Length(25f, LengthUnit.Percent));
            return label;
        }

        // ========================================================================
        // Causal Trajectory Preview — Forward Simulation Visualization
        // ========================================================================

        /// <summary>
        /// A single point on a causal trajectory, capturing predicted metrics
        /// at a specific G-code block index.
        /// </summary>
        [Serializable]
        public class CausalTrajectoryPoint
        {
            public int blockIndex;
            public float forceN;
            public float temperatureC;
            public float wearMM;
            public float surfaceRaUM;
            public float toolLifeMin;
        }

        /// <summary>
        /// Result of a forward causal simulation comparing baseline and modified
        /// trajectories for a specific intervention.
        /// </summary>
        [Serializable]
        public class CausalPreviewResult
        {
            public string interventionType;   // "REDUCE_FEED", "REDUCE_SPEED", etc.
            public float interventionValue;    // e.g. -20 for 20% reduction
            public List<CausalTrajectoryPoint> baselineTrajectory;
            public List<CausalTrajectoryPoint> modifiedTrajectory;
            public float forceChangePct;
            public float toolLifeChangePct;
            public float cycleTimeChangePct;
            public float surfaceQualityChangePct;
            public string reasoning;           // human-readable causal explanation
            public float confidence;
            public List<string> sideEffects;
        }

        // Causal preview UI state
        private VisualElement causalPreviewContainer;
        private VisualElement trajectoryChartContainer;
        private VisualElement impactSummaryContainer;
        private VisualElement confidenceBarContainer;
        private string selectedMetric = "Force";
        private CausalPreviewResult currentCausalPreview;
        private bool causalPreviewPending;

        /// <summary>
        /// Display causal trajectory comparison in the decision support panel.
        /// Called when the forward causal simulation returns a preview result.
        /// </summary>
        public void ShowCausalPreview(CausalPreviewResult preview)
        {
            if (panelRoot == null || preview == null) return;

            currentCausalPreview = preview;

            // Create the causal preview container if it doesn't exist
            EnsureCausalPreviewUIExists();

            if (causalPreviewContainer == null) return;

            // Clear previous content
            causalPreviewContainer.Clear();

            // Section header
            var header = new Label("Causal Trajectory Preview");
            header.AddToClassList("decision-section-title");
            causalPreviewContainer.Add(header);

            // Intervention label
            var interventionLabel = new Label(
                $"Intervention: {preview.interventionType} ({preview.interventionValue:+0;-0;0}%)");
            interventionLabel.AddToClassList("causal-intervention-label");
            causalPreviewContainer.Add(interventionLabel);

            // Trajectory comparison chart
            var chart = BuildTrajectoryComparisonChart(preview);
            causalPreviewContainer.Add(chart);

            // Impact summary cards
            var summary = BuildCausalImpactSummary(preview);
            causalPreviewContainer.Add(summary);

            // Confidence indicator
            var confidence = BuildConfidenceIndicator(preview.confidence);
            causalPreviewContainer.Add(confidence);

            causalPreviewContainer.style.display = DisplayStyle.Flex;
        }

        /// <summary>
        /// Ensure the causal preview container exists in the panel hierarchy.
        /// </summary>
        private void EnsureCausalPreviewUIExists()
        {
            if (causalPreviewContainer != null) return;

            causalPreviewContainer = new VisualElement { name = "causal-preview-container" };
            causalPreviewContainer.AddToClassList("causal-preview-container");
            causalPreviewContainer.style.display = DisplayStyle.None;

            // Insert before ranked actions or feedback, after impact preview
            if (rankedActionsContainer != null)
            {
                int idx = panelRoot.IndexOf(rankedActionsContainer);
                if (idx >= 0)
                    panelRoot.Insert(idx, causalPreviewContainer);
                else
                    panelRoot.Add(causalPreviewContainer);
            }
            else if (feedbackContainer != null)
            {
                int idx = panelRoot.IndexOf(feedbackContainer);
                if (idx >= 0)
                    panelRoot.Insert(idx, causalPreviewContainer);
                else
                    panelRoot.Add(causalPreviewContainer);
            }
            else
            {
                panelRoot.Add(causalPreviewContainer);
            }
        }

        /// <summary>
        /// Build the trajectory comparison chart with metric selector and two overlaid traces.
        /// Baseline is gray dashed, modified is colored solid. Green/red shading marks
        /// blocks where modification improves/worsens the selected metric.
        /// </summary>
        private VisualElement BuildTrajectoryComparisonChart(CausalPreviewResult preview)
        {
            var container = new VisualElement();
            container.AddToClassList("trajectory-chart");

            // Metric selector row
            var selectorRow = new VisualElement();
            selectorRow.style.flexDirection = FlexDirection.Row;
            selectorRow.AddToClassList("trajectory-metric-selector");

            string[] metrics = { "Force", "Temperature", "Wear", "Surface Roughness" };
            foreach (string metric in metrics)
            {
                string m = metric;  // capture for closure
                var btn = new Button(() => OnTrajectoryMetricSelected(m, preview))
                {
                    text = metric
                };
                btn.AddToClassList("trajectory-metric-btn");
                if (metric == selectedMetric)
                    btn.AddToClassList("trajectory-metric-btn-active");
                selectorRow.Add(btn);
            }
            container.Add(selectorRow);

            // Chart area
            trajectoryChartContainer = new VisualElement { name = "trajectory-chart-area" };
            trajectoryChartContainer.AddToClassList("trajectory-chart-area");
            container.Add(trajectoryChartContainer);

            // Populate chart with selected metric
            PopulateTrajectoryChart(preview, selectedMetric);

            // Legend
            var legend = new VisualElement();
            legend.style.flexDirection = FlexDirection.Row;
            legend.AddToClassList("trajectory-legend");

            var baselineLegend = new Label("--- Baseline");
            baselineLegend.AddToClassList("trajectory-baseline");
            legend.Add(baselineLegend);

            var modifiedLegend = new Label("--- Modified");
            modifiedLegend.AddToClassList("trajectory-modified");
            legend.Add(modifiedLegend);

            container.Add(legend);

            return container;
        }

        /// <summary>
        /// Called when operator selects a different metric in the trajectory chart.
        /// </summary>
        private void OnTrajectoryMetricSelected(string metric, CausalPreviewResult preview)
        {
            selectedMetric = metric;
            // Rebuild the chart with new metric
            if (currentCausalPreview != null)
                ShowCausalPreview(currentCausalPreview);
        }

        /// <summary>
        /// Populate the trajectory chart area with block-by-block comparison bars.
        /// Uses vertical bars to represent baseline (gray) and modified (colored) values.
        /// Green shading where modified improves; red where it worsens.
        /// </summary>
        private void PopulateTrajectoryChart(CausalPreviewResult preview, string metric)
        {
            if (trajectoryChartContainer == null) return;
            trajectoryChartContainer.Clear();

            var baseline = preview.baselineTrajectory;
            var modified = preview.modifiedTrajectory;

            if (baseline == null || modified == null || baseline.Count == 0) return;

            int count = Math.Min(baseline.Count, modified.Count);

            // Find max value for Y-axis scaling
            float maxVal = 0.001f;
            for (int i = 0; i < count; i++)
            {
                float bVal = GetMetricValue(baseline[i], metric);
                float mVal = GetMetricValue(modified[i], metric);
                maxVal = Math.Max(maxVal, Math.Max(bVal, mVal));
            }

            // X-axis label
            var xLabel = new Label("Block Index ->");
            xLabel.AddToClassList("trajectory-axis-label");
            trajectoryChartContainer.Add(xLabel);

            // Bar chart row
            var chartRow = new VisualElement();
            chartRow.style.flexDirection = FlexDirection.Row;
            chartRow.AddToClassList("trajectory-bar-row");

            for (int i = 0; i < count; i++)
            {
                float bVal = GetMetricValue(baseline[i], metric);
                float mVal = GetMetricValue(modified[i], metric);

                var blockGroup = new VisualElement();
                blockGroup.AddToClassList("trajectory-block-group");

                // Determine if modification is an improvement
                // For Force, Temperature, Wear, Surface Roughness: lower is better
                bool isImprovement = mVal < bVal;
                blockGroup.AddToClassList(isImprovement ? "impact-positive" : "impact-negative");

                // Baseline bar
                var bBar = new VisualElement();
                bBar.AddToClassList("trajectory-baseline-bar");
                float bPct = (bVal / maxVal) * 100f;
                bBar.style.height = new StyleLength(new Length(bPct, LengthUnit.Percent));
                blockGroup.Add(bBar);

                // Modified bar
                var mBar = new VisualElement();
                mBar.AddToClassList("trajectory-modified-bar");
                float mPct = (mVal / maxVal) * 100f;
                mBar.style.height = new StyleLength(new Length(mPct, LengthUnit.Percent));
                blockGroup.Add(mBar);

                chartRow.Add(blockGroup);
            }

            trajectoryChartContainer.Add(chartRow);
        }

        /// <summary>Extract a named metric value from a trajectory point.</summary>
        internal static float GetMetricValue(CausalTrajectoryPoint point, string metric)
        {
            if (point == null) return 0f;
            return metric switch
            {
                "Force" => point.forceN,
                "Temperature" => point.temperatureC,
                "Wear" => point.wearMM,
                "Surface Roughness" => point.surfaceRaUM,
                _ => point.forceN
            };
        }

        /// <summary>
        /// Build the impact summary cards showing force, tool life, cycle time,
        /// and surface quality changes with direction arrows and color coding.
        /// </summary>
        private VisualElement BuildCausalImpactSummary(CausalPreviewResult preview)
        {
            var container = new VisualElement();
            container.AddToClassList("causal-impact-summary");

            // Cards row
            var cardsRow = new VisualElement();
            cardsRow.style.flexDirection = FlexDirection.Row;
            cardsRow.style.flexWrap = Wrap.Wrap;
            cardsRow.AddToClassList("impact-cards-row");

            cardsRow.Add(BuildImpactCard("Force", preview.forceChangePct, lowerIsBetter: true));
            cardsRow.Add(BuildImpactCard("Tool Life", preview.toolLifeChangePct, lowerIsBetter: false));
            cardsRow.Add(BuildImpactCard("Cycle Time", preview.cycleTimeChangePct, lowerIsBetter: true));
            cardsRow.Add(BuildImpactCard("Surface Quality", preview.surfaceQualityChangePct, lowerIsBetter: true));

            container.Add(cardsRow);

            // Reasoning text
            if (!string.IsNullOrEmpty(preview.reasoning))
            {
                var reasoningLabel = new Label(preview.reasoning);
                reasoningLabel.AddToClassList("causal-reasoning-text");
                container.Add(reasoningLabel);
            }

            // Side effects
            if (preview.sideEffects != null && preview.sideEffects.Count > 0)
            {
                var sideEffectsHeader = new Label("Side Effects:");
                sideEffectsHeader.AddToClassList("causal-side-effects-header");
                container.Add(sideEffectsHeader);

                foreach (string sideEffect in preview.sideEffects)
                {
                    var effectLabel = new Label($"\u26a0 {sideEffect}");
                    effectLabel.AddToClassList("side-effect-warning");
                    container.Add(effectLabel);
                }
            }

            return container;
        }

        /// <summary>Build a single impact card with metric name, change %, arrow, and color.</summary>
        private static VisualElement BuildImpactCard(string metricName, float changePct, bool lowerIsBetter)
        {
            var card = new VisualElement();
            card.AddToClassList("impact-card");

            // Determine direction and whether it's positive or negative
            bool isPositive = lowerIsBetter ? (changePct < 0f) : (changePct > 0f);
            card.AddToClassList(isPositive ? "impact-positive" : "impact-negative");

            // Metric name
            var nameLabel = new Label(metricName);
            nameLabel.AddToClassList("impact-card-name");
            card.Add(nameLabel);

            // Change value with arrow
            string arrow = changePct > 0f ? "\u2191" : changePct < 0f ? "\u2193" : "\u2192";
            var valueLabel = new Label($"{arrow} {changePct:+0.0;-0.0;0.0}%");
            valueLabel.AddToClassList("impact-card-value");
            card.Add(valueLabel);

            return card;
        }

        /// <summary>
        /// Build a horizontal confidence indicator bar with label and color coding.
        /// Green > 0.7, Yellow 0.4-0.7, Red below 0.4.
        /// </summary>
        private VisualElement BuildConfidenceIndicator(float confidence)
        {
            var container = new VisualElement();
            container.AddToClassList("confidence-container");

            // Label
            string level = confidence > 0.7f ? "High" : confidence >= 0.4f ? "Medium" : "Low";
            var label = new Label($"Confidence: {level} ({confidence:P0})");
            label.AddToClassList("confidence-label");
            container.Add(label);

            // Bar track
            var barTrack = new VisualElement();
            barTrack.AddToClassList("confidence-bar");

            var barFill = new VisualElement();
            barFill.AddToClassList("confidence-bar-fill");
            barFill.style.width = new StyleLength(new Length(confidence * 100f, LengthUnit.Percent));

            // Color coding
            if (confidence > 0.7f)
                barFill.style.backgroundColor = new Color(0.2f, 0.85f, 0.4f);
            else if (confidence >= 0.4f)
                barFill.style.backgroundColor = new Color(0.9f, 0.75f, 0.15f);
            else
                barFill.style.backgroundColor = new Color(0.9f, 0.25f, 0.2f);

            barTrack.Add(barFill);
            container.Add(barTrack);

            return container;
        }

        /// <summary>
        /// Request a causal preview from MiracleBridge when what-if sliders change.
        /// Called after the debounced prediction request.
        /// </summary>
        private void RequestCausalPreview(float feedOverridePct, float spindleOverridePct)
        {
            if (miracleBridge == null)
                miracleBridge = MiracleBridge.Instance;

            if (miracleBridge == null) return;

            // Determine intervention type based on which slider changed most
            float feedChange = feedOverridePct - 100f;
            float spindleChange = spindleOverridePct - 100f;

            string interventionType;
            float interventionValue;

            if (Math.Abs(feedChange) >= Math.Abs(spindleChange))
            {
                interventionType = feedChange < 0 ? "REDUCE_FEED" : "INCREASE_FEED";
                interventionValue = feedChange;
            }
            else
            {
                interventionType = spindleChange < 0 ? "REDUCE_SPEED" : "INCREASE_SPEED";
                interventionValue = spindleChange;
            }

            // Generate a local causal preview using analytical models
            // In production, this would call miracleBridge.RequestCausalPreview()
            var preview = GenerateLocalCausalPreview(interventionType, interventionValue,
                feedOverridePct, spindleOverridePct);
            ShowCausalPreview(preview);
        }

        /// <summary>
        /// Generate a local causal trajectory preview using the analytical fallback model.
        /// Produces 10 simulated blocks of baseline vs modified trajectories.
        /// </summary>
        private CausalPreviewResult GenerateLocalCausalPreview(
            string interventionType, float interventionValue,
            float feedOverridePct, float spindleOverridePct)
        {
            float feedRatio = feedOverridePct / 100f;
            float spindleRatio = spindleOverridePct / 100f;

            // Force: proportional to feed^0.8
            float forceMultiplier = Mathf.Pow(feedRatio, 0.8f);
            // Temperature: proportional to speed^0.5
            float tempMultiplier = Mathf.Pow(spindleRatio, 0.5f);
            // Surface roughness: proportional to feed^2
            float surfaceMultiplier = Mathf.Pow(feedRatio, 2f);
            // Tool life: inversely proportional to feed^2
            float toolLifeMultiplier = 1f / Mathf.Max(Mathf.Pow(feedRatio, 2f), 0.01f);

            var baselineTrajectory = new List<CausalTrajectoryPoint>();
            var modifiedTrajectory = new List<CausalTrajectoryPoint>();

            const int numBlocks = 10;
            for (int i = 0; i < numBlocks; i++)
            {
                // Simulate progressive wear over blocks
                float wearProgression = 1f + (i * 0.05f);

                baselineTrajectory.Add(new CausalTrajectoryPoint
                {
                    blockIndex = i,
                    forceN = baselineForceN * wearProgression,
                    temperatureC = 85f * wearProgression,
                    wearMM = 0.1f * wearProgression,
                    surfaceRaUM = baselineRa * wearProgression,
                    toolLifeMin = baselineToolLifeMin * (1f - i * 0.08f)
                });

                modifiedTrajectory.Add(new CausalTrajectoryPoint
                {
                    blockIndex = i,
                    forceN = baselineForceN * forceMultiplier * wearProgression,
                    temperatureC = 85f * tempMultiplier * wearProgression,
                    wearMM = 0.1f * forceMultiplier * wearProgression,
                    surfaceRaUM = baselineRa * surfaceMultiplier * wearProgression,
                    toolLifeMin = baselineToolLifeMin * toolLifeMultiplier * (1f - i * 0.08f)
                });
            }

            float forceChangePct = (forceMultiplier - 1f) * 100f;
            float toolLifeChangePct = (toolLifeMultiplier - 1f) * 100f;
            float cycleTimeChangePct = feedRatio < 1f ? ((1f / feedRatio) - 1f) * 100f : (1f - feedRatio) * 100f;
            float surfaceChangePct = (surfaceMultiplier - 1f) * 100f;

            var sideEffects = new List<string>();
            if (Math.Abs(cycleTimeChangePct) > 5f)
                sideEffects.Add($"Cycle time will {(cycleTimeChangePct > 0 ? "increase" : "decrease")} by {Math.Abs(cycleTimeChangePct):F1}%");
            if (Math.Abs(forceChangePct) > 5f && interventionType.Contains("SPEED"))
                sideEffects.Add($"Cutting force {(forceChangePct > 0 ? "increase" : "decrease")} by {Math.Abs(forceChangePct):F1}%");

            string reasoning = $"Changing {interventionType.Replace("_", " ").ToLowerInvariant()} by {interventionValue:+0;-0;0}% "
                + $"is predicted to {(forceChangePct < 0 ? "reduce" : "increase")} cutting force by {Math.Abs(forceChangePct):F1}% "
                + $"and {(toolLifeChangePct > 0 ? "extend" : "reduce")} tool life by {Math.Abs(toolLifeChangePct):F1}%.";

            return new CausalPreviewResult
            {
                interventionType = interventionType,
                interventionValue = interventionValue,
                baselineTrajectory = baselineTrajectory,
                modifiedTrajectory = modifiedTrajectory,
                forceChangePct = forceChangePct,
                toolLifeChangePct = toolLifeChangePct,
                cycleTimeChangePct = cycleTimeChangePct,
                surfaceQualityChangePct = surfaceChangePct,
                reasoning = reasoning,
                confidence = 0.65f,
                sideEffects = sideEffects
            };
        }

        /// <summary>Update the do-nothing risk indicator bar and label.</summary>
        private void UpdateDoNothingRisk(float risk)
        {
            if (doNothingRiskLabel != null)
            {
                string severity = risk > 0.7f ? "HIGH" : risk > 0.4f ? "MEDIUM" : "LOW";
                doNothingRiskLabel.text = $"Do Nothing Risk: {risk:P0} ({severity})";
            }

            if (doNothingRiskBar != null)
            {
                var fill = doNothingRiskBar.Q<VisualElement>("do-nothing-risk-fill");
                if (fill != null)
                {
                    fill.style.width = new StyleLength(new Length(risk * 100f, LengthUnit.Percent));
                    // Color coding: green < 0.4, yellow 0.4-0.7, red > 0.7
                    if (risk > 0.7f)
                        fill.style.backgroundColor = new Color(0.9f, 0.2f, 0.2f);
                    else if (risk > 0.4f)
                        fill.style.backgroundColor = new Color(0.9f, 0.7f, 0.1f);
                    else
                        fill.style.backgroundColor = new Color(0.2f, 0.8f, 0.3f);
                }
            }
        }
    }
}
