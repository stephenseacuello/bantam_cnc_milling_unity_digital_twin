using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;
using Unity.Profiling;
using MiracleTwin.Core;
using MiracleTwin.Cutting;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Data model for an anomaly annotation displayed on the force chart.
    /// Each annotation marks a predicted anomaly at a specific G-code block.
    /// </summary>
    [System.Serializable]
    public class AnomalyAnnotation
    {
        public int blockIndex;
        public string markerType;       // "FORCE_CRITICAL", "THERMAL_WARNING", etc.
        public float severity;          // 0-1
        public float predictedValue;
        public float threshold;
        public string recommendation;
        public float chartX;            // X position on chart (set during rendering)
    }

    /// <summary>
    /// Multi-axis force breakdown data containing tangential, radial, and axial
    /// force components along with derived torque, power, and specific cutting force.
    /// </summary>
    [System.Serializable]
    public class ForceBreakdown
    {
        public float tangentialN;
        public float radialN;
        public float axialN;
        public float resultantN;
        public float torqueNm;
        public float powerW;
        public float specificCuttingForce;
        public float timestamp;

        /// <summary>
        /// Compute the resultant force magnitude from the three components.
        /// </summary>
        public void ComputeResultant()
        {
            resultantN = Mathf.Sqrt(tangentialN * tangentialN + radialN * radialN + axialN * axialN);
        }
    }

    /// <summary>
    /// Per-axis configuration for force chart visualization, controlling
    /// color, visibility, line thickness, and auto-scale range.
    /// </summary>
    [System.Serializable]
    public class ForceAxisConfig
    {
        public string axisName;
        public Color color;
        public bool isVisible = true;
        public float lineThickness = 2f;
        public float maxValue = 200f;
    }

    /// <summary>
    /// Display modes for the force chart controlling which traces are rendered.
    /// </summary>
    public enum ForceDisplayMode
    {
        Resultant,
        Components,
        Torque,
        Power,
        All
    }

    /// <summary>
    /// Scrolling line chart for Fx, Fy, Fz forces over time.
    /// Uses UI Toolkit custom drawing via generateVisualContent/Painter2D
    /// for efficient rendering of color-coded force traces with grid lines,
    /// axis labels, and a legend. Supports block-level anomaly marker
    /// annotations from the prediction engine.
    /// </summary>
    public class ForceChart : MonoBehaviour
    {
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private GCodeExecutor gcodeExecutor;
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private int maxSamples = 200;
        [SerializeField] private float maxForce = 200f;
        [SerializeField] private float sampleInterval = 0.033f;

        [Header("Appearance")]
        [SerializeField] private float lineWidth = 2f;
        [SerializeField] private int gridLinesHorizontal = 4;
        [SerializeField] private int gridLinesVertical = 5;

        [Header("Force Thresholds")]
        [SerializeField] private float warningForceThreshold = 150f;
        [SerializeField] private float criticalForceThreshold = 180f;
        [SerializeField] private float maxForceCapacity = 200f;

        private static readonly Color FxColor = new(1f, 0.2f, 0.2f, 1f);   // Red
        private static readonly Color FyColor = new(0.2f, 0.8f, 0.2f, 1f); // Green
        private static readonly Color FzColor = new(0.3f, 0.3f, 1f, 1f);   // Blue
        private static readonly Color GridColor = new(0.3f, 0.3f, 0.3f, 0.5f);
        private static readonly Color BackgroundColor = new(0.1f, 0.1f, 0.12f, 0.9f);
        private static readonly Color AxisLabelColor = new(0.7f, 0.7f, 0.7f, 1f);

        // Threshold zone colors (semi-transparent)
        private static readonly Color SafeZoneColor = new(0.1f, 0.6f, 0.1f, 0.12f);
        private static readonly Color WarningZoneColor = new(0.9f, 0.8f, 0.0f, 0.15f);
        private static readonly Color CriticalZoneColor = new(0.9f, 0.15f, 0.1f, 0.18f);
        private static readonly Color WarningLineColor = new(0.95f, 0.85f, 0.1f, 0.9f);
        private static readonly Color CriticalLineColor = new(1f, 0.2f, 0.15f, 0.9f);

        // Anomaly annotation colors by severity
        private static readonly Color SeverityLowColor = new(0.95f, 0.9f, 0.2f, 0.9f);       // Yellow
        private static readonly Color SeverityMediumColor = new(1f, 0.6f, 0.15f, 0.9f);       // Orange
        private static readonly Color SeverityHighColor = new(1f, 0.15f, 0.1f, 0.9f);         // Red

        // Multi-axis force breakdown trace colors
        private static readonly Color TangentialColor = new(0.3f, 0.5f, 1f, 1f);    // Blue
        private static readonly Color RadialColor = new(0.2f, 0.85f, 0.3f, 1f);     // Green
        private static readonly Color AxialColor = new(1f, 0.6f, 0.15f, 1f);        // Orange
        private static readonly Color ResultantColor = new(1f, 1f, 1f, 1f);          // White
        private static readonly Color TorqueColor = new(1f, 0.95f, 0.2f, 1f);       // Yellow
        private static readonly Color PowerColor = new(1f, 0.2f, 1f, 1f);           // Magenta
        private static readonly Color StatsOverlayColor = new(0.6f, 0.8f, 1f, 0.5f);
        private static readonly Color PolarRefColor = new(0.5f, 0.5f, 0.5f, 0.4f);

#if DEVELOPMENT_BUILD || UNITY_EDITOR
        private static readonly ProfilerMarker s_GenerateVisualContentMarker = new("ForceChart.OnGenerateVisualContent");
#endif

        private readonly Queue<Vector3> forceSamples = new();

        // Predicted force data from lookahead
        private float[] predictedForces = Array.Empty<float>();
        private float lastSampleTime;
        private VisualElement chartElement;

        // Threshold exceedance tracking
        private int warningExceedanceCount;
        private int criticalExceedanceCount;
        private bool wasInWarningZone;
        private bool wasInCriticalZone;

        // Anomaly annotations from prediction engine
        private List<AnomalyAnnotation> _anomalyAnnotations = new();
        private int _maxAnnotations = 20;
        private bool _showAnnotations = true;

        // Multi-axis force breakdown data
        private readonly Queue<ForceBreakdown> _forceBreakdowns = new();
        private ForceDisplayMode _displayMode = ForceDisplayMode.Resultant;
        private const int RollingAverageWindow = 10;
        private const float AutoScaleHeadroom = 1.1f; // 10% headroom

        [Header("Force Axis Configuration")]
        [SerializeField] private ForceAxisConfig tangentialAxis = new()
            { axisName = "Tangential", color = new Color(0.3f, 0.5f, 1f, 1f), isVisible = true, lineThickness = 2f, maxValue = 200f };
        [SerializeField] private ForceAxisConfig radialAxis = new()
            { axisName = "Radial", color = new Color(0.2f, 0.85f, 0.3f, 1f), isVisible = true, lineThickness = 2f, maxValue = 200f };
        [SerializeField] private ForceAxisConfig axialAxis = new()
            { axisName = "Axial", color = new Color(1f, 0.6f, 0.15f, 1f), isVisible = true, lineThickness = 2f, maxValue = 200f };
        [SerializeField] private ForceAxisConfig resultantAxis = new()
            { axisName = "Resultant", color = new Color(1f, 1f, 1f, 1f), isVisible = true, lineThickness = 3f, maxValue = 300f };
        [SerializeField] private ForceAxisConfig torqueAxis = new()
            { axisName = "Torque", color = new Color(1f, 0.95f, 0.2f, 1f), isVisible = true, lineThickness = 2f, maxValue = 50f };
        [SerializeField] private ForceAxisConfig powerAxis = new()
            { axisName = "Power", color = new Color(1f, 0.2f, 1f, 1f), isVisible = true, lineThickness = 2f, maxValue = 1000f };

        [Header("Polar Plot")]
        [SerializeField] private float maxAllowableForce = 200f;

        void OnEnable()
        {
            if (cuttingStateEvent != null)
                cuttingStateEvent.Register(OnCuttingState);
            if (gcodeExecutor != null)
                gcodeExecutor.OnLookaheadUpdated += OnLookaheadResults;
        }

        void OnDisable()
        {
            if (cuttingStateEvent != null)
                cuttingStateEvent.Unregister(OnCuttingState);
            if (gcodeExecutor != null)
                gcodeExecutor.OnLookaheadUpdated -= OnLookaheadResults;
        }

        void Start()
        {
            if (uiDocument == null) return;
            var root = uiDocument.rootVisualElement;

            // Find or create the chart visual element
            chartElement = root.Q<VisualElement>("force-chart");
            if (chartElement == null)
            {
                // If no element named "force-chart" exists, look for a generic chart container
                chartElement = root.Q<VisualElement>("chart-container");
            }

            if (chartElement != null)
            {
                chartElement.generateVisualContent += OnGenerateVisualContent;
            }
        }

        void OnDestroy()
        {
            if (chartElement != null)
                chartElement.generateVisualContent -= OnGenerateVisualContent;
        }

        private void OnCuttingState(CuttingStateData state)
        {
            if (Time.time - lastSampleTime < sampleInterval) return;
            lastSampleTime = Time.time;

            forceSamples.Enqueue(new Vector3(state.forceFx, state.forceFy, state.forceFz));
            while (forceSamples.Count > maxSamples)
                forceSamples.Dequeue();
        }

        private void OnLookaheadResults(IReadOnlyList<LookaheadResult> results)
        {
            if (results == null || results.Count == 0)
            {
                ClearPredictions();
                return;
            }
            var forces = new float[results.Count];
            for (int i = 0; i < results.Count; i++)
                forces[i] = results[i].peakForceN;
            SetPredictedForces(forces);
        }

        void Update()
        {
            // Mark for repaint every frame so the chart scrolls smoothly
            chartElement?.MarkDirtyRepaint();
        }

        /// <summary>
        /// Main rendering callback using Painter2D for the force chart.
        /// Draws background, grid, axis labels, force traces, and legend.
        /// </summary>
        private void OnGenerateVisualContent(MeshGenerationContext ctx)
        {
            var painter = ctx.painter2D;
            Rect rect = chartElement.contentRect;
            if (rect.width < 10 || rect.height < 10) return;

#if DEVELOPMENT_BUILD || UNITY_EDITOR
            using (s_GenerateVisualContentMarker.Auto())
            {
#endif

            float padding = 40f;     // Left padding for Y-axis labels
            float rightPad = 10f;
            float topPad = 25f;      // Top padding for legend
            float bottomPad = 20f;   // Bottom padding for X-axis labels

            float chartLeft = padding;
            float chartTop = topPad;
            float chartWidth = rect.width - padding - rightPad;
            float chartHeight = rect.height - topPad - bottomPad;

            if (chartWidth < 10 || chartHeight < 10) return;

            // 1. Draw background
            painter.fillColor = BackgroundColor;
            painter.BeginPath();
            painter.MoveTo(new Vector2(0, 0));
            painter.LineTo(new Vector2(rect.width, 0));
            painter.LineTo(new Vector2(rect.width, rect.height));
            painter.LineTo(new Vector2(0, rect.height));
            painter.ClosePath();
            painter.Fill();

            // 1b. Draw force threshold zone bands
            DrawThresholdBands(painter, chartLeft, chartTop, chartWidth, chartHeight);

            // 2. Draw grid lines
            painter.strokeColor = GridColor;
            painter.lineWidth = 1f;

            // Horizontal grid lines (force values)
            for (int i = 0; i <= gridLinesHorizontal; i++)
            {
                float y = chartTop + (chartHeight / gridLinesHorizontal) * i;
                painter.BeginPath();
                painter.MoveTo(new Vector2(chartLeft, y));
                painter.LineTo(new Vector2(chartLeft + chartWidth, y));
                painter.Stroke();
            }

            // Vertical grid lines (time divisions)
            for (int i = 0; i <= gridLinesVertical; i++)
            {
                float x = chartLeft + (chartWidth / gridLinesVertical) * i;
                painter.BeginPath();
                painter.MoveTo(new Vector2(x, chartTop));
                painter.LineTo(new Vector2(x, chartTop + chartHeight));
                painter.Stroke();
            }

            // 3. Draw force traces
            var samples = forceSamples.ToArray();
            if (samples.Length > 1)
            {
                DrawTrace(painter, samples, 0, FxColor, chartLeft, chartTop, chartWidth, chartHeight); // Fx
                DrawTrace(painter, samples, 1, FyColor, chartLeft, chartTop, chartWidth, chartHeight); // Fy
                DrawTrace(painter, samples, 2, FzColor, chartLeft, chartTop, chartWidth, chartHeight); // Fz
            }

            // 3a-2. Draw multi-axis force breakdown traces (if data available)
            if (_forceBreakdowns.Count > 1)
            {
                DrawForceBreakdownTraces(painter, chartLeft, chartTop, chartWidth, chartHeight);

                // Draw statistics overlay when in Components or All mode
                if (_displayMode == ForceDisplayMode.Components || _displayMode == ForceDisplayMode.All)
                {
                    DrawForceStatistics(painter, chartLeft, chartTop, chartWidth, chartHeight);
                }
            }

            // 3b. Draw predicted force trace (dashed appearance via short segments)
            if (predictedForces.Length > 1)
            {
                DrawPredictedTrace(painter, predictedForces, new Color(1f, 0.8f, 0.2f, 0.6f),
                    chartLeft, chartTop, chartWidth, chartHeight);
            }

            // 3c. Draw threshold lines with pulse effect when exceeded
            DrawThresholdLines(painter, samples, chartLeft, chartTop, chartWidth, chartHeight);

            // 3d. Track exceedance zone transitions
            TrackExceedance(samples);

            // 3e. Draw anomaly annotation markers
            if (_showAnnotations && _anomalyAnnotations.Count > 0)
            {
                var chartArea = new Rect(chartLeft, chartTop, chartWidth, chartHeight);
                DrawAnomalyMarkers(painter, chartArea);
            }

            // 4. Draw axis border
            painter.strokeColor = new Color(0.5f, 0.5f, 0.5f, 0.8f);
            painter.lineWidth = 1.5f;
            painter.BeginPath();
            painter.MoveTo(new Vector2(chartLeft, chartTop));
            painter.LineTo(new Vector2(chartLeft, chartTop + chartHeight));
            painter.LineTo(new Vector2(chartLeft + chartWidth, chartTop + chartHeight));
            painter.Stroke();

            // Note: Axis labels and legend text are drawn via Label elements
            // added in Start(). Painter2D does not support text rendering directly.
            // The labels below use small colored rectangles as legend keys.

            // 5. Draw legend color keys (small colored rectangles)
            float legendY = 5f;
            float legendX = chartLeft;
            float keySize = 10f;
            float keySpacing = 70f;

            // Fx legend key
            painter.fillColor = FxColor;
            painter.BeginPath();
            painter.MoveTo(new Vector2(legendX, legendY));
            painter.LineTo(new Vector2(legendX + keySize, legendY));
            painter.LineTo(new Vector2(legendX + keySize, legendY + keySize));
            painter.LineTo(new Vector2(legendX, legendY + keySize));
            painter.ClosePath();
            painter.Fill();

            // Fy legend key
            painter.fillColor = FyColor;
            painter.BeginPath();
            painter.MoveTo(new Vector2(legendX + keySpacing, legendY));
            painter.LineTo(new Vector2(legendX + keySpacing + keySize, legendY));
            painter.LineTo(new Vector2(legendX + keySpacing + keySize, legendY + keySize));
            painter.LineTo(new Vector2(legendX + keySpacing, legendY + keySize));
            painter.ClosePath();
            painter.Fill();

            // Fz legend key
            painter.fillColor = FzColor;
            painter.BeginPath();
            painter.MoveTo(new Vector2(legendX + keySpacing * 2, legendY));
            painter.LineTo(new Vector2(legendX + keySpacing * 2 + keySize, legendY));
            painter.LineTo(new Vector2(legendX + keySpacing * 2 + keySize, legendY + keySize));
            painter.LineTo(new Vector2(legendX + keySpacing * 2, legendY + keySize));
            painter.ClosePath();
            painter.Fill();

            // Predicted legend key
            painter.fillColor = new Color(1f, 0.8f, 0.2f, 0.6f);
            painter.BeginPath();
            painter.MoveTo(new Vector2(legendX + keySpacing * 3, legendY));
            painter.LineTo(new Vector2(legendX + keySpacing * 3 + keySize, legendY));
            painter.LineTo(new Vector2(legendX + keySpacing * 3 + keySize, legendY + keySize));
            painter.LineTo(new Vector2(legendX + keySpacing * 3, legendY + keySize));
            painter.ClosePath();
            painter.Fill();

            // 6. Draw anomaly annotation count badge in chart header
            if (_showAnnotations && _anomalyAnnotations.Count > 0)
            {
                float badgeX = legendX + keySpacing * 4 + 10f;
                float badgeW = 20f;
                float badgeH = 14f;
                float badgeY = legendY - 1f;
                float maxSev = 0f;
                foreach (var ann in _anomalyAnnotations)
                    if (ann.severity > maxSev) maxSev = ann.severity;
                Color badgeColor = GetSeverityColor(maxSev);
                // Badge background
                DrawRect(painter, new Color(badgeColor.r, badgeColor.g, badgeColor.b, 0.3f),
                    badgeX, badgeY, badgeW, badgeH);
                // Badge border
                painter.strokeColor = badgeColor;
                painter.lineWidth = 1f;
                painter.BeginPath();
                painter.MoveTo(new Vector2(badgeX, badgeY));
                painter.LineTo(new Vector2(badgeX + badgeW, badgeY));
                painter.LineTo(new Vector2(badgeX + badgeW, badgeY + badgeH));
                painter.LineTo(new Vector2(badgeX, badgeY + badgeH));
                painter.ClosePath();
                painter.Stroke();
                // Badge count indicator (small filled circle, since Painter2D can't render text)
                float dotR = 3f;
                float dotCX = badgeX + badgeW * 0.5f;
                float dotCY = badgeY + badgeH * 0.5f;
                painter.fillColor = badgeColor;
                painter.BeginPath();
                painter.Arc(new Vector2(dotCX, dotCY), dotR, 0f, 360f);
                painter.ClosePath();
                painter.Fill();
            }

#if DEVELOPMENT_BUILD || UNITY_EDITOR
            }
#endif
        }

        /// <summary>
        /// Draw a single force trace line using Painter2D.
        /// </summary>
        /// <param name="painter">The Painter2D instance</param>
        /// <param name="samples">Array of Vector3 force samples</param>
        /// <param name="component">0=X, 1=Y, 2=Z</param>
        /// <param name="color">Line color</param>
        /// <param name="left">Chart area left edge</param>
        /// <param name="top">Chart area top edge</param>
        /// <param name="width">Chart area width</param>
        /// <param name="height">Chart area height</param>
        private void DrawTrace(Painter2D painter, Vector3[] samples, int component, Color color,
            float left, float top, float width, float height)
        {
            if (samples.Length < 2) return;

            painter.strokeColor = color;
            painter.lineWidth = lineWidth;
            painter.lineCap = LineCap.Round;
            painter.lineJoin = LineJoin.Round;

            painter.BeginPath();

            for (int i = 0; i < samples.Length; i++)
            {
                float x = left + (i / (float)(maxSamples - 1)) * width;

                float value = component switch
                {
                    0 => samples[i].x,
                    1 => samples[i].y,
                    2 => samples[i].z,
                    _ => 0f
                };

                // Map force value to Y position (0 at bottom, maxForce at top)
                // Support negative forces: center line at middle, +/- maxForce at edges
                float normalized = Mathf.Clamp(value / maxForce, -1f, 1f);
                float y = top + height * 0.5f - normalized * (height * 0.5f);

                if (i == 0)
                    painter.MoveTo(new Vector2(x, y));
                else
                    painter.LineTo(new Vector2(x, y));
            }

            painter.Stroke();
        }

        /// <summary>
        /// Draw predicted force trace with dashed line style.
        /// </summary>
        private void DrawPredictedTrace(Painter2D painter, float[] values, Color color,
            float left, float top, float width, float height)
        {
            if (values.Length < 2) return;

            painter.strokeColor = color;
            painter.lineWidth = lineWidth * 0.8f;
            painter.lineCap = LineCap.Butt;

            // Draw as short dashed segments (every other pair of points)
            for (int i = 0; i < values.Length - 1; i += 2)
            {
                float x0 = left + width * 0.5f + (i / (float)(values.Length - 1)) * (width * 0.5f);
                float x1 = left + width * 0.5f + ((i + 1) / (float)(values.Length - 1)) * (width * 0.5f);

                float normalized0 = Mathf.Clamp(values[i] / maxForce, -1f, 1f);
                float normalized1 = Mathf.Clamp(values[i + 1] / maxForce, -1f, 1f);

                float y0 = top + height * 0.5f - normalized0 * (height * 0.5f);
                float y1 = top + height * 0.5f - normalized1 * (height * 0.5f);

                painter.BeginPath();
                painter.MoveTo(new Vector2(x0, y0));
                painter.LineTo(new Vector2(x1, y1));
                painter.Stroke();
            }
        }

        /// <summary>
        /// Set predicted force values from lookahead analysis for overlay rendering.
        /// </summary>
        public void SetPredictedForces(float[] forces)
        {
            predictedForces = forces ?? Array.Empty<float>();
        }

        public void ClearPredictions()
        {
            predictedForces = Array.Empty<float>();
        }

        /// <summary>
        /// Draw semi-transparent threshold zone bands (green/yellow/red).
        /// </summary>
        private void DrawThresholdBands(Painter2D painter, float left, float top, float width, float height)
        {
            float chartBottom = top + height;
            float center = top + height * 0.5f;

            // Helper: map a force value to Y pixel (positive side only, mirrored for negative)
            float WarningY(bool positive) => positive
                ? center - (warningForceThreshold / maxForce) * (height * 0.5f)
                : center + (warningForceThreshold / maxForce) * (height * 0.5f);
            float CriticalY(bool positive) => positive
                ? center - (criticalForceThreshold / maxForce) * (height * 0.5f)
                : center + (criticalForceThreshold / maxForce) * (height * 0.5f);

            // --- Positive half ---
            // Green zone: center to warning threshold
            DrawRect(painter, SafeZoneColor, left, WarningY(true), width, center - WarningY(true));
            // Yellow zone: warning to critical
            DrawRect(painter, WarningZoneColor, left, CriticalY(true), width, WarningY(true) - CriticalY(true));
            // Red zone: critical to top
            DrawRect(painter, CriticalZoneColor, left, top, width, CriticalY(true) - top);

            // --- Negative half (mirrored) ---
            DrawRect(painter, SafeZoneColor, left, center, width, WarningY(false) - center);
            DrawRect(painter, WarningZoneColor, left, WarningY(false), width, CriticalY(false) - WarningY(false));
            DrawRect(painter, CriticalZoneColor, left, CriticalY(false), width, chartBottom - CriticalY(false));
        }

        /// <summary>
        /// Draw threshold lines with pulse effect when any trace crosses into the zone.
        /// </summary>
        private void DrawThresholdLines(Painter2D painter, Vector3[] samples,
            float left, float top, float width, float height)
        {
            float center = top + height * 0.5f;
            float warningYPos = center - (warningForceThreshold / maxForce) * (height * 0.5f);
            float warningYNeg = center + (warningForceThreshold / maxForce) * (height * 0.5f);
            float criticalYPos = center - (criticalForceThreshold / maxForce) * (height * 0.5f);
            float criticalYNeg = center + (criticalForceThreshold / maxForce) * (height * 0.5f);

            // Determine if any current sample exceeds thresholds for pulse effect
            bool inWarning = false;
            bool inCritical = false;
            if (samples != null && samples.Length > 0)
            {
                var latest = samples[samples.Length - 1];
                float maxAbs = Mathf.Max(Mathf.Abs(latest.x), Mathf.Max(Mathf.Abs(latest.y), Mathf.Abs(latest.z)));
                inWarning = maxAbs >= warningForceThreshold;
                inCritical = maxAbs >= criticalForceThreshold;
            }

            // Warning threshold lines (pulse when exceeded)
            float warningAlpha = inWarning ? 0.6f + 0.4f * Mathf.Abs(Mathf.Sin(Time.time * 4f)) : 0.5f;
            float warningWidth = inWarning ? 2.5f : 1.5f;
            Color warningColor = new(WarningLineColor.r, WarningLineColor.g, WarningLineColor.b, warningAlpha);

            painter.strokeColor = warningColor;
            painter.lineWidth = warningWidth;
            painter.BeginPath();
            painter.MoveTo(new Vector2(left, warningYPos));
            painter.LineTo(new Vector2(left + width, warningYPos));
            painter.Stroke();
            painter.BeginPath();
            painter.MoveTo(new Vector2(left, warningYNeg));
            painter.LineTo(new Vector2(left + width, warningYNeg));
            painter.Stroke();

            // Critical threshold lines (pulse when exceeded)
            float criticalAlpha = inCritical ? 0.7f + 0.3f * Mathf.Abs(Mathf.Sin(Time.time * 6f)) : 0.5f;
            float criticalWidth = inCritical ? 3f : 1.5f;
            Color criticalColor = new(CriticalLineColor.r, CriticalLineColor.g, CriticalLineColor.b, criticalAlpha);

            painter.strokeColor = criticalColor;
            painter.lineWidth = criticalWidth;
            painter.BeginPath();
            painter.MoveTo(new Vector2(left, criticalYPos));
            painter.LineTo(new Vector2(left + width, criticalYPos));
            painter.Stroke();
            painter.BeginPath();
            painter.MoveTo(new Vector2(left, criticalYNeg));
            painter.LineTo(new Vector2(left + width, criticalYNeg));
            painter.Stroke();

            // Draw threshold labels on the right side
            // Note: Painter2D does not support text; labels are drawn as small
            // colored indicator rectangles on the right edge of the chart.
            float labelW = 6f;
            float labelH = 4f;
            float labelX = left + width + 2f;

            painter.fillColor = warningColor;
            DrawRect(painter, warningColor, labelX, warningYPos - labelH * 0.5f, labelW, labelH);
            DrawRect(painter, warningColor, labelX, warningYNeg - labelH * 0.5f, labelW, labelH);

            painter.fillColor = criticalColor;
            DrawRect(painter, criticalColor, labelX, criticalYPos - labelH * 0.5f, labelW, labelH);
            DrawRect(painter, criticalColor, labelX, criticalYNeg - labelH * 0.5f, labelW, labelH);
        }

        /// <summary>
        /// Track zone transitions and increment exceedance counters.
        /// </summary>
        private void TrackExceedance(Vector3[] samples)
        {
            if (samples == null || samples.Length == 0) return;

            var latest = samples[samples.Length - 1];
            float maxAbs = Mathf.Max(Mathf.Abs(latest.x), Mathf.Max(Mathf.Abs(latest.y), Mathf.Abs(latest.z)));

            bool inWarning = maxAbs >= warningForceThreshold;
            bool inCritical = maxAbs >= criticalForceThreshold;

            if (inWarning && !wasInWarningZone)
                warningExceedanceCount++;
            if (inCritical && !wasInCriticalZone)
                criticalExceedanceCount++;

            wasInWarningZone = inWarning;
            wasInCriticalZone = inCritical;
        }

        /// <summary>
        /// Draw a filled rectangle using Painter2D.
        /// </summary>
        private static void DrawRect(Painter2D painter, Color color, float x, float y, float w, float h)
        {
            painter.fillColor = color;
            painter.BeginPath();
            painter.MoveTo(new Vector2(x, y));
            painter.LineTo(new Vector2(x + w, y));
            painter.LineTo(new Vector2(x + w, y + h));
            painter.LineTo(new Vector2(x, y + h));
            painter.ClosePath();
            painter.Fill();
        }

        /// <summary>Number of times any force trace transitioned into the warning zone.</summary>
        public int WarningExceedanceCount => warningExceedanceCount;

        /// <summary>Number of times any force trace transitioned into the critical zone.</summary>
        public int CriticalExceedanceCount => criticalExceedanceCount;

        public Vector3[] GetSamples()
        {
            var arr = new Vector3[forceSamples.Count];
            forceSamples.CopyTo(arr, 0);
            return arr;
        }

        public void Clear()
        {
            forceSamples.Clear();
        }

        // ── Anomaly Annotation API ──────────────────────────────────────

        /// <summary>
        /// Add anomaly markers from prediction engine. Caps at _maxAnnotations,
        /// keeping only the highest-severity annotations when over the limit.
        /// </summary>
        public void SetAnomalyAnnotations(List<AnomalyAnnotation> annotations)
        {
            if (annotations == null)
            {
                _anomalyAnnotations.Clear();
                return;
            }

            if (annotations.Count > _maxAnnotations)
            {
                // Keep only the top N by severity (descending)
                _anomalyAnnotations = annotations
                    .OrderByDescending(a => a.severity)
                    .Take(_maxAnnotations)
                    .ToList();
            }
            else
            {
                _anomalyAnnotations = new List<AnomalyAnnotation>(annotations);
            }
        }

        /// <summary>Toggle annotation visibility on the chart.</summary>
        public void ToggleAnnotations(bool show)
        {
            _showAnnotations = show;
        }

        /// <summary>Current anomaly annotations (read-only access for testing).</summary>
        public IReadOnlyList<AnomalyAnnotation> AnomalyAnnotations => _anomalyAnnotations;

        /// <summary>Whether annotations are currently visible.</summary>
        public bool ShowAnnotations => _showAnnotations;

        /// <summary>Maximum number of annotations allowed.</summary>
        public int MaxAnnotations => _maxAnnotations;

        /// <summary>
        /// Draw anomaly markers on the chart. For each annotation, draws:
        /// - A vertical dashed line at the predicted block position
        /// - A small marker symbol at the top of the line
        /// - Color coded by severity (yellow/orange/red)
        /// </summary>
        private void DrawAnomalyMarkers(Painter2D painter, Rect chartArea)
        {
            if (_anomalyAnnotations == null || _anomalyAnnotations.Count == 0) return;

            foreach (var annotation in _anomalyAnnotations)
            {
                // Map blockIndex to X position within chart area
                float normalizedX = maxSamples > 1
                    ? Mathf.Clamp01(annotation.blockIndex / (float)(maxSamples - 1))
                    : 0.5f;
                float x = chartArea.x + normalizedX * chartArea.width;
                annotation.chartX = x;

                Color markerColor = GetSeverityColor(annotation.severity);

                // Draw vertical dashed line
                painter.strokeColor = new Color(markerColor.r, markerColor.g, markerColor.b, 0.6f);
                painter.lineWidth = 1.5f;
                painter.lineCap = LineCap.Butt;

                float dashLength = 6f;
                float gapLength = 4f;
                float yPos = chartArea.y;
                while (yPos < chartArea.y + chartArea.height)
                {
                    float dashEnd = Mathf.Min(yPos + dashLength, chartArea.y + chartArea.height);
                    painter.BeginPath();
                    painter.MoveTo(new Vector2(x, yPos));
                    painter.LineTo(new Vector2(x, dashEnd));
                    painter.Stroke();
                    yPos = dashEnd + gapLength;
                }

                // Draw marker symbol at top of the line
                float symbolSize = 8f;
                float symbolY = chartArea.y - symbolSize - 2f;
                painter.fillColor = markerColor;

                string symbol = GetMarkerSymbol(annotation.markerType);
                switch (symbol)
                {
                    case "\u25B2": // ▲ FORCE - triangle pointing up
                        painter.BeginPath();
                        painter.MoveTo(new Vector2(x, symbolY));
                        painter.LineTo(new Vector2(x + symbolSize * 0.5f, symbolY + symbolSize));
                        painter.LineTo(new Vector2(x - symbolSize * 0.5f, symbolY + symbolSize));
                        painter.ClosePath();
                        painter.Fill();
                        break;

                    case "\u25CF": // ● THERMAL - filled circle
                        painter.BeginPath();
                        painter.Arc(new Vector2(x, symbolY + symbolSize * 0.5f), symbolSize * 0.4f, 0f, 360f);
                        painter.ClosePath();
                        painter.Fill();
                        break;

                    case "\u25C6": // ◆ WEAR - diamond
                        float half = symbolSize * 0.5f;
                        float cy = symbolY + half;
                        painter.BeginPath();
                        painter.MoveTo(new Vector2(x, cy - half));
                        painter.LineTo(new Vector2(x + half, cy));
                        painter.LineTo(new Vector2(x, cy + half));
                        painter.LineTo(new Vector2(x - half, cy));
                        painter.ClosePath();
                        painter.Fill();
                        break;

                    case "\u2605": // ★ CHATTER - 4-pointed star approximated as rotated square
                        float starR = symbolSize * 0.45f;
                        float starCY = symbolY + symbolSize * 0.5f;
                        painter.BeginPath();
                        painter.MoveTo(new Vector2(x, starCY - starR));
                        painter.LineTo(new Vector2(x + starR * 0.35f, starCY - starR * 0.35f));
                        painter.LineTo(new Vector2(x + starR, starCY));
                        painter.LineTo(new Vector2(x + starR * 0.35f, starCY + starR * 0.35f));
                        painter.MoveTo(new Vector2(x, starCY + starR));
                        painter.LineTo(new Vector2(x - starR * 0.35f, starCY + starR * 0.35f));
                        painter.LineTo(new Vector2(x - starR, starCY));
                        painter.LineTo(new Vector2(x - starR * 0.35f, starCY - starR * 0.35f));
                        painter.ClosePath();
                        painter.Fill();
                        break;

                    case "\u25A0": // ■ SURFACE - filled square
                        float sqHalf = symbolSize * 0.4f;
                        float sqCY = symbolY + symbolSize * 0.5f;
                        DrawRect(painter, markerColor, x - sqHalf, sqCY - sqHalf, sqHalf * 2f, sqHalf * 2f);
                        break;

                    default: // ⚠ TOOL or unknown - triangle with exclamation (just triangle outline)
                        painter.strokeColor = markerColor;
                        painter.lineWidth = 2f;
                        painter.BeginPath();
                        painter.MoveTo(new Vector2(x, symbolY));
                        painter.LineTo(new Vector2(x + symbolSize * 0.5f, symbolY + symbolSize));
                        painter.LineTo(new Vector2(x - symbolSize * 0.5f, symbolY + symbolSize));
                        painter.ClosePath();
                        painter.Stroke();
                        break;
                }
            }
        }

        /// <summary>
        /// Draw annotation tooltip when hovering near a marker. Shows a
        /// background rect with marker type, predicted value vs threshold,
        /// and the recommended action.
        /// </summary>
        private void DrawAnnotationTooltip(Painter2D painter, AnomalyAnnotation annotation, Rect chartArea)
        {
            if (annotation == null) return;

            float tooltipW = 160f;
            float tooltipH = 48f;
            float tooltipX = Mathf.Clamp(annotation.chartX - tooltipW * 0.5f,
                chartArea.x, chartArea.x + chartArea.width - tooltipW);
            float tooltipY = chartArea.y - tooltipH - 14f;

            // Background rect with rounded corners (approximated as rect + border)
            Color bgColor = new(0.12f, 0.12f, 0.18f, 0.92f);
            DrawRect(painter, bgColor, tooltipX, tooltipY, tooltipW, tooltipH);

            // Border
            Color borderColor = GetSeverityColor(annotation.severity);
            painter.strokeColor = new Color(borderColor.r, borderColor.g, borderColor.b, 0.7f);
            painter.lineWidth = 1.5f;
            painter.BeginPath();
            painter.MoveTo(new Vector2(tooltipX, tooltipY));
            painter.LineTo(new Vector2(tooltipX + tooltipW, tooltipY));
            painter.LineTo(new Vector2(tooltipX + tooltipW, tooltipY + tooltipH));
            painter.LineTo(new Vector2(tooltipX, tooltipY + tooltipH));
            painter.ClosePath();
            painter.Stroke();

            // Severity indicator bar on the left side
            float barW = 3f;
            DrawRect(painter, borderColor, tooltipX, tooltipY, barW, tooltipH);

            // Small colored dot to indicate the marker type (text cannot be drawn by Painter2D)
            float dotR = 3f;
            painter.fillColor = borderColor;
            painter.BeginPath();
            painter.Arc(new Vector2(tooltipX + 12f, tooltipY + 10f), dotR, 0f, 360f);
            painter.ClosePath();
            painter.Fill();

            // Value indicator bar (predicted vs threshold)
            float barAreaX = tooltipX + 8f;
            float barAreaY = tooltipY + 22f;
            float barAreaW = tooltipW - 16f;
            float barH = 6f;
            // Background bar (threshold)
            DrawRect(painter, new Color(0.3f, 0.3f, 0.3f, 0.5f), barAreaX, barAreaY, barAreaW, barH);
            // Fill bar (predicted value relative to threshold)
            float fillRatio = annotation.threshold > 0f
                ? Mathf.Clamp01(annotation.predictedValue / annotation.threshold)
                : 1f;
            DrawRect(painter, borderColor, barAreaX, barAreaY, barAreaW * fillRatio, barH);

            // Recommendation indicator (small colored rectangle)
            if (!string.IsNullOrEmpty(annotation.recommendation))
            {
                float recY = tooltipY + 34f;
                DrawRect(painter, new Color(0.5f, 0.7f, 1f, 0.4f), barAreaX, recY, barAreaW, 8f);
            }
        }

        /// <summary>
        /// Get the marker symbol for each anomaly type.
        /// FORCE -> triangle, THERMAL -> circle, WEAR -> diamond,
        /// CHATTER -> star, SURFACE -> square, TOOL -> warning triangle.
        /// </summary>
        public static string GetMarkerSymbol(string markerType)
        {
            if (string.IsNullOrEmpty(markerType)) return "\u26A0"; // ⚠

            // Extract the primary category from marker types like "FORCE_CRITICAL"
            string upper = markerType.ToUpperInvariant();

            if (upper.StartsWith("FORCE"))   return "\u25B2"; // ▲
            if (upper.StartsWith("THERMAL")) return "\u25CF"; // ●
            if (upper.StartsWith("WEAR"))    return "\u25C6"; // ◆
            if (upper.StartsWith("CHATTER")) return "\u2605"; // ★
            if (upper.StartsWith("SURFACE")) return "\u25A0"; // ■
            if (upper.StartsWith("TOOL"))    return "\u26A0"; // ⚠

            return "\u26A0"; // ⚠ default
        }

        /// <summary>
        /// Get color for severity level.
        /// Less than 0.5 -> yellow (warning), >= 0.5 -> orange, >= 0.8 -> red (critical).
        /// </summary>
        public static Color GetSeverityColor(float severity)
        {
            severity = Mathf.Clamp01(severity);

            if (severity >= 0.8f)
                return SeverityHighColor;    // Red
            if (severity >= 0.5f)
                return SeverityMediumColor;  // Orange
            return SeverityLowColor;         // Yellow
        }

        // ── Multi-Axis Force Breakdown API ────────────────────────────────

        /// <summary>Current display mode for the force chart.</summary>
        public ForceDisplayMode DisplayMode => _displayMode;

        /// <summary>Current force breakdown samples (read-only access for testing).</summary>
        public IReadOnlyCollection<ForceBreakdown> ForceBreakdowns => _forceBreakdowns;

        /// <summary>Per-axis configuration accessors.</summary>
        public ForceAxisConfig TangentialAxis => tangentialAxis;
        public ForceAxisConfig RadialAxis => radialAxis;
        public ForceAxisConfig AxialAxis => axialAxis;
        public ForceAxisConfig ResultantAxis => resultantAxis;
        public ForceAxisConfig TorqueAxis => torqueAxis;
        public ForceAxisConfig PowerAxis => powerAxis;

        /// <summary>
        /// Switch between different force display modes to control which
        /// traces are rendered on the chart.
        /// </summary>
        public void SetDisplayMode(ForceDisplayMode mode)
        {
            _displayMode = mode;
        }

        /// <summary>
        /// Add a new multi-axis force breakdown sample. Automatically computes
        /// the resultant if it is zero, and trims the queue to maxSamples.
        /// </summary>
        public void AddForceBreakdown(ForceBreakdown breakdown)
        {
            if (breakdown == null) return;

            // Auto-compute resultant if not already set
            if (breakdown.resultantN <= 0f)
                breakdown.ComputeResultant();

            _forceBreakdowns.Enqueue(breakdown);
            while (_forceBreakdowns.Count > maxSamples)
                _forceBreakdowns.Dequeue();

            // Update auto-scaling per axis with 10% headroom
            UpdateAutoScale(breakdown);
        }

        /// <summary>
        /// Update per-axis max values for auto-scaling with headroom.
        /// </summary>
        private void UpdateAutoScale(ForceBreakdown breakdown)
        {
            float absT = Mathf.Abs(breakdown.tangentialN);
            float absR = Mathf.Abs(breakdown.radialN);
            float absA = Mathf.Abs(breakdown.axialN);
            float absRes = Mathf.Abs(breakdown.resultantN);
            float absTq = Mathf.Abs(breakdown.torqueNm);
            float absPw = Mathf.Abs(breakdown.powerW);

            if (absT * AutoScaleHeadroom > tangentialAxis.maxValue)
                tangentialAxis.maxValue = absT * AutoScaleHeadroom;
            if (absR * AutoScaleHeadroom > radialAxis.maxValue)
                radialAxis.maxValue = absR * AutoScaleHeadroom;
            if (absA * AutoScaleHeadroom > axialAxis.maxValue)
                axialAxis.maxValue = absA * AutoScaleHeadroom;
            if (absRes * AutoScaleHeadroom > resultantAxis.maxValue)
                resultantAxis.maxValue = absRes * AutoScaleHeadroom;
            if (absTq * AutoScaleHeadroom > torqueAxis.maxValue)
                torqueAxis.maxValue = absTq * AutoScaleHeadroom;
            if (absPw * AutoScaleHeadroom > powerAxis.maxValue)
                powerAxis.maxValue = absPw * AutoScaleHeadroom;
        }

        /// <summary>
        /// Draw multi-axis force breakdown traces based on the current display mode.
        /// Called from OnGenerateVisualContent when breakdown data is available.
        /// </summary>
        private void DrawForceBreakdownTraces(Painter2D painter,
            float left, float top, float width, float height)
        {
            var breakdowns = _forceBreakdowns.ToArray();
            if (breakdowns.Length < 2) return;

            bool showComponents = _displayMode == ForceDisplayMode.Components || _displayMode == ForceDisplayMode.All;
            bool showResultant = _displayMode == ForceDisplayMode.Resultant || _displayMode == ForceDisplayMode.All;
            bool showTorque = _displayMode == ForceDisplayMode.Torque || _displayMode == ForceDisplayMode.All;
            bool showPower = _displayMode == ForceDisplayMode.Power || _displayMode == ForceDisplayMode.All;

            // Draw component traces on primary Y-axis
            if (showComponents)
            {
                if (tangentialAxis.isVisible)
                    DrawBreakdownTrace(painter, breakdowns, b => b.tangentialN, tangentialAxis,
                        left, top, width, height);
                if (radialAxis.isVisible)
                    DrawBreakdownTrace(painter, breakdowns, b => b.radialN, radialAxis,
                        left, top, width, height);
                if (axialAxis.isVisible)
                    DrawBreakdownTrace(painter, breakdowns, b => b.axialN, axialAxis,
                        left, top, width, height);
            }

            if (showResultant && resultantAxis.isVisible)
            {
                DrawBreakdownTrace(painter, breakdowns, b => b.resultantN, resultantAxis,
                    left, top, width, height);
            }

            // Torque and power on secondary Y-axis (right side of chart)
            if (showTorque && torqueAxis.isVisible)
            {
                DrawBreakdownTrace(painter, breakdowns, b => b.torqueNm, torqueAxis,
                    left, top, width, height);
            }

            if (showPower && powerAxis.isVisible)
            {
                DrawBreakdownTrace(painter, breakdowns, b => b.powerW, powerAxis,
                    left, top, width, height);
            }
        }

        /// <summary>
        /// Draw a single breakdown trace using a value selector and axis config.
        /// </summary>
        private void DrawBreakdownTrace(Painter2D painter, ForceBreakdown[] breakdowns,
            Func<ForceBreakdown, float> valueSelector, ForceAxisConfig axisConfig,
            float left, float top, float width, float height)
        {
            if (breakdowns.Length < 2) return;

            painter.strokeColor = axisConfig.color;
            painter.lineWidth = axisConfig.lineThickness;
            painter.lineCap = LineCap.Round;
            painter.lineJoin = LineJoin.Round;

            float axisMax = Mathf.Max(axisConfig.maxValue, 0.001f);

            painter.BeginPath();
            for (int i = 0; i < breakdowns.Length; i++)
            {
                float x = left + (i / (float)(maxSamples - 1)) * width;
                float value = valueSelector(breakdowns[i]);
                float normalized = Mathf.Clamp(value / axisMax, -1f, 1f);
                float y = top + height * 0.5f - normalized * (height * 0.5f);

                if (i == 0)
                    painter.MoveTo(new Vector2(x, y));
                else
                    painter.LineTo(new Vector2(x, y));
            }
            painter.Stroke();
        }

        // ── Force Polar Plot ──────────────────────────────────────────────

        /// <summary>
        /// Draw a polar force view showing tangential vs radial force direction
        /// in the XY plane. Useful for detecting asymmetric cutting and chatter
        /// patterns. Includes a circle reference for max allowable force.
        /// </summary>
        public void DrawPolarForceView(Painter2D painter, float centerX, float centerY, float radius)
        {
            var breakdowns = _forceBreakdowns.ToArray();

            // Draw max allowable force reference circle
            painter.strokeColor = PolarRefColor;
            painter.lineWidth = 1.5f;
            painter.BeginPath();
            painter.Arc(new Vector2(centerX, centerY), radius, 0f, 360f);
            painter.ClosePath();
            painter.Stroke();

            // Draw concentric reference rings at 25%, 50%, 75%
            for (int ring = 1; ring <= 3; ring++)
            {
                float ringRadius = radius * (ring / 4f);
                painter.strokeColor = new Color(PolarRefColor.r, PolarRefColor.g, PolarRefColor.b, 0.2f);
                painter.lineWidth = 1f;
                painter.BeginPath();
                painter.Arc(new Vector2(centerX, centerY), ringRadius, 0f, 360f);
                painter.ClosePath();
                painter.Stroke();
            }

            // Draw crosshair axes
            painter.strokeColor = new Color(0.4f, 0.4f, 0.4f, 0.5f);
            painter.lineWidth = 1f;
            painter.BeginPath();
            painter.MoveTo(new Vector2(centerX - radius, centerY));
            painter.LineTo(new Vector2(centerX + radius, centerY));
            painter.Stroke();
            painter.BeginPath();
            painter.MoveTo(new Vector2(centerX, centerY - radius));
            painter.LineTo(new Vector2(centerX, centerY + radius));
            painter.Stroke();

            if (breakdowns.Length < 1) return;

            float forceScale = maxAllowableForce > 0f ? radius / maxAllowableForce : 1f;

            // Draw force trajectory (connecting successive points)
            painter.strokeColor = new Color(TangentialColor.r, TangentialColor.g, TangentialColor.b, 0.6f);
            painter.lineWidth = 1.5f;
            painter.lineCap = LineCap.Round;
            painter.lineJoin = LineJoin.Round;

            painter.BeginPath();
            for (int i = 0; i < breakdowns.Length; i++)
            {
                float px = centerX + breakdowns[i].radialN * forceScale;
                float py = centerY - breakdowns[i].tangentialN * forceScale;

                if (i == 0)
                    painter.MoveTo(new Vector2(px, py));
                else
                    painter.LineTo(new Vector2(px, py));
            }
            painter.Stroke();

            // Draw the most recent point as a filled circle
            if (breakdowns.Length > 0)
            {
                var latest = breakdowns[breakdowns.Length - 1];
                float latestX = centerX + latest.radialN * forceScale;
                float latestY = centerY - latest.tangentialN * forceScale;

                painter.fillColor = new Color(1f, 1f, 1f, 0.9f);
                painter.BeginPath();
                painter.Arc(new Vector2(latestX, latestY), 4f, 0f, 360f);
                painter.ClosePath();
                painter.Fill();

                // Draw line from center to current point for direction indicator
                painter.strokeColor = new Color(1f, 1f, 1f, 0.4f);
                painter.lineWidth = 1f;
                painter.BeginPath();
                painter.MoveTo(new Vector2(centerX, centerY));
                painter.LineTo(new Vector2(latestX, latestY));
                painter.Stroke();
            }
        }

        // ── Force Statistics Overlay ──────────────────────────────────────

        /// <summary>
        /// Draw statistical reference lines (mean, peak, std dev) and a rolling
        /// average overlay on the force chart. Shows mean as a solid line, peak
        /// as a dashed line, and +/- 1 standard deviation as shaded band.
        /// The rolling average (10-sample window) is drawn as a smooth overlay.
        /// </summary>
        public void DrawForceStatistics(Painter2D painter,
            float left, float top, float width, float height)
        {
            var breakdowns = _forceBreakdowns.ToArray();
            if (breakdowns.Length < 2) return;

            // Compute statistics on the resultant force
            float sum = 0f;
            float peak = 0f;
            float sumSq = 0f;
            for (int i = 0; i < breakdowns.Length; i++)
            {
                float v = breakdowns[i].resultantN;
                sum += v;
                sumSq += v * v;
                if (v > peak) peak = v;
            }

            float mean = sum / breakdowns.Length;
            float variance = (sumSq / breakdowns.Length) - (mean * mean);
            float stdDev = Mathf.Sqrt(Mathf.Max(variance, 0f));

            float axisMax = Mathf.Max(resultantAxis.maxValue, 0.001f);

            // Draw mean reference line (solid, semi-transparent)
            float meanY = top + height * 0.5f - (mean / axisMax) * (height * 0.5f);
            painter.strokeColor = new Color(StatsOverlayColor.r, StatsOverlayColor.g, StatsOverlayColor.b, 0.7f);
            painter.lineWidth = 1.5f;
            painter.BeginPath();
            painter.MoveTo(new Vector2(left, meanY));
            painter.LineTo(new Vector2(left + width, meanY));
            painter.Stroke();

            // Draw peak reference line (dashed)
            float peakY = top + height * 0.5f - (peak / axisMax) * (height * 0.5f);
            painter.strokeColor = new Color(1f, 0.4f, 0.4f, 0.5f);
            painter.lineWidth = 1f;
            float dashX = left;
            while (dashX < left + width)
            {
                float dashEnd = Mathf.Min(dashX + 8f, left + width);
                painter.BeginPath();
                painter.MoveTo(new Vector2(dashX, peakY));
                painter.LineTo(new Vector2(dashEnd, peakY));
                painter.Stroke();
                dashX = dashEnd + 4f;
            }

            // Draw standard deviation band (shaded region around mean)
            float stdHighY = top + height * 0.5f - ((mean + stdDev) / axisMax) * (height * 0.5f);
            float stdLowY = top + height * 0.5f - ((mean - stdDev) / axisMax) * (height * 0.5f);
            stdHighY = Mathf.Clamp(stdHighY, top, top + height);
            stdLowY = Mathf.Clamp(stdLowY, top, top + height);
            DrawRect(painter, new Color(StatsOverlayColor.r, StatsOverlayColor.g, StatsOverlayColor.b, 0.1f),
                left, stdHighY, width, stdLowY - stdHighY);

            // Draw rolling average line (10-sample window)
            DrawRollingAverage(painter, breakdowns, left, top, width, height, axisMax);

            // Draw small indicator rectangles on the right edge for mean and peak
            float indicatorW = 8f;
            float indicatorH = 4f;
            float indicatorX = left + width + 2f;
            DrawRect(painter, new Color(StatsOverlayColor.r, StatsOverlayColor.g, StatsOverlayColor.b, 0.7f),
                indicatorX, meanY - indicatorH * 0.5f, indicatorW, indicatorH);
            DrawRect(painter, new Color(1f, 0.4f, 0.4f, 0.5f),
                indicatorX, peakY - indicatorH * 0.5f, indicatorW, indicatorH);
        }

        /// <summary>
        /// Draw a rolling average line over the resultant force using a sliding window.
        /// </summary>
        private void DrawRollingAverage(Painter2D painter, ForceBreakdown[] breakdowns,
            float left, float top, float width, float height, float axisMax)
        {
            if (breakdowns.Length < RollingAverageWindow) return;

            painter.strokeColor = new Color(0.4f, 0.9f, 1f, 0.6f);
            painter.lineWidth = 2f;
            painter.lineCap = LineCap.Round;
            painter.lineJoin = LineJoin.Round;

            // Compute initial window sum
            float windowSum = 0f;
            for (int i = 0; i < RollingAverageWindow; i++)
                windowSum += breakdowns[i].resultantN;

            painter.BeginPath();
            bool first = true;

            for (int i = RollingAverageWindow - 1; i < breakdowns.Length; i++)
            {
                if (i >= RollingAverageWindow)
                {
                    windowSum += breakdowns[i].resultantN;
                    windowSum -= breakdowns[i - RollingAverageWindow].resultantN;
                }

                float avg = windowSum / RollingAverageWindow;
                float x = left + (i / (float)(maxSamples - 1)) * width;
                float normalized = Mathf.Clamp(avg / axisMax, -1f, 1f);
                float y = top + height * 0.5f - normalized * (height * 0.5f);

                if (first)
                {
                    painter.MoveTo(new Vector2(x, y));
                    first = false;
                }
                else
                {
                    painter.LineTo(new Vector2(x, y));
                }
            }
            painter.Stroke();
        }
    }
}
