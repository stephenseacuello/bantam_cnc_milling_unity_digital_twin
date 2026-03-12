using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using Unity.Profiling;
using MiracleTwin.Core;
using MiracleTwin.Cutting;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Scrolling line chart for Fx, Fy, Fz forces over time.
    /// Uses UI Toolkit custom drawing via generateVisualContent/Painter2D
    /// for efficient rendering of color-coded force traces with grid lines,
    /// axis labels, and a legend.
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
    }
}
