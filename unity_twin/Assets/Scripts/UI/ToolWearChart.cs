using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Scrolling line chart plotting flankWearVB over time with a red danger
    /// line at VBmax (tool end-of-life threshold).
    ///
    /// Uses UI Toolkit custom drawing via generateVisualContent/Painter2D.
    /// Subscribes to CuttingStateEventSO for data, buffers samples in Queue.
    /// </summary>
    public class ToolWearChart : MonoBehaviour
    {
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private int maxSamples = 200;
        [SerializeField] private float maxWear = 0.35f;          // mm, Y-axis max
        [SerializeField] private float vbMaxThreshold = 0.30f;   // mm, red danger line
        [SerializeField] private float sampleInterval = 0.1f;    // Wear changes slowly, lower sample rate

        [Header("Appearance")]
        [SerializeField] private float lineWidth = 2f;
        [SerializeField] private int gridLinesHorizontal = 4;
        [SerializeField] private int gridLinesVertical = 5;

        private static readonly Color WearColor = new(0.9f, 0.7f, 0.1f, 1f);       // Yellow/amber
        private static readonly Color DangerLineColor = new(1f, 0.15f, 0.15f, 0.9f); // Red
        private static readonly Color GridColor = new(0.3f, 0.3f, 0.3f, 0.5f);
        private static readonly Color BackgroundColor = new(0.1f, 0.1f, 0.12f, 0.9f);

        private readonly Queue<float> wearSamples = new();
        private float lastSampleTime;
        private VisualElement chartElement;

        void OnEnable()
        {
            if (cuttingStateEvent != null)
                cuttingStateEvent.Register(OnCuttingState);
        }

        void OnDisable()
        {
            if (cuttingStateEvent != null)
                cuttingStateEvent.Unregister(OnCuttingState);
        }

        void Start()
        {
            if (uiDocument == null) return;
            var root = uiDocument.rootVisualElement;

            chartElement = root.Q<VisualElement>("tool-wear-chart");
            if (chartElement == null)
                chartElement = root.Q<VisualElement>("chart-container-wear");

            if (chartElement != null)
                chartElement.generateVisualContent += OnGenerateVisualContent;
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

            wearSamples.Enqueue(state.flankWearVB);
            while (wearSamples.Count > maxSamples)
                wearSamples.Dequeue();
        }

        void Update()
        {
            chartElement?.MarkDirtyRepaint();
        }

        private void OnGenerateVisualContent(MeshGenerationContext ctx)
        {
            var painter = ctx.painter2D;
            Rect rect = chartElement.contentRect;
            if (rect.width < 10 || rect.height < 10) return;

            float padding = 40f;
            float rightPad = 10f;
            float topPad = 25f;
            float bottomPad = 20f;

            float chartLeft = padding;
            float chartTop = topPad;
            float chartWidth = rect.width - padding - rightPad;
            float chartHeight = rect.height - topPad - bottomPad;

            if (chartWidth < 10 || chartHeight < 10) return;

            // Background
            painter.fillColor = BackgroundColor;
            painter.BeginPath();
            painter.MoveTo(new Vector2(0, 0));
            painter.LineTo(new Vector2(rect.width, 0));
            painter.LineTo(new Vector2(rect.width, rect.height));
            painter.LineTo(new Vector2(0, rect.height));
            painter.ClosePath();
            painter.Fill();

            // Grid
            painter.strokeColor = GridColor;
            painter.lineWidth = 1f;

            for (int i = 0; i <= gridLinesHorizontal; i++)
            {
                float y = chartTop + (chartHeight / gridLinesHorizontal) * i;
                painter.BeginPath();
                painter.MoveTo(new Vector2(chartLeft, y));
                painter.LineTo(new Vector2(chartLeft + chartWidth, y));
                painter.Stroke();
            }

            for (int i = 0; i <= gridLinesVertical; i++)
            {
                float x = chartLeft + (chartWidth / gridLinesVertical) * i;
                painter.BeginPath();
                painter.MoveTo(new Vector2(x, chartTop));
                painter.LineTo(new Vector2(x, chartTop + chartHeight));
                painter.Stroke();
            }

            // VBmax danger line (red dashed horizontal line)
            float dangerNormalized = Mathf.InverseLerp(0f, maxWear, vbMaxThreshold);
            float dangerY = chartTop + chartHeight * (1f - dangerNormalized);

            painter.strokeColor = DangerLineColor;
            painter.lineWidth = 2f;

            // Draw dashed line segments for VBmax threshold
            float dashLength = 8f;
            float gapLength = 5f;
            float currentX = chartLeft;
            while (currentX < chartLeft + chartWidth)
            {
                float endX = Mathf.Min(currentX + dashLength, chartLeft + chartWidth);
                painter.BeginPath();
                painter.MoveTo(new Vector2(currentX, dangerY));
                painter.LineTo(new Vector2(endX, dangerY));
                painter.Stroke();
                currentX = endX + gapLength;
            }

            // Draw wear trace
            var samples = wearSamples.ToArray();
            if (samples.Length > 1)
            {
                painter.strokeColor = WearColor;
                painter.lineWidth = lineWidth;
                painter.lineCap = LineCap.Round;
                painter.lineJoin = LineJoin.Round;

                painter.BeginPath();

                for (int i = 0; i < samples.Length; i++)
                {
                    float x = chartLeft + (i / (float)(maxSamples - 1)) * chartWidth;
                    float normalized = Mathf.InverseLerp(0f, maxWear, samples[i]);
                    float y = chartTop + chartHeight * (1f - normalized);

                    if (i == 0)
                        painter.MoveTo(new Vector2(x, y));
                    else
                        painter.LineTo(new Vector2(x, y));
                }

                painter.Stroke();
            }

            // Axis border
            painter.strokeColor = new Color(0.5f, 0.5f, 0.5f, 0.8f);
            painter.lineWidth = 1.5f;
            painter.BeginPath();
            painter.MoveTo(new Vector2(chartLeft, chartTop));
            painter.LineTo(new Vector2(chartLeft, chartTop + chartHeight));
            painter.LineTo(new Vector2(chartLeft + chartWidth, chartTop + chartHeight));
            painter.Stroke();

            // Legend key
            float legendY = 5f;
            float legendX = chartLeft;
            float keySize = 10f;

            painter.fillColor = WearColor;
            painter.BeginPath();
            painter.MoveTo(new Vector2(legendX, legendY));
            painter.LineTo(new Vector2(legendX + keySize, legendY));
            painter.LineTo(new Vector2(legendX + keySize, legendY + keySize));
            painter.LineTo(new Vector2(legendX, legendY + keySize));
            painter.ClosePath();
            painter.Fill();

            // VBmax legend key (red dash)
            painter.strokeColor = DangerLineColor;
            painter.lineWidth = 2f;
            float dangerLegendX = legendX + 80f;
            painter.BeginPath();
            painter.MoveTo(new Vector2(dangerLegendX, legendY + keySize * 0.5f));
            painter.LineTo(new Vector2(dangerLegendX + 15f, legendY + keySize * 0.5f));
            painter.Stroke();
        }

        public void Clear()
        {
            wearSamples.Clear();
        }
    }
}
