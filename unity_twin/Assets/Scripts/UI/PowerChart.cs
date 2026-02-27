using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Scrolling line chart plotting powerWatts over time with a nominal
    /// power reference line showing the machine's typical operating power.
    ///
    /// Uses UI Toolkit custom drawing via generateVisualContent/Painter2D.
    /// Subscribes to CuttingStateEventSO for data, buffers samples in Queue.
    /// </summary>
    public class PowerChart : MonoBehaviour
    {
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private int maxSamples = 200;
        [SerializeField] private float maxPower = 500f;           // Watts, Y-axis max
        [SerializeField] private float nominalPower = 200f;       // Watts, reference line for Bantam Explorer
        [SerializeField] private float sampleInterval = 0.033f;

        [Header("Appearance")]
        [SerializeField] private float lineWidth = 2f;
        [SerializeField] private int gridLinesHorizontal = 4;
        [SerializeField] private int gridLinesVertical = 5;

        private static readonly Color PowerColor = new(0.2f, 0.7f, 1f, 1f);          // Cyan/blue
        private static readonly Color NominalLineColor = new(0.5f, 0.8f, 0.2f, 0.7f); // Green reference
        private static readonly Color GridColor = new(0.3f, 0.3f, 0.3f, 0.5f);
        private static readonly Color BackgroundColor = new(0.1f, 0.1f, 0.12f, 0.9f);

        private readonly Queue<float> powerSamples = new();
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

            chartElement = root.Q<VisualElement>("power-chart");
            if (chartElement == null)
                chartElement = root.Q<VisualElement>("chart-container-power");

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

            powerSamples.Enqueue(state.powerWatts);
            while (powerSamples.Count > maxSamples)
                powerSamples.Dequeue();
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

            // Nominal power reference line (green dashed)
            float nominalNormalized = Mathf.InverseLerp(0f, maxPower, nominalPower);
            float nominalY = chartTop + chartHeight * (1f - nominalNormalized);

            painter.strokeColor = NominalLineColor;
            painter.lineWidth = 1.5f;

            float dashLength = 8f;
            float gapLength = 5f;
            float currentX = chartLeft;
            while (currentX < chartLeft + chartWidth)
            {
                float endX = Mathf.Min(currentX + dashLength, chartLeft + chartWidth);
                painter.BeginPath();
                painter.MoveTo(new Vector2(currentX, nominalY));
                painter.LineTo(new Vector2(endX, nominalY));
                painter.Stroke();
                currentX = endX + gapLength;
            }

            // Draw power trace
            var samples = powerSamples.ToArray();
            if (samples.Length > 1)
            {
                painter.strokeColor = PowerColor;
                painter.lineWidth = lineWidth;
                painter.lineCap = LineCap.Round;
                painter.lineJoin = LineJoin.Round;

                painter.BeginPath();

                for (int i = 0; i < samples.Length; i++)
                {
                    float x = chartLeft + (i / (float)(maxSamples - 1)) * chartWidth;
                    float normalized = Mathf.InverseLerp(0f, maxPower, samples[i]);
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

            // Legend keys
            float legendY = 5f;
            float legendX = chartLeft;
            float keySize = 10f;

            // Power legend key
            painter.fillColor = PowerColor;
            painter.BeginPath();
            painter.MoveTo(new Vector2(legendX, legendY));
            painter.LineTo(new Vector2(legendX + keySize, legendY));
            painter.LineTo(new Vector2(legendX + keySize, legendY + keySize));
            painter.LineTo(new Vector2(legendX, legendY + keySize));
            painter.ClosePath();
            painter.Fill();

            // Nominal power legend key (green dash)
            painter.strokeColor = NominalLineColor;
            painter.lineWidth = 2f;
            float nominalLegendX = legendX + 80f;
            painter.BeginPath();
            painter.MoveTo(new Vector2(nominalLegendX, legendY + keySize * 0.5f));
            painter.LineTo(new Vector2(nominalLegendX + 15f, legendY + keySize * 0.5f));
            painter.Stroke();
        }

        public void Clear()
        {
            powerSamples.Clear();
        }
    }
}
