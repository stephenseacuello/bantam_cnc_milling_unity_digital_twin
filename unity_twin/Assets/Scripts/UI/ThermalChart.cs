using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Scrolling line chart plotting toolTemperature and interfaceTemperature
    /// as two separate lines over time.
    ///
    /// Uses UI Toolkit custom drawing via generateVisualContent/Painter2D.
    /// Subscribes to CuttingStateEventSO for data, buffers samples in Queue.
    /// </summary>
    public class ThermalChart : MonoBehaviour
    {
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private int maxSamples = 200;
        [SerializeField] private float maxTemperature = 300f;
        [SerializeField] private float minTemperature = 20f;
        [SerializeField] private float sampleInterval = 0.033f;

        [Header("Appearance")]
        [SerializeField] private float lineWidth = 2f;
        [SerializeField] private int gridLinesHorizontal = 4;
        [SerializeField] private int gridLinesVertical = 5;

        private static readonly Color ToolTempColor = new(1f, 0.6f, 0.1f, 1f);      // Orange
        private static readonly Color InterfaceTempColor = new(1f, 0.2f, 0.2f, 1f);  // Red
        private static readonly Color GridColor = new(0.3f, 0.3f, 0.3f, 0.5f);
        private static readonly Color BackgroundColor = new(0.1f, 0.1f, 0.12f, 0.9f);

        /// <summary>x = toolTemperature, y = interfaceTemperature</summary>
        private readonly Queue<Vector2> temperatureSamples = new();
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

            chartElement = root.Q<VisualElement>("thermal-chart");
            if (chartElement == null)
                chartElement = root.Q<VisualElement>("chart-container-thermal");

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

            temperatureSamples.Enqueue(new Vector2(state.toolTemperature, state.interfaceTemperature));
            while (temperatureSamples.Count > maxSamples)
                temperatureSamples.Dequeue();
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

            // Draw traces
            var samples = temperatureSamples.ToArray();
            if (samples.Length > 1)
            {
                DrawTrace(painter, samples, 0, ToolTempColor, chartLeft, chartTop, chartWidth, chartHeight);
                DrawTrace(painter, samples, 1, InterfaceTempColor, chartLeft, chartTop, chartWidth, chartHeight);
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
            float keySpacing = 100f;

            painter.fillColor = ToolTempColor;
            painter.BeginPath();
            painter.MoveTo(new Vector2(legendX, legendY));
            painter.LineTo(new Vector2(legendX + keySize, legendY));
            painter.LineTo(new Vector2(legendX + keySize, legendY + keySize));
            painter.LineTo(new Vector2(legendX, legendY + keySize));
            painter.ClosePath();
            painter.Fill();

            painter.fillColor = InterfaceTempColor;
            painter.BeginPath();
            painter.MoveTo(new Vector2(legendX + keySpacing, legendY));
            painter.LineTo(new Vector2(legendX + keySpacing + keySize, legendY));
            painter.LineTo(new Vector2(legendX + keySpacing + keySize, legendY + keySize));
            painter.LineTo(new Vector2(legendX + keySpacing, legendY + keySize));
            painter.ClosePath();
            painter.Fill();
        }

        private void DrawTrace(Painter2D painter, Vector2[] samples, int component, Color color,
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
                float value = component == 0 ? samples[i].x : samples[i].y;

                float normalized = Mathf.InverseLerp(minTemperature, maxTemperature, value);
                float y = top + height * (1f - normalized);

                if (i == 0)
                    painter.MoveTo(new Vector2(x, y));
                else
                    painter.LineTo(new Vector2(x, y));
            }

            painter.Stroke();
        }

        public void Clear()
        {
            temperatureSamples.Clear();
        }
    }
}
