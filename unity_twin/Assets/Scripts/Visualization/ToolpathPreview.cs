using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.Cutting;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// Renders upcoming G-code toolpath using LineRenderer.
    /// Green = rapid (G0), Blue = feed (G1), Red = current segment.
    /// Shows the next N segments ahead of current position.
    /// </summary>
    public class ToolpathPreview : MonoBehaviour
    {
        [Header("Settings")]
        [SerializeField] private int previewSegments = 50;
        [SerializeField] private float lineWidth = 0.001f;
        [SerializeField] private float rapidLineWidth = 0.0005f;

        [Header("Colors")]
        [SerializeField] private Color rapidColor = new(0.2f, 0.9f, 0.2f, 0.5f);
        [SerializeField] private Color feedColor = new(0.3f, 0.3f, 1f, 0.7f);
        [SerializeField] private Color currentColor = new(1f, 0.2f, 0.2f, 1f);
        [SerializeField] private Color arcColor = new(0.8f, 0.3f, 1f, 0.7f);

        public bool IsVisible { get; private set; } = true;

        private LineRenderer lineRenderer;
        private List<ToolpathSegment> segments;
        private int currentSegmentIndex;

        void Awake()
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
            lineRenderer.useWorldSpace = true;
            lineRenderer.startWidth = lineWidth;
            lineRenderer.endWidth = lineWidth;
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        }

        /// <summary>Load toolpath segments for preview.</summary>
        public void SetToolpath(List<ToolpathSegment> toolpathSegments)
        {
            segments = toolpathSegments;
            currentSegmentIndex = 0;
            UpdatePreview();
        }

        /// <summary>Advance to the next segment (called as G-code executes).</summary>
        public void AdvanceToSegment(int index)
        {
            currentSegmentIndex = index;
            UpdatePreview();
        }

        private void UpdatePreview()
        {
            if (segments == null || !IsVisible)
            {
                lineRenderer.positionCount = 0;
                return;
            }

            int startIdx = currentSegmentIndex;
            int endIdx = Mathf.Min(startIdx + previewSegments, segments.Count);
            int count = endIdx - startIdx;

            if (count <= 0)
            {
                lineRenderer.positionCount = 0;
                return;
            }

            // Build position array
            var positions = new List<Vector3>();
            var colors = new List<Color>();

            for (int i = startIdx; i < endIdx; i++)
            {
                var seg = segments[i];
                Vector3 start = seg.startPos / 1000f; // mm → meters
                Vector3 end = seg.endPos / 1000f;

                positions.Add(start);
                positions.Add(end);

                Color c = seg.type switch
                {
                    SegmentType.Rapid => rapidColor,
                    SegmentType.CWArc => arcColor,
                    SegmentType.CCWArc => arcColor,
                    _ => i == currentSegmentIndex ? currentColor : feedColor
                };
                colors.Add(c);
                colors.Add(c);
            }

            lineRenderer.positionCount = positions.Count;
            lineRenderer.SetPositions(positions.ToArray());

            // Apply gradient
            if (positions.Count >= 2)
            {
                var gradient = new Gradient();
                var colorKeys = new GradientColorKey[Mathf.Min(8, colors.Count)];
                var alphaKeys = new GradientAlphaKey[2];
                alphaKeys[0] = new GradientAlphaKey(1f, 0f);
                alphaKeys[1] = new GradientAlphaKey(0.3f, 1f);

                for (int i = 0; i < colorKeys.Length; i++)
                {
                    float t = (float)i / (colorKeys.Length - 1);
                    int idx = Mathf.Min((int)(t * colors.Count), colors.Count - 1);
                    colorKeys[i] = new GradientColorKey(colors[idx], t);
                }

                gradient.SetKeys(colorKeys, alphaKeys);
                lineRenderer.colorGradient = gradient;
            }
        }

        public void Toggle()
        {
            IsVisible = !IsVisible;
            UpdatePreview();
        }

        public void Clear()
        {
            segments = null;
            lineRenderer.positionCount = 0;
        }
    }
}
