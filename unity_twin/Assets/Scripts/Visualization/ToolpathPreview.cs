using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.Cutting;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// Draws a 3D preview of the G-code toolpath using a LineRenderer.
    /// Color-codes segment types: blue=rapid, green=linear cut, orange=arc.
    /// Dims already-executed segments to show progress.
    ///
    /// Subscribes to GCodeExecutor events for automatic preview/progress tracking.
    /// Coordinate mapping: G-code X→Unity X, Y→Unity Z, Z→Unity -Y (mm→meters).
    /// </summary>
    public class ToolpathPreview : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private GCodeExecutor gCodeExecutor;

        [Header("Appearance")]
        [SerializeField] private float lineWidth = 0.001f;
        [SerializeField] private Color rapidColor = new(0.3f, 0.5f, 1f, 0.4f);
        [SerializeField] private Color linearColor = new(0.2f, 1f, 0.3f, 0.7f);
        [SerializeField] private Color arcColor = new(1f, 0.6f, 0.2f, 0.7f);
        [SerializeField] private Color executedColor = new(0.3f, 0.3f, 0.3f, 0.2f);
        [SerializeField] private Color currentColor = new(1f, 0.2f, 0.2f, 1f);

        [Header("Settings")]
        [Tooltip("Number of interpolation points per arc segment")]
        [SerializeField] private int arcResolution = 16;

        public bool IsVisible { get; private set; } = true;

        private LineRenderer lineRenderer;
        private List<Vector3> worldPositions = new();
        private List<Color> baseColors = new();
        private int[] segmentStartIndices;
        private int lastProgressSegment = -1;
        private bool hasPath;

        void Awake()
        {
            lineRenderer = GetComponent<LineRenderer>();
            if (lineRenderer == null)
                lineRenderer = gameObject.AddComponent<LineRenderer>();

            lineRenderer.useWorldSpace = true;
            lineRenderer.startWidth = lineWidth;
            lineRenderer.endWidth = lineWidth;
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            lineRenderer.positionCount = 0;
            lineRenderer.numCornerVertices = 2;
            lineRenderer.numCapVertices = 2;
        }

        void OnEnable()
        {
            if (gCodeExecutor != null)
            {
                gCodeExecutor.OnProgramLoaded += OnProgramLoaded;
                gCodeExecutor.OnExecutionComplete += OnExecutionComplete;
            }
        }

        void OnDisable()
        {
            if (gCodeExecutor != null)
            {
                gCodeExecutor.OnProgramLoaded -= OnProgramLoaded;
                gCodeExecutor.OnExecutionComplete -= OnExecutionComplete;
            }
        }

        /// <summary>Set the executor reference at runtime (for machine switching).</summary>
        public void SetExecutor(GCodeExecutor executor)
        {
            if (gCodeExecutor != null)
            {
                gCodeExecutor.OnProgramLoaded -= OnProgramLoaded;
                gCodeExecutor.OnExecutionComplete -= OnExecutionComplete;
            }

            gCodeExecutor = executor;

            if (gCodeExecutor != null && isActiveAndEnabled)
            {
                gCodeExecutor.OnProgramLoaded += OnProgramLoaded;
                gCodeExecutor.OnExecutionComplete += OnExecutionComplete;
            }
        }

        /// <summary>Legacy API: directly set toolpath segments (without GCodeExecutor events).</summary>
        public void SetToolpath(List<ToolpathSegment> toolpathSegments)
        {
            BuildPath(toolpathSegments);
        }

        /// <summary>Legacy API: advance to segment index.</summary>
        public void AdvanceToSegment(int index)
        {
            UpdateProgressToSegment(index);
        }

        private void OnProgramLoaded(IReadOnlyList<ToolpathSegment> segments)
        {
            BuildPath(segments);
        }

        private void OnExecutionComplete()
        {
            Invoke(nameof(Clear), 3f);
        }

        void Update()
        {
            if (!hasPath || gCodeExecutor == null || !gCodeExecutor.IsExecuting) return;

            int currentSeg = gCodeExecutor.CurrentSegmentIndex;
            if (currentSeg != lastProgressSegment)
            {
                UpdateProgressToSegment(currentSeg);
                lastProgressSegment = currentSeg;
            }
        }

        private void BuildPath(IReadOnlyList<ToolpathSegment> segments)
        {
            worldPositions.Clear();
            baseColors.Clear();
            lastProgressSegment = -1;

            Transform machineTransform = gCodeExecutor != null ? gCodeExecutor.transform : transform;

            var startIndices = new List<int>();

            for (int i = 0; i < segments.Count; i++)
            {
                var seg = segments[i];
                startIndices.Add(worldPositions.Count);

                Color color = seg.type switch
                {
                    SegmentType.Rapid => rapidColor,
                    SegmentType.CWArc or SegmentType.CCWArc => arcColor,
                    _ => linearColor
                };

                if (seg.type == SegmentType.CWArc || seg.type == SegmentType.CCWArc)
                {
                    for (int j = 0; j <= arcResolution; j++)
                    {
                        float t = (float)j / arcResolution;
                        Vector3 pos = InterpolateArc(seg, t);
                        worldPositions.Add(GCodeToWorld(pos, machineTransform));
                        baseColors.Add(color);
                    }
                }
                else
                {
                    worldPositions.Add(GCodeToWorld(seg.startPos, machineTransform));
                    baseColors.Add(color);
                    worldPositions.Add(GCodeToWorld(seg.endPos, machineTransform));
                    baseColors.Add(color);
                }
            }

            segmentStartIndices = startIndices.ToArray();

            lineRenderer.positionCount = worldPositions.Count;
            lineRenderer.SetPositions(worldPositions.ToArray());
            ApplyGradient(baseColors);

            hasPath = true;
            Debug.Log($"[ToolpathPreview] Built preview: {segments.Count} segments, {worldPositions.Count} line points");
        }

        /// <summary>
        /// Convert G-code position (mm) to Unity world space.
        /// G-code X → Unity X, G-code Y → Unity Z, G-code Z → Unity -Y.
        /// </summary>
        private static Vector3 GCodeToWorld(Vector3 gcodePosMM, Transform machineTransform)
        {
            Vector3 localPos = new(
                gcodePosMM.x / 1000f,
                -gcodePosMM.z / 1000f,
                gcodePosMM.y / 1000f
            );
            return machineTransform.TransformPoint(localPos);
        }

        private static Vector3 InterpolateArc(ToolpathSegment seg, float t)
        {
            Vector3 center = seg.arcCenter;
            Vector2 startDir = new(seg.startPos.x - center.x, seg.startPos.y - center.y);

            float startAngle = Mathf.Atan2(startDir.y, startDir.x);
            float radius = startDir.magnitude;

            Vector2 endDir = new(seg.endPos.x - center.x, seg.endPos.y - center.y);
            float endAngle = Mathf.Atan2(endDir.y, endDir.x);

            float sweep;
            if (seg.type == SegmentType.CWArc)
            {
                sweep = startAngle - endAngle;
                if (sweep <= 0) sweep += 2f * Mathf.PI;
                sweep = -sweep;
            }
            else
            {
                sweep = endAngle - startAngle;
                if (sweep <= 0) sweep += 2f * Mathf.PI;
            }

            float angle = startAngle + sweep * t;
            float z = Mathf.Lerp(seg.startPos.z, seg.endPos.z, t);

            return new Vector3(
                center.x + radius * Mathf.Cos(angle),
                center.y + radius * Mathf.Sin(angle),
                z
            );
        }

        private void UpdateProgressToSegment(int currentSeg)
        {
            if (segmentStartIndices == null) return;

            var displayColors = new List<Color>(baseColors);

            for (int i = 0; i < segmentStartIndices.Length; i++)
            {
                int start = segmentStartIndices[i];
                int end = (i + 1 < segmentStartIndices.Length)
                    ? segmentStartIndices[i + 1]
                    : displayColors.Count;

                Color c;
                if (i < currentSeg)
                    c = executedColor;
                else if (i == currentSeg)
                    c = currentColor;
                else
                    continue; // keep base color

                for (int j = start; j < end; j++)
                    displayColors[j] = c;
            }

            ApplyGradient(displayColors);
        }

        private void ApplyGradient(List<Color> colors)
        {
            if (colors.Count < 2) return;

            // Sample up to 8 gradient keys from the color list
            int keyCount = Mathf.Min(8, colors.Count);
            var colorKeys = new GradientColorKey[keyCount];
            var alphaKeys = new GradientAlphaKey[keyCount];

            for (int i = 0; i < keyCount; i++)
            {
                float t = (float)i / (keyCount - 1);
                int idx = Mathf.Min(Mathf.FloorToInt(t * (colors.Count - 1)), colors.Count - 1);
                colorKeys[i] = new GradientColorKey(colors[idx], t);
                alphaKeys[i] = new GradientAlphaKey(colors[idx].a, t);
            }

            var gradient = new Gradient();
            gradient.SetKeys(colorKeys, alphaKeys);
            lineRenderer.colorGradient = gradient;
        }

        public void Toggle()
        {
            IsVisible = !IsVisible;
            lineRenderer.enabled = IsVisible;
        }

        public void Clear()
        {
            lineRenderer.positionCount = 0;
            worldPositions.Clear();
            baseColors.Clear();
            segmentStartIndices = null;
            lastProgressSegment = -1;
            hasPath = false;
        }
    }
}
