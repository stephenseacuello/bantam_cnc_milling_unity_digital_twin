using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.Cutting;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// Renders the results of a GCodeExecutor dry-run preview as color-coded 3D
    /// line segments. Each segment is drawn with a risk color derived from the
    /// PreviewReport: green = safe, yellow = medium chatter/near-miss,
    /// red = collision or high chatter risk.
    ///
    /// Subscribes to <see cref="GCodeExecutor.OnPreviewComplete"/> and automatically
    /// builds the visualization when a preview finishes. Call <see cref="ClearPreview"/>
    /// to destroy all rendered segments.
    /// </summary>
    public class ToolpathPreviewRenderer : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private GCodeExecutor gCodeExecutor;

        [Header("Appearance")]
        [Tooltip("Width of each segment line in world units (meters).")]
        [SerializeField] private float lineWidth = 0.0015f;

        [Tooltip("Color for segments with no risk (chatter LOW, no collision).")]
        [SerializeField] private Color safeColor = new(0.1f, 0.9f, 0.2f, 0.8f);

        [Tooltip("Color for segments with medium risk (chatter MEDIUM or near-miss).")]
        [SerializeField] private Color warningColor = new(1f, 0.85f, 0.1f, 0.9f);

        [Tooltip("Color for segments with high risk (collision or chatter HIGH).")]
        [SerializeField] private Color dangerColor = new(1f, 0.15f, 0.1f, 1f);

        /// <summary>Container for all spawned segment LineRenderer GameObjects.</summary>
        private GameObject segmentContainer;

        /// <summary>Cached list of spawned segment GameObjects for cleanup.</summary>
        private readonly List<GameObject> segmentObjects = new();

        private Material lineMaterial;

        void Awake()
        {
            // Shared material for all line renderers
            lineMaterial = new Material(Shader.Find("Sprites/Default"));
        }

        void OnEnable()
        {
            if (gCodeExecutor != null)
                gCodeExecutor.OnPreviewComplete += OnPreviewComplete;
        }

        void OnDisable()
        {
            if (gCodeExecutor != null)
                gCodeExecutor.OnPreviewComplete -= OnPreviewComplete;
        }

        /// <summary>Set the executor reference at runtime (e.g. after machine switching).</summary>
        public void SetExecutor(GCodeExecutor executor)
        {
            // Unsubscribe from previous
            if (gCodeExecutor != null)
                gCodeExecutor.OnPreviewComplete -= OnPreviewComplete;

            gCodeExecutor = executor;

            if (gCodeExecutor != null && isActiveAndEnabled)
                gCodeExecutor.OnPreviewComplete += OnPreviewComplete;
        }

        private void OnPreviewComplete(GCodeExecutor.PreviewReport report)
        {
            ClearPreview();
            BuildPreview(report);
        }

        /// <summary>
        /// Build a 3D line visualization from the preview report. Creates one
        /// LineRenderer per segment so each can carry its own risk color.
        /// </summary>
        private void BuildPreview(GCodeExecutor.PreviewReport report)
        {
            if (report == null || report.segments == null || report.segments.Count == 0)
                return;

            // Create a container to keep the hierarchy tidy
            segmentContainer = new GameObject("PreviewSegments");
            segmentContainer.transform.SetParent(transform, false);

            Transform machineTransform = gCodeExecutor != null ? gCodeExecutor.transform : transform;

            for (int i = 0; i < report.segments.Count; i++)
            {
                var seg = report.segments[i];

                // Map risk color from the report, falling back to our configured palette
                Color color = ResolveColor(seg.riskColor, seg.chatterRisk, seg.collisionSeverity);

                GameObject segGO = new($"Seg_{seg.blockIndex}");
                segGO.transform.SetParent(segmentContainer.transform, false);

                var lr = segGO.AddComponent<LineRenderer>();
                lr.useWorldSpace = true;
                lr.startWidth = lineWidth;
                lr.endWidth = lineWidth;
                lr.material = lineMaterial;
                lr.numCornerVertices = 2;
                lr.numCapVertices = 2;
                lr.positionCount = 2;

                lr.SetPosition(0, GCodeToWorld(seg.startPos, machineTransform));
                lr.SetPosition(1, GCodeToWorld(seg.endPos, machineTransform));

                // Apply uniform color via gradient
                var gradient = new Gradient();
                gradient.SetKeys(
                    new[] { new GradientColorKey(color, 0f), new GradientColorKey(color, 1f) },
                    new[] { new GradientAlphaKey(color.a, 0f), new GradientAlphaKey(color.a, 1f) }
                );
                lr.colorGradient = gradient;

                segmentObjects.Add(segGO);
            }

            Debug.Log($"[ToolpathPreviewRenderer] Rendered {report.segments.Count} preview segments.");
        }

        /// <summary>
        /// Choose the display color for a segment. Uses the report's riskColor when
        /// it differs from default, otherwise classifies from chatter risk and collision.
        /// </summary>
        private Color ResolveColor(Color reportColor, string chatterRisk,
            FixtureProfile.CollisionSeverity collision)
        {
            // If the report already computed a meaningful color, prefer our palette mapping
            // based on the same logic for visual consistency with the configured colors.
            if (collision != FixtureProfile.CollisionSeverity.None &&
                collision != FixtureProfile.CollisionSeverity.NearMiss)
                return dangerColor;

            if (chatterRisk == "HIGH")
                return dangerColor;

            if (chatterRisk == "MEDIUM" ||
                collision == FixtureProfile.CollisionSeverity.NearMiss)
                return warningColor;

            return safeColor;
        }

        /// <summary>
        /// Convert G-code position (mm) to Unity world space.
        /// Matches the convention used by ToolpathPreview:
        /// G-code X -> Unity X, G-code Y -> Unity Z, G-code Z -> Unity -Y.
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

        /// <summary>
        /// Destroy all rendered preview line segments and their container.
        /// </summary>
        public void ClearPreview()
        {
            foreach (var go in segmentObjects)
            {
                if (go != null)
                    Destroy(go);
            }
            segmentObjects.Clear();

            if (segmentContainer != null)
            {
                Destroy(segmentContainer);
                segmentContainer = null;
            }
        }

        void OnDestroy()
        {
            ClearPreview();

            if (lineMaterial != null)
                Destroy(lineMaterial);
        }
    }
}
