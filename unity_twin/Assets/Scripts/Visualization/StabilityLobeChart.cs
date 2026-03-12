using UnityEngine;
using MiracleTwin.Cutting;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// 2D stability lobe diagram overlay showing safe cutting parameter zones.
    /// Uses Altintas-Budak zeroth-order approximation (ZOA) for chatter stability limits.
    /// Displays current operating point on the diagram.
    /// </summary>
    public class StabilityLobeChart : MonoBehaviour
    {
        [Header("Machine Parameters")]
        [SerializeField] private float naturalFrequency = 800f;    // Hz
        [SerializeField] private float dampingRatio = 0.03f;
        [SerializeField] private float stiffness = 5e6f;          // N/m
        [SerializeField] private float Ktc = 796f;                 // N/mm²
        [SerializeField] private int fluteCount = 2;

        [Header("Display")]
        [SerializeField] private RectTransform chartArea;
        [SerializeField] private float minRPM = 5000f;
        [SerializeField] private float maxRPM = 25000f;
        [SerializeField] private float maxDepth = 5f;              // mm
        [SerializeField] private int lobePoints = 200;
        [SerializeField] private Color stableColor = new(0.2f, 0.8f, 0.2f, 0.3f);
        [SerializeField] private Color unstableColor = new(0.8f, 0.2f, 0.2f, 0.3f);
        [SerializeField] private Color operatingPointColor = Color.yellow;
        [SerializeField] private Color nearBoundaryColor = new(1f, 0.9f, 0.2f, 0.5f);

        [Header("Lobe Surface Rendering")]
        [SerializeField] private StabilityLobePredictor lobePredictor;
        [SerializeField] private int lobeSurfaceResolution = 50;

        /// <summary>Texture used to render the wear-adjusted lobe surface.</summary>
        private Texture2D lobeSurfaceTexture;
        private GameObject lobeSurfaceQuad;

        public float CurrentRPM { get; set; }
        public float CurrentDepth { get; set; }
        public bool IsStable { get; private set; } = true;

        /// <summary>Lookahead operating points to render on the diagram.</summary>
        private readonly System.Collections.Generic.List<Vector2> lookaheadPoints = new();

        /// <summary>Add a set of lookahead operating points (RPM, depth) to visualize.</summary>
        public void SetLookaheadPoints(System.Collections.Generic.List<Vector2> points)
        {
            lookaheadPoints.Clear();
            if (points != null)
                lookaheadPoints.AddRange(points);
        }

        /// <summary>Clear all lookahead points.</summary>
        public void ClearLookaheadPoints()
        {
            lookaheadPoints.Clear();
        }

        private LineRenderer lobeLine;

        void Start()
        {
            lobeLine = gameObject.AddComponent<LineRenderer>();
            lobeLine.useWorldSpace = false;
            lobeLine.startWidth = 0.002f;
            lobeLine.endWidth = 0.002f;
            var shader = Shader.Find("Universal Render Pipeline/Unlit")
                      ?? Shader.Find("Sprites/Default")
                      ?? Shader.Find("Hidden/InternalErrorShader");
            if (shader != null)
                lobeLine.material = new Material(shader);
            lobeLine.startColor = Color.white;
            lobeLine.endColor = Color.white;

            ComputeStabilityLobes();
        }

        /// <summary>
        /// Compute stability lobe diagram using Altintas-Budak ZOA.
        /// ap_lim = -1 / (2 · Ktc · N · Re[G(jω)])
        /// where G(jω) is the transfer function at chatter frequency ω.
        /// </summary>
        private void ComputeStabilityLobes()
        {
            var positions = new Vector3[lobePoints];

            for (int i = 0; i < lobePoints; i++)
            {
                float t = (float)i / (lobePoints - 1);
                float rpm = Mathf.Lerp(minRPM, maxRPM, t);

                float apLim = CalculateStabilityLimit(rpm);
                apLim = Mathf.Clamp(apLim, 0, maxDepth);

                float xNorm = t;
                float yNorm = apLim / maxDepth;

                positions[i] = new Vector3(xNorm - 0.5f, yNorm - 0.5f, 0);
            }

            lobeLine.positionCount = lobePoints;
            lobeLine.SetPositions(positions);
        }

        private float CalculateStabilityLimit(float rpm)
        {
            float omega_n = naturalFrequency * 2f * Mathf.PI;
            float toothPassFreq = rpm * fluteCount / 60f;

            // Sweep chatter frequencies near natural frequency
            float bestApLim = float.MaxValue;

            for (float ratio = 0.5f; ratio < 2.0f; ratio += 0.01f)
            {
                float omega_c = omega_n * ratio;

                // Transfer function at chatter frequency
                float r = omega_c / omega_n;
                float realG = (1f - r * r) / (stiffness * ((1f - r * r) * (1f - r * r) +
                              (2f * dampingRatio * r) * (2f * dampingRatio * r)));

                if (realG >= 0) continue; // Only unstable when Re[G] < 0

                float apLim = -1f / (2f * Ktc * fluteCount * realG);

                // Check if this chatter frequency matches a lobe
                float N_lobe = (omega_c - Mathf.Atan2(-2f * dampingRatio * r, 1f - r * r)) /
                               (2f * Mathf.PI * toothPassFreq);

                if (N_lobe > 0 && apLim > 0 && apLim < bestApLim)
                    bestApLim = apLim;
            }

            return bestApLim < float.MaxValue ? bestApLim : maxDepth;
        }

        /// <summary>Check if current operating point is stable.</summary>
        public void UpdateOperatingPoint(float rpm, float depthMM)
        {
            CurrentRPM = rpm;
            CurrentDepth = depthMM;
            float limit = CalculateStabilityLimit(rpm);
            IsStable = depthMM < limit;
        }

        /// <summary>
        /// Render the lobe surface as a color-coded 2D texture overlay.
        /// Green = stable pocket (large margin), Yellow = near boundary, Red = unstable.
        /// </summary>
        private void RenderLobeSurface()
        {
            if (lobePredictor == null) return;

            lobePredictor.ComputeLobeSurface(minRPM, maxRPM, 0f, maxDepth, lobeSurfaceResolution);
            var (surface, rpms, depths) = lobePredictor.GetLobeSurfaceData();
            if (surface == null) return;

            int res = lobeSurfaceResolution;

            if (lobeSurfaceTexture == null || lobeSurfaceTexture.width != res)
            {
                lobeSurfaceTexture = new Texture2D(res, res, TextureFormat.RGBA32, false);
                lobeSurfaceTexture.filterMode = FilterMode.Bilinear;
                lobeSurfaceTexture.wrapMode = TextureWrapMode.Clamp;
            }

            for (int x = 0; x < res; x++)
            {
                for (int y = 0; y < res; y++)
                {
                    float margin = surface[x, y]; // positive = stable, negative = unstable
                    float depth = depths[y];
                    float normalizedMargin = depth > 0.001f ? margin / depth : 1f;

                    Color c;
                    if (normalizedMargin > 0.2f)
                        c = stableColor;            // Green: stable pocket, large margin
                    else if (normalizedMargin > 0f)
                        c = nearBoundaryColor;      // Yellow: near boundary
                    else
                        c = unstableColor;          // Red: unstable

                    lobeSurfaceTexture.SetPixel(x, y, c);
                }
            }
            lobeSurfaceTexture.Apply();

            // Create or update the quad to display the texture
            if (lobeSurfaceQuad == null)
            {
                lobeSurfaceQuad = GameObject.CreatePrimitive(PrimitiveType.Quad);
                lobeSurfaceQuad.name = "LobeSurfaceOverlay";
                lobeSurfaceQuad.transform.SetParent(transform, false);
                lobeSurfaceQuad.transform.localPosition = Vector3.zero;
                lobeSurfaceQuad.transform.localScale = Vector3.one;
                // Remove collider
                var col = lobeSurfaceQuad.GetComponent<Collider>();
                if (col != null) Destroy(col);
            }

            var renderer = lobeSurfaceQuad.GetComponent<Renderer>();
            if (renderer != null)
            {
                var shader = Shader.Find("Universal Render Pipeline/Unlit")
                          ?? Shader.Find("Sprites/Default")
                          ?? Shader.Find("Hidden/InternalErrorShader");
                if (shader != null)
                {
                    renderer.material = new Material(shader);
                    renderer.material.mainTexture = lobeSurfaceTexture;
                }
            }
        }

        /// <summary>
        /// Called periodically to refresh the lobe surface when wear changes.
        /// </summary>
        void Update()
        {
            if (lobePredictor != null)
            {
                // Re-render lobe surface when predictor data changes
                var (surface, _, _) = lobePredictor.GetLobeSurfaceData();
                if (surface == null)
                {
                    RenderLobeSurface();
                }
            }
        }

        /// <summary>
        /// Refresh the lobe surface visualization. Call this after wear updates.
        /// </summary>
        public void RefreshLobeSurface()
        {
            RenderLobeSurface();
        }

        /// <summary>
        /// Get the stability margin color for an operating point.
        /// </summary>
        private Color GetStabilityColor(float rpm, float depth)
        {
            float limit = CalculateStabilityLimit(rpm);
            float margin = limit - depth;
            float normalizedMargin = depth > 0.001f ? margin / depth : 1f;

            if (normalizedMargin > 0.2f) return stableColor;
            if (normalizedMargin > 0f) return nearBoundaryColor;
            return unstableColor;
        }

        /// <summary>
        /// Draw current operating point and lookahead points as debug visualization.
        /// Current point as yellow dot; lookahead as trail of colored dots.
        /// Green = stable pocket (large margin), Yellow = near boundary, Red = unstable.
        /// </summary>
        void OnDrawGizmos()
        {
            // Draw current operating point
            if (CurrentRPM > 0 && CurrentDepth > 0)
            {
                float xNorm = Mathf.InverseLerp(minRPM, maxRPM, CurrentRPM);
                float yNorm = CurrentDepth / maxDepth;
                Vector3 worldPos = transform.TransformPoint(new Vector3(xNorm - 0.5f, yNorm - 0.5f, 0));

                Gizmos.color = operatingPointColor;
                Gizmos.DrawSphere(worldPos, 0.008f);
            }

            // Draw lookahead points as a trail with stability coloring
            if (lookaheadPoints.Count == 0) return;

            for (int i = 0; i < lookaheadPoints.Count; i++)
            {
                float rpm = lookaheadPoints[i].x;
                float depth = lookaheadPoints[i].y;

                // Normalize to chart space
                float xNorm = Mathf.InverseLerp(minRPM, maxRPM, rpm);
                float yNorm = depth / maxDepth;
                Vector3 worldPos = transform.TransformPoint(new Vector3(xNorm - 0.5f, yNorm - 0.5f, 0));

                // Color by stability margin: green (safe), yellow (near boundary), red (unstable)
                Gizmos.color = GetStabilityColor(rpm, depth);

                // Trail dots get smaller as they get further from current point
                float size = Mathf.Lerp(0.006f, 0.003f, (float)i / Mathf.Max(1, lookaheadPoints.Count - 1));
                Gizmos.DrawSphere(worldPos, size);

                // Draw connecting line between trail points
                if (i > 0)
                {
                    float prevRpm = lookaheadPoints[i - 1].x;
                    float prevDepth = lookaheadPoints[i - 1].y;
                    float prevX = Mathf.InverseLerp(minRPM, maxRPM, prevRpm);
                    float prevY = prevDepth / maxDepth;
                    Vector3 prevPos = transform.TransformPoint(new Vector3(prevX - 0.5f, prevY - 0.5f, 0));
                    Gizmos.DrawLine(prevPos, worldPos);
                }
            }
        }
    }
}
