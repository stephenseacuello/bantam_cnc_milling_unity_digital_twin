using UnityEngine;

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

        public float CurrentRPM { get; set; }
        public float CurrentDepth { get; set; }
        public bool IsStable { get; private set; } = true;

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
    }
}
