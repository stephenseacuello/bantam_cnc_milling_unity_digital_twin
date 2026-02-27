using UnityEngine;
using MiracleTwin.Core;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// GPU-instanced force arrows at the tool tip showing Fx (red), Fy (green), Fz (blue),
    /// and resultant force vector (white/magenta).
    /// Arrow length proportional to force magnitude: 1mm per 50N.
    ///
    /// Phase 3 additions:
    /// - 4th arrow for resultant force vector (white/magenta)
    /// - Peak force tracking with exponential decay
    /// - Force limit warnings with pulsing red indicator
    /// </summary>
    public class ForceArrowRenderer : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private Mesh arrowMesh;
        [SerializeField] private Material arrowMaterial;

        [Header("Settings")]
        [SerializeField] private float scaleFactor = 0.001f / 50f; // 1mm per 50N
        [SerializeField] private float minimumForce = 5f;          // Don't show below this
        [SerializeField] private bool showPeakForces = false;

        [Header("Peak Tracking")]
        [SerializeField] private float peakDecayRate = 0.95f;      // Per-frame exponential decay
        [SerializeField] private float forceLimit = 500f;           // Force limit for warning (N)

        private static readonly Color FxColor = new(1f, 0.2f, 0.2f, 0.9f);
        private static readonly Color FyColor = new(0.2f, 1f, 0.2f, 0.9f);
        private static readonly Color FzColor = new(0.3f, 0.3f, 1f, 0.9f);
        private static readonly Color ResultantColor = new(1f, 0.4f, 1f, 0.9f);      // Magenta/white
        private static readonly Color PeakColor = new(1f, 1f, 1f, 0.4f);              // Translucent white for peak ghosts
        private static readonly Color ForceLimitColor = new(1f, 0f, 0f, 1f);          // Red for limit warnings

        // 4 main arrows (Fx, Fy, Fz, Resultant) + 3 peak arrows (Fx, Fy, Fz peaks)
        private Matrix4x4[] matrices = new Matrix4x4[7];
        private MaterialPropertyBlock[] propBlocks = new MaterialPropertyBlock[7];
        private Vector3 toolPosition;
        private Vector3 forces;
        private Vector3 peakForces;
        private bool isActive;

        // Force limit warning state
        private bool forceLimitExceeded;
        private float pulseTimer;

        void Start()
        {
            for (int i = 0; i < propBlocks.Length; i++)
                propBlocks[i] = new MaterialPropertyBlock();

            // Component arrows
            propBlocks[0].SetColor("_Color", FxColor);
            propBlocks[1].SetColor("_Color", FyColor);
            propBlocks[2].SetColor("_Color", FzColor);

            // Resultant arrow (magenta/white)
            propBlocks[3].SetColor("_Color", ResultantColor);

            // Peak force ghost arrows (translucent versions of component colors)
            propBlocks[4].SetColor("_Color", new Color(FxColor.r, FxColor.g, FxColor.b, 0.3f));
            propBlocks[5].SetColor("_Color", new Color(FyColor.r, FyColor.g, FyColor.b, 0.3f));
            propBlocks[6].SetColor("_Color", new Color(FzColor.r, FzColor.g, FzColor.b, 0.3f));
        }

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

        private void OnCuttingState(CuttingStateData state)
        {
            isActive = state.isCutting;
            toolPosition = state.toolPosition;
            forces = new Vector3(state.forceFx, state.forceFy, state.forceFz);

            // Update peak forces: track the maximum seen, decayed over time
            peakForces.x = Mathf.Max(peakForces.x, Mathf.Abs(forces.x));
            peakForces.y = Mathf.Max(peakForces.y, Mathf.Abs(forces.y));
            peakForces.z = Mathf.Max(peakForces.z, Mathf.Abs(forces.z));
        }

        void Update()
        {
            if (!isActive || arrowMesh == null || arrowMaterial == null) return;

            // Decay peak forces each frame
            peakForces *= peakDecayRate;

            // Draw component force arrows
            DrawForceArrow(0, Vector3.right, forces.x);
            DrawForceArrow(1, Vector3.up, forces.y);
            DrawForceArrow(2, Vector3.forward, forces.z);

            // Draw resultant force vector (white/magenta)
            DrawResultantArrow();

            // Draw peak force ghost arrows when enabled
            if (showPeakForces)
            {
                DrawPeakArrow(4, Vector3.right, peakForces.x, forces.x);
                DrawPeakArrow(5, Vector3.up, peakForces.y, forces.y);
                DrawPeakArrow(6, Vector3.forward, peakForces.z, forces.z);
            }

            // Force limit warning: pulse red when any component exceeds limit
            UpdateForceLimitWarning();
        }

        private void DrawForceArrow(int index, Vector3 direction, float magnitude)
        {
            float absMag = Mathf.Abs(magnitude);
            if (absMag < minimumForce) return;

            float length = absMag * scaleFactor;
            Vector3 dir = magnitude >= 0 ? direction : -direction;

            matrices[index] = Matrix4x4.TRS(
                toolPosition,
                Quaternion.LookRotation(dir),
                new Vector3(length * 0.1f, length * 0.1f, length)
            );

            Graphics.DrawMesh(arrowMesh, matrices[index], arrowMaterial, 0, null, 0, propBlocks[index]);
        }

        private void DrawResultantArrow()
        {
            float resultantMag = forces.magnitude;
            if (resultantMag < minimumForce) return;

            float length = resultantMag * scaleFactor;
            Vector3 dir = forces.normalized;

            // Avoid degenerate rotation when force is near zero
            if (dir.sqrMagnitude < 0.001f) return;

            matrices[3] = Matrix4x4.TRS(
                toolPosition,
                Quaternion.LookRotation(dir),
                new Vector3(length * 0.15f, length * 0.15f, length) // Slightly thicker than components
            );

            Graphics.DrawMesh(arrowMesh, matrices[3], arrowMaterial, 0, null, 0, propBlocks[3]);
        }

        private void DrawPeakArrow(int index, Vector3 direction, float peakMagnitude, float currentForce)
        {
            // Only draw peak ghost if it exceeds the current force (showing the "remembered" peak)
            if (peakMagnitude < minimumForce) return;
            float currentAbs = Mathf.Abs(currentForce);
            if (peakMagnitude <= currentAbs * 1.05f) return; // Don't draw if peak is same as current

            float length = peakMagnitude * scaleFactor;
            Vector3 dir = currentForce >= 0 ? direction : -direction;

            matrices[index] = Matrix4x4.TRS(
                toolPosition,
                Quaternion.LookRotation(dir),
                new Vector3(length * 0.06f, length * 0.06f, length) // Thinner than live arrows
            );

            Graphics.DrawMesh(arrowMesh, matrices[index], arrowMaterial, 0, null, 0, propBlocks[index]);
        }

        private void UpdateForceLimitWarning()
        {
            float maxComponent = Mathf.Max(Mathf.Abs(forces.x), Mathf.Abs(forces.y), Mathf.Abs(forces.z));
            forceLimitExceeded = maxComponent > forceLimit;

            if (forceLimitExceeded)
            {
                // Pulse the resultant arrow red as a warning
                pulseTimer += Time.deltaTime * 6f; // ~3 pulses per second
                float pulse = (Mathf.Sin(pulseTimer) + 1f) * 0.5f; // 0..1

                Color warningColor = Color.Lerp(ResultantColor, ForceLimitColor, pulse);
                propBlocks[3].SetColor("_Color", warningColor);

                // Also pulse the component arrows that exceed the limit
                if (Mathf.Abs(forces.x) > forceLimit)
                    propBlocks[0].SetColor("_Color", Color.Lerp(FxColor, ForceLimitColor, pulse));
                else
                    propBlocks[0].SetColor("_Color", FxColor);

                if (Mathf.Abs(forces.y) > forceLimit)
                    propBlocks[1].SetColor("_Color", Color.Lerp(FyColor, ForceLimitColor, pulse));
                else
                    propBlocks[1].SetColor("_Color", FyColor);

                if (Mathf.Abs(forces.z) > forceLimit)
                    propBlocks[2].SetColor("_Color", Color.Lerp(FzColor, ForceLimitColor, pulse));
                else
                    propBlocks[2].SetColor("_Color", FzColor);
            }
            else
            {
                pulseTimer = 0f;
                propBlocks[0].SetColor("_Color", FxColor);
                propBlocks[1].SetColor("_Color", FyColor);
                propBlocks[2].SetColor("_Color", FzColor);
                propBlocks[3].SetColor("_Color", ResultantColor);
            }
        }

        /// <summary>Whether any force component currently exceeds the force limit.</summary>
        public bool IsForceLimitExceeded => forceLimitExceeded;

        /// <summary>Current peak forces (with decay applied).</summary>
        public Vector3 PeakForces => peakForces;

        public void SetVisible(bool visible)
        {
            enabled = visible;
        }
    }
}
