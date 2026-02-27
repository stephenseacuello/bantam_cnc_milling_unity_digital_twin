using UnityEngine;
using UnityEngine.VFX;
using MiracleTwin.Core;
using MiracleTwin.Cutting;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// Drives VFX Graph chip particle parameters based on cutting state.
    /// Spawn rate proportional to MRR, velocity from chip formation model,
    /// curl radius and particle size from material properties.
    ///
    /// Phase 3 additions:
    /// - MaterialDatabase reference for material-specific particle tuning
    /// - Curl radius adjusted by material density (denser = tighter curl)
    /// - Particle size scaled by material density and chip thickness ratio
    /// - Chip color derived from material properties (aluminum silver, titanium grey, etc.)
    /// </summary>
    public class ChipParticleController : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private VisualEffect chipVFX;

        [Header("Material")]
        [SerializeField] private MaterialType workpieceMaterial = MaterialType.Al6061T6;

        [Header("Particle Settings")]
        [SerializeField] private float spawnRateMultiplier = 100f;
        [SerializeField] private float velocityMultiplier = 0.01f;
        [SerializeField] private float minParticleSize = 0.0005f;
        [SerializeField] private float maxParticleSize = 0.002f;

        // VFX Graph property names
        private static readonly int SpawnRateProp = Shader.PropertyToID("SpawnRate");
        private static readonly int VelocityProp = Shader.PropertyToID("ChipVelocity");
        private static readonly int PositionProp = Shader.PropertyToID("SpawnPosition");
        private static readonly int DirectionProp = Shader.PropertyToID("ChipDirection");
        private static readonly int SizeProp = Shader.PropertyToID("ChipSize");
        private static readonly int CurlProp = Shader.PropertyToID("CurlRadius");
        private static readonly int ColorProp = Shader.PropertyToID("ChipColor");

        private bool isCutting;
        private Vector3 toolPosition;
        private float mrr;
        private float chipVelocity;
        private float chipCurlRadius;
        private float chipThicknessRatio;
        private Vector3 toolDirection;

        // Material-derived properties (cached on start)
        private float materialDensity;
        private float referenceDensity;         // Al6061-T6 as baseline reference
        private Color materialChipColor;
        private float densityScaleFactor;       // ratio of material density to reference

        /// <summary>Supported workpiece material types.</summary>
        public enum MaterialType
        {
            Al6061T6,
            Ti6Al4V,
            Custom
        }

        void Awake()
        {
            ApplyMaterialProperties();
        }

        /// <summary>
        /// Apply material-specific properties for particle tuning.
        /// Uses MaterialDatabase static definitions.
        /// </summary>
        private void ApplyMaterialProperties()
        {
            referenceDensity = MaterialDatabase.Al6061T6.density; // 2700 kg/m3

            MaterialDatabase.MaterialProperties mat;
            switch (workpieceMaterial)
            {
                case MaterialType.Ti6Al4V:
                    mat = MaterialDatabase.Ti6Al4V;
                    materialChipColor = new Color(0.55f, 0.55f, 0.60f, 1f); // Darker titanium grey
                    break;
                case MaterialType.Al6061T6:
                default:
                    mat = MaterialDatabase.Al6061T6;
                    materialChipColor = new Color(0.75f, 0.75f, 0.78f, 1f); // Aluminum silver
                    break;
            }

            materialDensity = mat.density;
            densityScaleFactor = materialDensity / referenceDensity;

            // Higher density materials produce smaller, tighter-curled chips
            // Scale particle size inversely with density ratio
            float sizeScale = 1f / Mathf.Sqrt(densityScaleFactor);
            minParticleSize *= sizeScale;
            maxParticleSize *= sizeScale;
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
            isCutting = state.isCutting;
            toolPosition = state.toolPosition;
            toolDirection = state.toolDirection;
            mrr = state.mrr;
            chipVelocity = state.chipVelocity;
            chipCurlRadius = state.chipCurlRadius;
            chipThicknessRatio = state.chipThicknessRatio;
        }

        void Update()
        {
            if (chipVFX == null) return;

            if (isCutting && mrr > 0)
            {
                float spawnRate = mrr * spawnRateMultiplier;
                chipVFX.SetFloat(SpawnRateProp, spawnRate);
                chipVFX.SetVector3(PositionProp, toolPosition);
                chipVFX.SetVector3(DirectionProp, -toolDirection);
                chipVFX.SetFloat(VelocityProp, chipVelocity * velocityMultiplier);

                // Material density-adjusted particle size:
                // Denser materials produce smaller chips at same MRR
                // Also scale with chip thickness ratio when available
                float thicknessScale = chipThicknessRatio > 0 ? Mathf.Clamp(chipThicknessRatio, 0.5f, 2f) : 1f;
                float adjustedMinSize = minParticleSize * thicknessScale;
                float adjustedMaxSize = maxParticleSize * thicknessScale;
                chipVFX.SetFloat(SizeProp, Random.Range(adjustedMinSize, adjustedMaxSize));

                // Material density-adjusted curl radius:
                // Denser materials produce tighter curls (smaller radius)
                // Reference curl from chip formation model, scaled inversely by density ratio
                float densityAdjustedCurl = chipCurlRadius / densityScaleFactor;
                chipVFX.SetFloat(CurlProp, densityAdjustedCurl / 1000f); // mm to m

                // Material-specific chip color
                chipVFX.SetVector4(ColorProp, materialChipColor);
            }
            else
            {
                chipVFX.SetFloat(SpawnRateProp, 0f);
            }
        }

        /// <summary>
        /// Change the workpiece material at runtime. Recalculates all
        /// material-derived particle properties.
        /// </summary>
        public void SetMaterial(MaterialType material)
        {
            workpieceMaterial = material;

            // Reset size bounds before re-applying (undo previous scaling)
            minParticleSize = 0.0005f;
            maxParticleSize = 0.002f;

            ApplyMaterialProperties();
        }

        /// <summary>Set a custom chip color (for non-standard materials).</summary>
        public void SetCustomChipColor(Color color)
        {
            materialChipColor = color;
        }

        public void SetEnabled(bool enabled)
        {
            this.enabled = enabled;
            if (!enabled && chipVFX != null)
                chipVFX.SetFloat(SpawnRateProp, 0f);
        }
    }
}
