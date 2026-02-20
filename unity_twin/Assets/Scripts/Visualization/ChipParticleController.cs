using UnityEngine;
using UnityEngine.VFX;
using MiracleTwin.Core;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// Drives VFX Graph chip particle parameters based on cutting state.
    /// Spawn rate proportional to MRR, velocity from chip formation model,
    /// curl radius and particle size from material properties.
    /// </summary>
    public class ChipParticleController : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private VisualEffect chipVFX;

        [Header("Particle Settings")]
        [SerializeField] private float spawnRateMultiplier = 100f;
        [SerializeField] private float velocityMultiplier = 0.01f;
        [SerializeField] private float minParticleSize = 0.0005f;
        [SerializeField] private float maxParticleSize = 0.002f;
        [SerializeField] private Color chipColor = new(0.75f, 0.75f, 0.78f, 1f); // Aluminum silver

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
        private Vector3 toolDirection;

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
                chipVFX.SetFloat(SizeProp, Random.Range(minParticleSize, maxParticleSize));
                chipVFX.SetFloat(CurlProp, chipCurlRadius / 1000f);
                chipVFX.SetVector4(ColorProp, chipColor);
            }
            else
            {
                chipVFX.SetFloat(SpawnRateProp, 0f);
            }
        }

        public void SetEnabled(bool enabled)
        {
            this.enabled = enabled;
            if (!enabled && chipVFX != null)
                chipVFX.SetFloat(SpawnRateProp, 0f);
        }
    }
}
