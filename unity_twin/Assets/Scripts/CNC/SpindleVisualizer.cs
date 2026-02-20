using UnityEngine;
using MiracleTwin.Core;

namespace MiracleTwin.CNC
{
    /// <summary>
    /// Visualizes spindle rotation with motion blur effect at high RPM.
    /// Rotates the collet/end mill assembly based on current spindle speed.
    /// </summary>
    public class SpindleVisualizer : MonoBehaviour
    {
        [SerializeField] private float motionBlurThreshold = 5000f;
        [SerializeField] private Material motionBlurMaterial;

        private Renderer endMillRenderer;
        private Material originalMaterial;
        private float currentRPM;
        private bool isBlurred;

        public float CurrentRPM
        {
            get => currentRPM;
            set => currentRPM = Mathf.Clamp(value, 0, 23000);
        }

        void Start()
        {
            endMillRenderer = GetComponentInChildren<Renderer>();
            if (endMillRenderer != null)
                originalMaterial = endMillRenderer.material;
        }

        void Update()
        {
            if (currentRPM > 0)
            {
                float degreesPerFrame = currentRPM * 360f / 60f * Time.deltaTime;
                transform.Rotate(Vector3.up, degreesPerFrame, Space.Self);
            }

            // Motion blur swap
            if (endMillRenderer != null && motionBlurMaterial != null)
            {
                bool shouldBlur = currentRPM > motionBlurThreshold;
                if (shouldBlur != isBlurred)
                {
                    endMillRenderer.material = shouldBlur ? motionBlurMaterial : originalMaterial;
                    isBlurred = shouldBlur;
                }
            }
        }
    }
}
