using UnityEngine;
using MiracleTwin.Core;
using MiracleTwin.Cutting;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// Color-maps surface roughness Ra values onto machined surfaces.
    /// Green = fine (Ra < 0.4µm), Yellow = medium, Red = rough (Ra > 3.2µm).
    /// Updates as cutting progresses based on feed, speed, and wear.
    /// </summary>
    public class SurfaceRoughnessOverlay : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private Renderer workpieceRenderer;

        [Header("Settings")]
        [SerializeField] private bool isEnabled = false;
        [SerializeField] private float updateInterval = 0.5f;

        public float CurrentRa { get; private set; }
        public string SurfaceGrade { get; private set; } = "N/A";

        private MaterialPropertyBlock propBlock;
        private float lastUpdateTime;
        private static readonly int RoughnessProp = Shader.PropertyToID("_Roughness");

        void Awake()
        {
            propBlock = new MaterialPropertyBlock();
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
            if (!state.isCutting || !isEnabled) return;
            if (Time.time - lastUpdateTime < updateInterval) return;

            lastUpdateTime = Time.time;

            float fz = state.feedRate > 0 && state.spindleRPM > 0
                ? state.feedRate / (state.spindleRPM * 2f)  // Assume 2-flute
                : 0.05f;

            CurrentRa = SurfaceRoughnessModel.CalculateRa(fz, 0.4f);
            SurfaceGrade = SurfaceRoughnessModel.GetSurfaceGrade(CurrentRa);

            if (workpieceRenderer != null)
            {
                workpieceRenderer.GetPropertyBlock(propBlock);
                propBlock.SetFloat(RoughnessProp, CurrentRa);
                workpieceRenderer.SetPropertyBlock(propBlock);
            }
        }

        public void Toggle()
        {
            isEnabled = !isEnabled;
        }
    }
}
