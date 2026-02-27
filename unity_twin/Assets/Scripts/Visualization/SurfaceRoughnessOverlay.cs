using UnityEngine;
using MiracleTwin.Core;
using MiracleTwin.Cutting;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// Color-maps surface roughness Ra values onto machined surfaces.
    /// Green = fine (Ra less than 0.4um), Yellow = medium, Red = rough (Ra greater than 3.2um).
    /// Updates as cutting progresses based on feed, speed, and wear.
    ///
    /// Phase 3 changes:
    /// - Removed hardcoded 2-flute assumption; uses ToolDefinition.fluteCount
    /// - Uses flankWearVB from CuttingStateData for wear-adjusted roughness
    /// - Uses CuttingStateData pre-calculated Ra/Rz/grade directly when available
    /// - Falls back to local calculation only when CuttingStateData fields are zero
    /// </summary>
    public class SurfaceRoughnessOverlay : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private Renderer workpieceRenderer;
        [SerializeField] private ToolDefinition activeTool;

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

            // Prefer pre-calculated values from CuttingSimulationManager (via CuttingStateData)
            // which already accounts for vibration amplitude, wear ploughing, and correct flute count
            if (state.surfaceRoughnessRa > 0)
            {
                CurrentRa = state.surfaceRoughnessRa;
                SurfaceGrade = state.surfaceGrade ?? SurfaceRoughnessModel.GetSurfaceGrade(CurrentRa);
            }
            else
            {
                // Fallback: calculate locally using ToolDefinition for correct flute count
                int fluteCount = activeTool != null ? activeTool.fluteCount : 2;
                float noseRadius = activeTool != null ? activeTool.noseRadius : 0.4f;

                float fz;
                if (state.feedRate > 0 && state.spindleRPM > 0 && fluteCount > 0)
                    fz = state.feedRate / (state.spindleRPM * fluteCount);
                else
                    fz = 0.05f;

                // Use flankWearVB from CuttingStateData for wear-adjusted roughness
                float flankWearVB = state.flankWearVB;

                CurrentRa = SurfaceRoughnessModel.CalculateRa(fz, noseRadius, 0f, flankWearVB);
                SurfaceGrade = SurfaceRoughnessModel.GetSurfaceGrade(CurrentRa);
            }

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

        /// <summary>
        /// Set the active tool definition at runtime (e.g., after a tool change).
        /// </summary>
        public void SetToolDefinition(ToolDefinition tool)
        {
            activeTool = tool;
        }
    }
}
