using UnityEngine;
using MiracleTwin.Core;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// Heat map overlay on the workpiece surface showing temperature distribution.
    /// Color ramp: 20°C silver → 80°C yellow → 150°C orange → 200°C+ red.
    /// Driven by thermal model data through CuttingStateEvent.
    /// </summary>
    public class HeatMapOverlay : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private Material heatMapMaterial;
        [SerializeField] private Renderer workpieceRenderer;

        [Header("Temperature Range")]
        [SerializeField] private float minTemp = 20f;
        [SerializeField] private float maxTemp = 200f;
        [SerializeField] private float fadeSpeed = 2f;

        [Header("Color Ramp")]
        [SerializeField] private Gradient temperatureGradient;

        public bool IsEnabled { get; private set; } = true;
        public float CurrentMaxTemperature { get; private set; }

        private static readonly int TempProp = Shader.PropertyToID("_Temperature");
        private static readonly int MinTempProp = Shader.PropertyToID("_MinTemp");
        private static readonly int MaxTempProp = Shader.PropertyToID("_MaxTemp");
        private static readonly int HeatPosProp = Shader.PropertyToID("_HeatPosition");
        private static readonly int HeatRadiusProp = Shader.PropertyToID("_HeatRadius");

        private Vector3 lastCutPosition;
        private float currentDisplayTemp;
        private MaterialPropertyBlock propBlock;

        void Awake()
        {
            propBlock = new MaterialPropertyBlock();

            if (temperatureGradient == null)
            {
                temperatureGradient = new Gradient();
                temperatureGradient.SetKeys(
                    new GradientColorKey[] {
                        new(new Color(0.8f, 0.8f, 0.8f), 0f),    // Silver (ambient)
                        new(new Color(1f, 1f, 0.3f), 0.3f),       // Yellow (80°C)
                        new(new Color(1f, 0.5f, 0.1f), 0.65f),    // Orange (150°C)
                        new(new Color(1f, 0.1f, 0.1f), 1f),       // Red (200°C+)
                    },
                    new GradientAlphaKey[] {
                        new(0f, 0f),
                        new(0.5f, 0.2f),
                        new(0.8f, 0.5f),
                        new(1f, 1f),
                    }
                );
            }
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
            if (state.isCutting)
            {
                lastCutPosition = state.toolPosition;
                CurrentMaxTemperature = Mathf.Max(state.toolTemperature, state.interfaceTemperature);
            }
        }

        void Update()
        {
            if (!IsEnabled || workpieceRenderer == null) return;

            // Smooth temperature display
            float targetTemp = CurrentMaxTemperature;
            currentDisplayTemp = Mathf.Lerp(currentDisplayTemp, targetTemp, fadeSpeed * Time.deltaTime);

            // Update material properties
            workpieceRenderer.GetPropertyBlock(propBlock);
            propBlock.SetFloat(TempProp, currentDisplayTemp);
            propBlock.SetFloat(MinTempProp, minTemp);
            propBlock.SetFloat(MaxTempProp, maxTemp);
            propBlock.SetVector(HeatPosProp, lastCutPosition);
            propBlock.SetFloat(HeatRadiusProp, 0.01f);
            workpieceRenderer.SetPropertyBlock(propBlock);
        }

        public void Toggle()
        {
            IsEnabled = !IsEnabled;
        }
    }
}
