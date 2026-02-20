using UnityEngine;
using MiracleTwin.Core;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// 3D visual indicator for tool wear on the end mill.
    /// Shows wear progression as a darkened/roughened area on the tool flank.
    /// Also updates a UI progress bar.
    /// </summary>
    public class WearIndicator : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private Renderer toolRenderer;

        [Header("Visual Settings")]
        [SerializeField] private Color freshToolColor = new(0.8f, 0.8f, 0.85f, 1f);
        [SerializeField] private Color wornToolColor = new(0.3f, 0.25f, 0.2f, 1f);
        [SerializeField] private float wearBandMaxHeight = 0.003f; // 3mm max visual wear band

        public float CurrentWearPercentage { get; private set; }
        public float CurrentVB { get; private set; }
        public string RecommendedAction { get; private set; } = "CONTINUE";

        private MaterialPropertyBlock propBlock;
        private static readonly int WearProp = Shader.PropertyToID("_WearAmount");
        private static readonly int WearColorProp = Shader.PropertyToID("_WearColor");

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
            CurrentWearPercentage = state.wearPercentage;
            CurrentVB = state.flankWearVB;

            if (CurrentVB < 0.10f) RecommendedAction = "CONTINUE";
            else if (CurrentVB < 0.15f) RecommendedAction = "MONITOR";
            else if (CurrentVB < 0.25f) RecommendedAction = "PLAN_REPLACEMENT";
            else RecommendedAction = "REPLACE_NOW";
        }

        void Update()
        {
            if (toolRenderer == null) return;

            float wearFraction = Mathf.Clamp01(CurrentWearPercentage / 100f);

            toolRenderer.GetPropertyBlock(propBlock);
            propBlock.SetFloat(WearProp, wearFraction);
            propBlock.SetColor(WearColorProp, Color.Lerp(freshToolColor, wornToolColor, wearFraction));
            toolRenderer.SetPropertyBlock(propBlock);

            // Scale wear band visual
            float bandHeight = wearFraction * wearBandMaxHeight;
            var scale = transform.localScale;
            scale.y = Mathf.Max(0.001f, bandHeight);
            transform.localScale = scale;
        }
    }
}
