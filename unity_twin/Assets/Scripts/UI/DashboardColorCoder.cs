using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;
using RosMessageTypes.Miracle;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Monitors live machine data and applies severity-based colors to dashboard labels.
    /// Attaches to the same UIDocument as the main dashboard. Uses configurable thresholds
    /// for force, temperature, wear, and OEE metrics.
    /// </summary>
    public class DashboardColorCoder : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private MachineStateEventSO machineStateEvent;
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;

        [Header("Force Thresholds (N)")]
        [SerializeField] private float forceWarning = 120f;
        [SerializeField] private float forceCritical = 160f;

        [Header("Temperature Thresholds (C)")]
        [SerializeField] private float tempWarning = 60f;
        [SerializeField] private float tempCritical = 85f;

        [Header("Wear Thresholds (mm)")]
        [SerializeField] private float wearWarning = 0.20f;
        [SerializeField] private float wearCritical = 0.27f;

        [Header("OEE Thresholds (0-1)")]
        [SerializeField] private float oeeCritical = 0.5f;
        [SerializeField] private float oeeWarning = 0.7f;

        private Label _forceXLabel, _forceYLabel, _forceZLabel;
        private Label _spindleTempLabel, _toolTempLabel;
        private Label _wearVbLabel;
        private Label _oeeLabel, _availLabel, _perfLabel, _qualLabel;

        private readonly Color _normalColor = new(0.78f, 0.82f, 0.88f);
        private readonly Color _warningColor = new(0.98f, 0.8f, 0.08f);
        private readonly Color _criticalColor = new(0.95f, 0.25f, 0.25f);
        private readonly Color _goodColor = new(0.2f, 0.9f, 0.4f);

        void OnEnable()
        {
            var root = uiDocument != null ? uiDocument.rootVisualElement : null;
            if (root == null) return;

            _forceXLabel = root.Q<Label>("force-x-value");
            _forceYLabel = root.Q<Label>("force-y-value");
            _forceZLabel = root.Q<Label>("force-z-value");
            _spindleTempLabel = root.Q<Label>("spindle-temp-value");
            _toolTempLabel = root.Q<Label>("tool-temp-value");
            _wearVbLabel = root.Q<Label>("wear-vb-value");
            _oeeLabel = root.Q<Label>("oee-value");
            _availLabel = root.Q<Label>("availability-value");
            _perfLabel = root.Q<Label>("performance-value");
            _qualLabel = root.Q<Label>("quality-value");

            if (machineStateEvent != null) machineStateEvent.Register(OnMachineState);
            if (cuttingStateEvent != null) cuttingStateEvent.Register(OnCuttingState);
        }

        void OnDisable()
        {
            if (machineStateEvent != null) machineStateEvent.Unregister(OnMachineState);
            if (cuttingStateEvent != null) cuttingStateEvent.Unregister(OnCuttingState);
        }

        private void OnMachineState(MachineStateMsg msg)
        {
            // MachineStateMsg carries spindle_load but no direct force/temp fields.
            // Spindle temp label colored from spindle load as proxy (temp data arrives via CuttingState).
            ColorLabel(_spindleTempLabel, (float)msg.spindle_load, forceWarning, forceCritical);
        }

        private void OnCuttingState(CuttingStateData data)
        {
            ColorLabel(_forceXLabel, Mathf.Abs(data.forceFx), forceWarning, forceCritical);
            ColorLabel(_forceYLabel, Mathf.Abs(data.forceFy), forceWarning, forceCritical);
            ColorLabel(_forceZLabel, Mathf.Abs(data.forceFz), forceWarning, forceCritical);
            ColorLabel(_toolTempLabel, data.toolTemperature, tempWarning, tempCritical);
            ColorLabel(_wearVbLabel, data.flankWearVB, wearWarning, wearCritical);
        }

        private void ColorLabel(Label label, float value, float warning, float critical)
        {
            if (label == null) return;
            label.style.color = value >= critical ? _criticalColor
                              : value >= warning ? _warningColor
                              : _normalColor;
        }

        /// <summary>
        /// Call from the OEE calculator to color OEE-related labels.
        /// Values are expected in 0-1 range.
        /// </summary>
        public void UpdateOEE(float oee, float avail, float perf, float qual)
        {
            ColorLabelInverse(_oeeLabel, oee, oeeWarning, oeeCritical);
            ColorLabelInverse(_availLabel, avail, oeeWarning, oeeCritical);
            ColorLabelInverse(_perfLabel, perf, oeeWarning, oeeCritical);
            ColorLabelInverse(_qualLabel, qual, oeeWarning, oeeCritical);
        }

        /// <summary>Inverse color: low values are bad (red below critical, yellow below warning, green above).</summary>
        private void ColorLabelInverse(Label label, float value, float warning, float critical)
        {
            if (label == null) return;
            label.style.color = value <= critical ? _criticalColor
                              : value <= warning ? _warningColor
                              : _goodColor;
        }
    }
}
