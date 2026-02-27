using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Emergency stop button — toggles between E-STOP and RESET.
    /// Double-click to trigger E-STOP; single click to reset when in E-STOP state.
    /// Also responds to InputManager Reset (R key) to sync button state.
    /// </summary>
    public class EStopButton : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;

        private Button estopBtn;
        private float lastClickTime;
        private const float DOUBLE_CLICK_THRESHOLD = 0.5f;
        private bool isInEStop;

        void Start()
        {
            if (uiDocument == null) return;
            estopBtn = uiDocument.rootVisualElement.Q<Button>("estop-btn");
            estopBtn?.RegisterCallback<ClickEvent>(_ => OnEStopClick());
        }

        void OnEnable()
        {
            if (InputManager.Instance != null)
                InputManager.Instance.OnReset += OnExternalReset;
        }

        void OnDisable()
        {
            if (InputManager.Instance != null)
                InputManager.Instance.OnReset -= OnExternalReset;
        }

        private void OnEStopClick()
        {
            if (isInEStop)
            {
                // Already in E-STOP: single click resets all controllers
                ResetEStop();
            }
            else
            {
                // Not in E-STOP: require double-click confirmation
                if (Time.unscaledTime - lastClickTime < DOUBLE_CLICK_THRESHOLD)
                {
                    TriggerEStop();
                }
                else
                {
                    Debug.Log("[EStop] Click again to confirm E-STOP");
                }
            }
            lastClickTime = Time.unscaledTime;
        }

        private void TriggerEStop()
        {
            Debug.LogWarning("[EStop] EMERGENCY STOP TRIGGERED");
            isInEStop = true;
            SimulationClock.Instance?.Pause();

            foreach (var cnc in Object.FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (cnc is CNC.ICNCController controller)
                    controller.EmergencyStop();
            }

            if (estopBtn != null)
                estopBtn.text = "RESET";
        }

        private void ResetEStop()
        {
            Debug.Log("[EStop] E-STOP reset — returning all machines to IDLE");
            isInEStop = false;
            SimulationClock.Instance?.Play();

            foreach (var cnc in Object.FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (cnc is CNC.ICNCController controller)
                    controller.Home();
            }

            if (estopBtn != null)
                estopBtn.text = "E-STOP";
        }

        private void OnExternalReset()
        {
            isInEStop = false;
            if (estopBtn != null)
                estopBtn.text = "E-STOP";
        }
    }
}
