using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Emergency stop button — calls TriggerEStop ROS2 service.
    /// Big red button with confirmation on double-click.
    /// </summary>
    public class EStopButton : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;

        private Button estopBtn;
        private float lastClickTime;
        private const float DOUBLE_CLICK_THRESHOLD = 0.5f;

        void Start()
        {
            if (uiDocument == null) return;
            estopBtn = uiDocument.rootVisualElement.Q<Button>("estop-btn");
            estopBtn?.RegisterCallback<ClickEvent>(_ => OnEStopClick());
        }

        private void OnEStopClick()
        {
            if (Time.unscaledTime - lastClickTime < DOUBLE_CLICK_THRESHOLD)
            {
                TriggerEStop();
            }
            else
            {
                Debug.Log("[EStop] Click again to confirm E-STOP");
            }
            lastClickTime = Time.unscaledTime;
        }

        private void TriggerEStop()
        {
            Debug.LogWarning("[EStop] EMERGENCY STOP TRIGGERED");
            SimulationClock.Instance?.Pause();

            // Locally trigger E-Stop on all CNC controllers in the scene
            foreach (var cnc in Object.FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (cnc is CNC.ICNCController controller)
                    controller.EmergencyStop();
            }
        }
    }
}
