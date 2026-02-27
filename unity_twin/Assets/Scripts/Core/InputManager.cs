using System;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.CNC;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Central input handler for the MIRACLE Digital Twin.
    /// Maps keyboard/mouse inputs to simulation actions.
    /// </summary>
    public class InputManager : MonoBehaviour
    {
        public static InputManager Instance { get; private set; }

        public event Action OnPlayPause;
        public event Action OnEStop;
        public event Action OnReset;
        public event Action OnToggleHUD;
        public event Action<float> OnSpeedChange;

        private float lastEscapeTime;
        private const float DOUBLE_TAP_THRESHOLD = 0.3f;

        void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;
        }

        void Update()
        {
            // Skip all keyboard shortcuts when a UIToolkit text field has focus
            if (IsTextInputFocused()) return;

            // Play/Pause toggle
            if (Input.GetKeyDown(KeyCode.Space))
            {
                OnPlayPause?.Invoke();
                SimulationClock.Instance?.TogglePause();
            }

            // Speed control
            if (Input.GetKeyDown(KeyCode.Plus) || Input.GetKeyDown(KeyCode.Equals))
            {
                float newSpeed = Mathf.Min(
                    (SimulationClock.Instance?.SpeedMultiplier ?? 1f) * 2f, 100f);
                SimulationClock.Instance?.SetAccelerated(newSpeed);
                OnSpeedChange?.Invoke(newSpeed);
            }
            if (Input.GetKeyDown(KeyCode.Minus))
            {
                float newSpeed = Mathf.Max(
                    (SimulationClock.Instance?.SpeedMultiplier ?? 1f) / 2f, 0.1f);
                if (newSpeed <= 1f)
                    SimulationClock.Instance?.Play();
                else
                    SimulationClock.Instance?.SetAccelerated(newSpeed);
                OnSpeedChange?.Invoke(newSpeed);
            }

            // E-Stop (double-tap Escape)
            if (Input.GetKeyDown(KeyCode.Escape))
            {
                if (Time.unscaledTime - lastEscapeTime < DOUBLE_TAP_THRESHOLD)
                {
                    OnEStop?.Invoke();
                    TriggerEStopAll();
                    Debug.LogWarning("[InputManager] E-STOP triggered!");
                }
                lastEscapeTime = Time.unscaledTime;
            }

            // Reset simulation (also clears E-STOP state on all controllers)
            if (Input.GetKeyDown(KeyCode.R) && !Input.GetKey(KeyCode.LeftControl))
            {
                OnReset?.Invoke();
                ResetAllControllers();
                SimulationClock.Instance?.ResetSimulation();
            }

            // Toggle HUD
            if (Input.GetKeyDown(KeyCode.H))
            {
                OnToggleHUD?.Invoke();
            }
        }

        /// <summary>
        /// Returns true if a UIToolkit TextField has keyboard focus.
        /// Used to suppress keyboard shortcuts while the user is typing.
        /// </summary>
        public static bool IsTextInputFocused()
        {
            foreach (var doc in FindObjectsByType<UIDocument>(FindObjectsSortMode.None))
            {
                if (doc.rootVisualElement == null) continue;
                var focused = doc.rootVisualElement.focusController?.focusedElement as VisualElement;
                for (var ve = focused; ve != null; ve = ve.parent)
                {
                    if (ve is TextField) return true;
                }
            }
            return false;
        }

        private static void TriggerEStopAll()
        {
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (mb is ICNCController controller)
                    controller.EmergencyStop();
            }
        }

        private static void ResetAllControllers()
        {
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (mb is ICNCController controller)
                    controller.Home();
            }
        }

        void OnDestroy()
        {
            if (Instance == this) Instance = null;
        }
    }
}
