using System;
using UnityEngine;

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
                    Debug.LogWarning("[InputManager] E-STOP triggered!");
                }
                lastEscapeTime = Time.unscaledTime;
            }

            // Reset simulation
            if (Input.GetKeyDown(KeyCode.R) && !Input.GetKey(KeyCode.LeftControl))
            {
                OnReset?.Invoke();
                SimulationClock.Instance?.ResetSimulation();
            }

            // Toggle HUD
            if (Input.GetKeyDown(KeyCode.H))
            {
                OnToggleHUD?.Invoke();
            }
        }

        void OnDestroy()
        {
            if (Instance == this) Instance = null;
        }
    }
}
