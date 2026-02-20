using System;
using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Central time authority for the MIRACLE Digital Twin.
    /// Supports Real-Time (1:1 with wall clock), Accelerated (up to 100x),
    /// Replay (driven by ReplayController), and Paused modes.
    ///
    /// Features:
    /// - FormatSimTime() returns "HH:MM:SS.f" formatted string
    /// - TotalElapsedWallTime tracks real wall-clock time since startup
    /// - SpeedMultiplier properly clamped to [0.1, 100]
    /// </summary>
    public class SimulationClock : MonoBehaviour
    {
        public enum Mode { RealTime, Accelerated, Replay, Paused }

        public static SimulationClock Instance { get; private set; }

        [SerializeField] private Mode initialMode = Mode.Paused;
        [SerializeField, Range(0.1f, 100f)] private float speedMultiplier = 1f;

        public Mode CurrentMode { get; private set; }
        public double SimTime { get; private set; }
        public double DeltaTime { get; private set; }
        public float SpeedMultiplier
        {
            get => speedMultiplier;
            set => speedMultiplier = Mathf.Clamp(value, 0.1f, 100f);
        }
        public bool IsPaused => CurrentMode == Mode.Paused;
        public bool IsRunning => CurrentMode != Mode.Paused;

        /// <summary>
        /// Total wall-clock time elapsed since the SimulationClock was initialized,
        /// measured in real seconds (unaffected by speed multiplier or pause state).
        /// </summary>
        public double TotalElapsedWallTime =>
            Time.realtimeSinceStartupAsDouble - wallTimeAtStart;

        /// <summary>
        /// Wall-clock time elapsed since the last reset, measured in real seconds.
        /// </summary>
        public double WallTimeSinceReset =>
            Time.realtimeSinceStartupAsDouble - wallTimeAtLastReset;

        public event Action<double> OnTick;
        public event Action<Mode> OnModeChanged;
        public event Action OnReset;

        private double wallTimeAtStart;
        private double wallTimeAtLastReset;

        void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;
            DontDestroyOnLoad(transform.root.gameObject);
            CurrentMode = initialMode;
            wallTimeAtStart = Time.realtimeSinceStartupAsDouble;
            wallTimeAtLastReset = wallTimeAtStart;
        }

        void Update()
        {
            switch (CurrentMode)
            {
                case Mode.RealTime:
                    DeltaTime = Time.unscaledDeltaTime;
                    SimTime += DeltaTime;
                    OnTick?.Invoke(SimTime);
                    break;

                case Mode.Accelerated:
                    DeltaTime = Time.unscaledDeltaTime * speedMultiplier;
                    SimTime += DeltaTime;
                    OnTick?.Invoke(SimTime);
                    break;

                case Mode.Replay:
                    // DeltaTime and SimTime driven externally by ReplayController
                    OnTick?.Invoke(SimTime);
                    break;

                case Mode.Paused:
                    DeltaTime = 0;
                    break;
            }
        }

        public void SetMode(Mode newMode)
        {
            if (newMode == CurrentMode) return;
            var oldMode = CurrentMode;
            CurrentMode = newMode;

            if (newMode == Mode.RealTime)
                wallTimeAtStart = Time.realtimeSinceStartupAsDouble - SimTime;

            OnModeChanged?.Invoke(newMode);
            Debug.Log($"[SimulationClock] Mode changed: {oldMode} -> {newMode}");
        }

        public void Play() => SetMode(Mode.RealTime);
        public void Pause() => SetMode(Mode.Paused);
        public void TogglePause() => SetMode(IsPaused ? Mode.RealTime : Mode.Paused);

        public void SetAccelerated(float multiplier)
        {
            SpeedMultiplier = multiplier;
            SetMode(Mode.Accelerated);
        }

        /// <summary>Called by ReplayController to set time directly.</summary>
        public void SeekTo(double time)
        {
            SimTime = time;
            DeltaTime = 0;
        }

        /// <summary>Called by ReplayController to advance replay time.</summary>
        public void AdvanceReplay(double dt)
        {
            DeltaTime = dt;
            SimTime += dt;
        }

        public void ResetSimulation()
        {
            SimTime = 0;
            DeltaTime = 0;
            wallTimeAtLastReset = Time.realtimeSinceStartupAsDouble;
            SetMode(Mode.Paused);
            OnReset?.Invoke();
            Debug.Log("[SimulationClock] Simulation reset");
        }

        /// <summary>
        /// Format simulation time as "HH:MM:SS.f" where f is tenths of a second.
        /// </summary>
        public string FormatSimTime()
        {
            var ts = TimeSpan.FromSeconds(SimTime);
            return $"{ts.Hours:D2}:{ts.Minutes:D2}:{ts.Seconds:D2}.{ts.Milliseconds / 100:D1}";
        }

        /// <summary>
        /// Format wall-clock elapsed time as "HH:MM:SS".
        /// </summary>
        public string FormatWallTime()
        {
            var ts = TimeSpan.FromSeconds(TotalElapsedWallTime);
            return $"{ts.Hours:D2}:{ts.Minutes:D2}:{ts.Seconds:D2}";
        }

        void OnDestroy()
        {
            if (Instance == this) Instance = null;
        }
    }
}
