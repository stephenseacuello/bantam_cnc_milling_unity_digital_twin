using System;
using UnityEngine;
using UnityEngine.Profiling;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Singleton performance monitor tracking key runtime metrics.
    ///
    /// Metrics:
    /// - FPS: rolling average over a configurable window (default 60 samples)
    /// - GPU frame time: estimated from FrameTimingManager when available
    /// - Total draw calls: via Unity batch count from internal profiler
    /// - Memory usage: managed + native heap
    ///
    /// Fires OnLowFPS when FPS drops below the configurable threshold (default 30).
    /// Dashboard and other UI systems can read metrics from this singleton.
    /// </summary>
    public class PerformanceMonitor : MonoBehaviour
    {
        public static PerformanceMonitor Instance { get; private set; }

        [Header("Settings")]
        [SerializeField] private int fpsRollingWindowSize = 60;
        [SerializeField] private float lowFpsThreshold = 30f;
        [SerializeField] private float metricsUpdateInterval = 0.5f;

        // --- Public metrics for dashboard consumption ---

        /// <summary>Rolling average FPS over the configured window.</summary>
        public float AverageFPS { get; private set; }

        /// <summary>Current instantaneous FPS (1 / deltaTime).</summary>
        public float CurrentFPS { get; private set; }

        /// <summary>Minimum FPS observed in the current rolling window.</summary>
        public float MinFPS { get; private set; }

        /// <summary>Maximum FPS observed in the current rolling window.</summary>
        public float MaxFPS { get; private set; }

        /// <summary>Estimated GPU frame time in milliseconds (0 if unavailable).</summary>
        public float GpuFrameTimeMs { get; private set; }

        /// <summary>Estimated CPU frame time in milliseconds.</summary>
        public float CpuFrameTimeMs { get; private set; }

        /// <summary>Total batches (draw calls) in the last frame.</summary>
        public int TotalDrawCalls { get; private set; }

        /// <summary>Total triangles rendered in the last frame.</summary>
        public int TotalTriangles { get; private set; }

        /// <summary>Total SetPass calls in the last frame.</summary>
        public int SetPassCalls { get; private set; }

        /// <summary>Managed memory usage in MB.</summary>
        public float ManagedMemoryMB { get; private set; }

        /// <summary>Total allocated memory in MB.</summary>
        public float TotalAllocatedMemoryMB { get; private set; }

        /// <summary>True when FPS is currently below the low FPS threshold.</summary>
        public bool IsLowFPS { get; private set; }

        /// <summary>Fired when FPS drops below the threshold. Parameter: current average FPS.</summary>
        public event Action<float> OnLowFPS;

        // Rolling window for FPS calculation
        private float[] fpsBuffer;
        private int fpsBufferIndex;
        private int fpsBufferCount;

        // Frame timing
        private FrameTiming[] frameTimings;
        private float metricsTimer;
        private bool wasLowFps;

        void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;
            DontDestroyOnLoad(transform.root.gameObject);

            fpsBuffer = new float[fpsRollingWindowSize];
            frameTimings = new FrameTiming[1];
        }

        void Update()
        {
            float dt = Time.unscaledDeltaTime;
            if (dt <= 0) return;

            // Record instantaneous FPS into rolling window
            CurrentFPS = 1f / dt;
            CpuFrameTimeMs = dt * 1000f;

            fpsBuffer[fpsBufferIndex] = CurrentFPS;
            fpsBufferIndex = (fpsBufferIndex + 1) % fpsRollingWindowSize;
            fpsBufferCount = Mathf.Min(fpsBufferCount + 1, fpsRollingWindowSize);

            // Periodic full metrics update (not every frame to reduce overhead)
            metricsTimer += dt;
            if (metricsTimer >= metricsUpdateInterval)
            {
                metricsTimer = 0;
                UpdateMetrics();
            }
        }

        private void UpdateMetrics()
        {
            // Calculate rolling average, min, max FPS
            float sum = 0;
            float min = float.MaxValue;
            float max = float.MinValue;
            for (int i = 0; i < fpsBufferCount; i++)
            {
                sum += fpsBuffer[i];
                if (fpsBuffer[i] < min) min = fpsBuffer[i];
                if (fpsBuffer[i] > max) max = fpsBuffer[i];
            }
            AverageFPS = fpsBufferCount > 0 ? sum / fpsBufferCount : 0;
            MinFPS = fpsBufferCount > 0 ? min : 0;
            MaxFPS = fpsBufferCount > 0 ? max : 0;

            // GPU frame time via FrameTimingManager (available on supported platforms)
            FrameTimingManager.CaptureFrameTimings();
            uint count = FrameTimingManager.GetLatestTimings((uint)frameTimings.Length, frameTimings);
            if (count > 0)
            {
                GpuFrameTimeMs = (float)frameTimings[0].gpuFrameTime;
            }

            // Rendering stats (batches, triangles, setpass calls)
            // These are available through the internal UnityStats in Editor,
            // and through Profiler counters in builds.
#if UNITY_EDITOR
            TotalDrawCalls = UnityEditor.UnityStats.batches;
            TotalTriangles = UnityEditor.UnityStats.triangles;
            SetPassCalls = UnityEditor.UnityStats.setPassCalls;
#else
            // In builds, use Profiler.GetRuntimeMemorySize or custom counters.
            // Draw call counts are not directly available in builds without
            // FrameTimingManager or custom profiling; keep last known values.
#endif

            // Memory usage
            ManagedMemoryMB = GC.GetTotalMemory(false) / (1024f * 1024f);
            TotalAllocatedMemoryMB = Profiler.GetTotalAllocatedMemoryLong() / (1024f * 1024f);

            // Low FPS detection
            IsLowFPS = AverageFPS > 0 && AverageFPS < lowFpsThreshold;
            if (IsLowFPS && !wasLowFps)
            {
                Debug.LogWarning($"[PerformanceMonitor] LOW FPS WARNING: {AverageFPS:F1} FPS " +
                                 $"(threshold: {lowFpsThreshold} FPS). " +
                                 $"GPU: {GpuFrameTimeMs:F1}ms, CPU: {CpuFrameTimeMs:F1}ms, " +
                                 $"Draw calls: {TotalDrawCalls}");
                OnLowFPS?.Invoke(AverageFPS);
            }
            wasLowFps = IsLowFPS;
        }

        /// <summary>
        /// Get a formatted summary string for debug display.
        /// </summary>
        public string GetSummary()
        {
            return $"FPS: {AverageFPS:F0} (min:{MinFPS:F0} max:{MaxFPS:F0}) | " +
                   $"GPU: {GpuFrameTimeMs:F1}ms | CPU: {CpuFrameTimeMs:F1}ms | " +
                   $"Draws: {TotalDrawCalls} | Tris: {TotalTriangles} | " +
                   $"Mem: {TotalAllocatedMemoryMB:F0}MB";
        }

        void OnDestroy()
        {
            if (Instance == this) Instance = null;
        }
    }
}
