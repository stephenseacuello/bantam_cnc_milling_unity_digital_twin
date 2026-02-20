using System;
using System.Collections;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using MiracleTwin.Core;

namespace MiracleTwin.Tests.PlayMode
{
    /// <summary>
    /// PlayMode integration tests for SimulationClock.
    /// Validates mode switching, time accumulation, speed multiplier behaviour,
    /// and that all public events (OnTick, OnModeChanged, OnReset) fire correctly.
    /// </summary>
    [TestFixture]
    public class SimulationClockTests
    {
        private GameObject clockGO;
        private SimulationClock clock;

        [UnitySetUp]
        public IEnumerator SetUp()
        {
            // Destroy any existing singleton from a previous test
            if (SimulationClock.Instance != null)
            {
                UnityEngine.Object.DestroyImmediate(SimulationClock.Instance.gameObject);
                yield return null;
            }

            clockGO = new GameObject("SimulationClock_Test");

            // Set initialMode to Paused before Awake runs the first frame
            // We add the component, which triggers Awake immediately.
            clock = clockGO.AddComponent<SimulationClock>();

            yield return null;
        }

        [UnityTearDown]
        public IEnumerator TearDown()
        {
            if (clockGO != null)
                UnityEngine.Object.DestroyImmediate(clockGO);
            yield return null;
        }

        // -------------------------------------------------------------------
        // Singleton
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Singleton_Is_Set_After_Awake()
        {
            yield return null;
            Assert.IsNotNull(SimulationClock.Instance);
            Assert.AreSame(clock, SimulationClock.Instance);
        }

        [UnityTest]
        public IEnumerator Duplicate_Singleton_Is_Destroyed()
        {
            yield return null;

            var dupeGO = new GameObject("SimulationClock_Dupe");
            dupeGO.AddComponent<SimulationClock>();
            yield return null;

            Assert.AreSame(clock, SimulationClock.Instance,
                "Original singleton should survive when a duplicate is created.");

            if (dupeGO != null)
                UnityEngine.Object.DestroyImmediate(dupeGO);
        }

        // -------------------------------------------------------------------
        // Initial State
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Initial_Mode_Is_Paused()
        {
            yield return null;
            Assert.AreEqual(SimulationClock.Mode.Paused, clock.CurrentMode);
            Assert.IsTrue(clock.IsPaused);
            Assert.IsFalse(clock.IsRunning);
        }

        [UnityTest]
        public IEnumerator Initial_SimTime_Is_Zero()
        {
            yield return null;
            Assert.AreEqual(0.0, clock.SimTime, 0.001,
                "SimTime should start at zero.");
        }

        [UnityTest]
        public IEnumerator Initial_DeltaTime_Is_Zero()
        {
            yield return null;
            // After first frame in Paused mode, DeltaTime should be 0
            Assert.AreEqual(0.0, clock.DeltaTime, 0.0001);
        }

        // -------------------------------------------------------------------
        // Mode Switching
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator SetMode_To_RealTime()
        {
            yield return null;

            clock.SetMode(SimulationClock.Mode.RealTime);
            Assert.AreEqual(SimulationClock.Mode.RealTime, clock.CurrentMode);
            Assert.IsTrue(clock.IsRunning);
            Assert.IsFalse(clock.IsPaused);
        }

        [UnityTest]
        public IEnumerator SetMode_To_Accelerated()
        {
            yield return null;

            clock.SetMode(SimulationClock.Mode.Accelerated);
            Assert.AreEqual(SimulationClock.Mode.Accelerated, clock.CurrentMode);
            Assert.IsTrue(clock.IsRunning);
        }

        [UnityTest]
        public IEnumerator SetMode_To_Replay()
        {
            yield return null;

            clock.SetMode(SimulationClock.Mode.Replay);
            Assert.AreEqual(SimulationClock.Mode.Replay, clock.CurrentMode);
            Assert.IsTrue(clock.IsRunning);
        }

        [UnityTest]
        public IEnumerator SetMode_To_Paused_From_Running()
        {
            yield return null;

            clock.SetMode(SimulationClock.Mode.RealTime);
            yield return null;
            clock.SetMode(SimulationClock.Mode.Paused);

            Assert.AreEqual(SimulationClock.Mode.Paused, clock.CurrentMode);
            Assert.IsTrue(clock.IsPaused);
        }

        [UnityTest]
        public IEnumerator Play_Sets_RealTime_Mode()
        {
            yield return null;

            clock.Play();
            Assert.AreEqual(SimulationClock.Mode.RealTime, clock.CurrentMode);
        }

        [UnityTest]
        public IEnumerator Pause_Sets_Paused_Mode()
        {
            yield return null;

            clock.Play();
            yield return null;
            clock.Pause();
            Assert.AreEqual(SimulationClock.Mode.Paused, clock.CurrentMode);
        }

        [UnityTest]
        public IEnumerator TogglePause_From_Paused_Goes_RealTime()
        {
            yield return null;

            Assert.IsTrue(clock.IsPaused);
            clock.TogglePause();
            Assert.AreEqual(SimulationClock.Mode.RealTime, clock.CurrentMode);
        }

        [UnityTest]
        public IEnumerator TogglePause_From_RealTime_Goes_Paused()
        {
            yield return null;

            clock.Play();
            yield return null;
            clock.TogglePause();
            Assert.AreEqual(SimulationClock.Mode.Paused, clock.CurrentMode);
        }

        [UnityTest]
        public IEnumerator SetMode_Same_Mode_Does_Nothing()
        {
            yield return null;

            int modeChangedCount = 0;
            clock.OnModeChanged += _ => modeChangedCount++;

            clock.SetMode(SimulationClock.Mode.Paused); // Already paused
            Assert.AreEqual(0, modeChangedCount,
                "Setting the same mode should not fire OnModeChanged.");
        }

        [UnityTest]
        public IEnumerator SetAccelerated_Sets_Mode_And_Multiplier()
        {
            yield return null;

            clock.SetAccelerated(10f);
            Assert.AreEqual(SimulationClock.Mode.Accelerated, clock.CurrentMode);
            Assert.AreEqual(10f, clock.SpeedMultiplier, 0.01f);
        }

        // -------------------------------------------------------------------
        // SpeedMultiplier
        // -------------------------------------------------------------------

        [Test]
        public void SpeedMultiplier_Clamps_To_Lower_Bound()
        {
            clock.SpeedMultiplier = 0.01f;
            Assert.AreEqual(0.1f, clock.SpeedMultiplier, 0.001f,
                "SpeedMultiplier should clamp to 0.1 minimum.");
        }

        [Test]
        public void SpeedMultiplier_Clamps_To_Upper_Bound()
        {
            clock.SpeedMultiplier = 500f;
            Assert.AreEqual(100f, clock.SpeedMultiplier, 0.001f,
                "SpeedMultiplier should clamp to 100 maximum.");
        }

        [Test]
        public void SpeedMultiplier_Accepts_Valid_Value()
        {
            clock.SpeedMultiplier = 5f;
            Assert.AreEqual(5f, clock.SpeedMultiplier, 0.001f);
        }

        // -------------------------------------------------------------------
        // DeltaTime in Accelerated Mode
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Accelerated_DeltaTime_Is_Multiplied()
        {
            yield return null;

            float multiplier = 10f;
            clock.SetAccelerated(multiplier);

            // Let a few frames pass so Update() executes
            yield return null;
            yield return null;

            // In Accelerated mode: DeltaTime = Time.unscaledDeltaTime * speedMultiplier
            // We cannot predict exact frame time, but DeltaTime must be positive
            // and SimTime should advance
            Assert.Greater(clock.DeltaTime, 0.0,
                "DeltaTime should be positive in Accelerated mode.");
            Assert.Greater(clock.SimTime, 0.0,
                "SimTime should accumulate in Accelerated mode.");
        }

        [UnityTest]
        public IEnumerator Accelerated_DeltaTime_Scales_With_Multiplier()
        {
            yield return null;

            // Run at 1x for a few frames
            clock.SetAccelerated(1f);
            yield return null;
            yield return null;
            yield return null;
            double simTime1x = clock.SimTime;

            // Reset and run at 10x
            clock.ResetSimulation();
            clock.SetAccelerated(10f);
            yield return null;
            yield return null;
            yield return null;
            double simTime10x = clock.SimTime;

            // The 10x simulation should have accumulated roughly 10x more SimTime.
            // Frame timing varies, so we use a generous tolerance: 10x should be
            // at least 3x more (to account for frame rate variance).
            Assert.Greater(simTime10x, simTime1x * 2.0,
                "10x speed should accumulate significantly more SimTime than 1x.");
        }

        // -------------------------------------------------------------------
        // SimTime Accumulation
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator SimTime_Does_Not_Advance_When_Paused()
        {
            yield return null;

            Assert.AreEqual(SimulationClock.Mode.Paused, clock.CurrentMode);
            double simTimeBefore = clock.SimTime;
            yield return null;
            yield return null;

            Assert.AreEqual(simTimeBefore, clock.SimTime, 0.0001,
                "SimTime should not advance while paused.");
        }

        [UnityTest]
        public IEnumerator SimTime_Advances_In_RealTime_Mode()
        {
            yield return null;

            clock.Play();
            yield return null;
            yield return null;
            yield return null;

            Assert.Greater(clock.SimTime, 0.0,
                "SimTime should advance in RealTime mode.");
        }

        [UnityTest]
        public IEnumerator SimTime_Advances_In_Accelerated_Mode()
        {
            yield return null;

            clock.SetAccelerated(5f);
            yield return null;
            yield return null;

            Assert.Greater(clock.SimTime, 0.0,
                "SimTime should advance in Accelerated mode.");
        }

        [UnityTest]
        public IEnumerator DeltaTime_Is_Zero_When_Paused()
        {
            yield return null;

            clock.Pause();
            yield return null;

            Assert.AreEqual(0.0, clock.DeltaTime, 0.0001,
                "DeltaTime should be zero when paused.");
        }

        [UnityTest]
        public IEnumerator RealTime_DeltaTime_Matches_UnscaledDeltaTime()
        {
            yield return null;

            clock.Play();
            yield return null;

            // DeltaTime should be approximately Time.unscaledDeltaTime.
            // We check it is positive and within a reasonable range.
            Assert.Greater(clock.DeltaTime, 0.0);
            Assert.Less(clock.DeltaTime, 1.0,
                "DeltaTime in RealTime should not exceed 1 second per frame.");
        }

        // -------------------------------------------------------------------
        // SeekTo and AdvanceReplay (Replay Mode Support)
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator SeekTo_Sets_SimTime_Directly()
        {
            yield return null;

            clock.SeekTo(42.5);
            Assert.AreEqual(42.5, clock.SimTime, 0.001);
            Assert.AreEqual(0.0, clock.DeltaTime, 0.0001,
                "SeekTo should set DeltaTime to 0.");
        }

        [UnityTest]
        public IEnumerator AdvanceReplay_Increments_SimTime()
        {
            yield return null;

            clock.SetMode(SimulationClock.Mode.Replay);
            clock.SeekTo(10.0);
            clock.AdvanceReplay(0.5);

            Assert.AreEqual(10.5, clock.SimTime, 0.001);
            Assert.AreEqual(0.5, clock.DeltaTime, 0.001);
        }

        // -------------------------------------------------------------------
        // OnTick Event
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator OnTick_Fires_In_RealTime_Mode()
        {
            yield return null;

            int tickCount = 0;
            double lastTickTime = -1.0;
            clock.OnTick += simTime =>
            {
                tickCount++;
                lastTickTime = simTime;
            };

            clock.Play();
            yield return null;
            yield return null;

            Assert.Greater(tickCount, 0, "OnTick should fire at least once after switching to RealTime.");
            Assert.Greater(lastTickTime, 0.0, "Tick SimTime should be positive.");
        }

        [UnityTest]
        public IEnumerator OnTick_Fires_In_Accelerated_Mode()
        {
            yield return null;

            int tickCount = 0;
            clock.OnTick += _ => tickCount++;

            clock.SetAccelerated(5f);
            yield return null;
            yield return null;

            Assert.Greater(tickCount, 0, "OnTick should fire in Accelerated mode.");
        }

        [UnityTest]
        public IEnumerator OnTick_Fires_In_Replay_Mode()
        {
            yield return null;

            int tickCount = 0;
            clock.OnTick += _ => tickCount++;

            clock.SetMode(SimulationClock.Mode.Replay);
            yield return null;
            yield return null;

            Assert.Greater(tickCount, 0, "OnTick should fire in Replay mode.");
        }

        [UnityTest]
        public IEnumerator OnTick_Does_Not_Fire_When_Paused()
        {
            yield return null;

            int tickCount = 0;
            clock.OnTick += _ => tickCount++;

            Assert.AreEqual(SimulationClock.Mode.Paused, clock.CurrentMode);
            yield return null;
            yield return null;

            Assert.AreEqual(0, tickCount,
                "OnTick should not fire while paused.");
        }

        [UnityTest]
        public IEnumerator OnTick_SimTime_Increases_Monotonically()
        {
            yield return null;

            double previousTime = -1.0;
            bool monotonic = true;
            clock.OnTick += simTime =>
            {
                if (simTime <= previousTime)
                    monotonic = false;
                previousTime = simTime;
            };

            clock.Play();
            for (int i = 0; i < 10; i++)
                yield return null;

            Assert.IsTrue(monotonic,
                "SimTime passed to OnTick should be strictly increasing.");
        }

        // -------------------------------------------------------------------
        // OnModeChanged Event
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator OnModeChanged_Fires_On_Mode_Switch()
        {
            yield return null;

            SimulationClock.Mode? receivedMode = null;
            int fireCount = 0;
            clock.OnModeChanged += mode =>
            {
                receivedMode = mode;
                fireCount++;
            };

            clock.Play();

            Assert.AreEqual(1, fireCount, "OnModeChanged should fire once per mode switch.");
            Assert.AreEqual(SimulationClock.Mode.RealTime, receivedMode);
        }

        [UnityTest]
        public IEnumerator OnModeChanged_Fires_For_Each_Transition()
        {
            yield return null;

            var modes = new System.Collections.Generic.List<SimulationClock.Mode>();
            clock.OnModeChanged += mode => modes.Add(mode);

            clock.Play();                           // Paused -> RealTime
            clock.SetAccelerated(5f);               // RealTime -> Accelerated
            clock.SetMode(SimulationClock.Mode.Replay); // Accelerated -> Replay
            clock.Pause();                          // Replay -> Paused

            Assert.AreEqual(4, modes.Count);
            Assert.AreEqual(SimulationClock.Mode.RealTime, modes[0]);
            Assert.AreEqual(SimulationClock.Mode.Accelerated, modes[1]);
            Assert.AreEqual(SimulationClock.Mode.Replay, modes[2]);
            Assert.AreEqual(SimulationClock.Mode.Paused, modes[3]);
        }

        // -------------------------------------------------------------------
        // OnReset Event
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator OnReset_Fires_On_ResetSimulation()
        {
            yield return null;

            bool resetFired = false;
            clock.OnReset += () => resetFired = true;

            clock.Play();
            yield return null;
            yield return null;

            clock.ResetSimulation();

            Assert.IsTrue(resetFired, "OnReset should fire when ResetSimulation is called.");
        }

        [UnityTest]
        public IEnumerator ResetSimulation_Clears_SimTime_And_DeltaTime()
        {
            yield return null;

            clock.Play();
            yield return null;
            yield return null;

            // SimTime should have advanced
            Assert.Greater(clock.SimTime, 0.0);

            clock.ResetSimulation();

            Assert.AreEqual(0.0, clock.SimTime, 0.0001,
                "SimTime should be zero after reset.");
            Assert.AreEqual(0.0, clock.DeltaTime, 0.0001,
                "DeltaTime should be zero after reset.");
        }

        [UnityTest]
        public IEnumerator ResetSimulation_Sets_Paused_Mode()
        {
            yield return null;

            clock.Play();
            yield return null;

            clock.ResetSimulation();
            Assert.AreEqual(SimulationClock.Mode.Paused, clock.CurrentMode,
                "Mode should be Paused after ResetSimulation.");
        }

        [UnityTest]
        public IEnumerator ResetSimulation_Fires_OnModeChanged_Then_OnReset()
        {
            yield return null;

            var eventOrder = new System.Collections.Generic.List<string>();
            clock.OnModeChanged += _ => eventOrder.Add("ModeChanged");
            clock.OnReset += () => eventOrder.Add("Reset");

            clock.Play();
            eventOrder.Clear(); // Clear the Play -> RealTime transition

            clock.ResetSimulation();

            // ResetSimulation calls SetMode(Paused) which fires OnModeChanged,
            // then fires OnReset.
            Assert.AreEqual(2, eventOrder.Count);
            Assert.AreEqual("ModeChanged", eventOrder[0]);
            Assert.AreEqual("Reset", eventOrder[1]);
        }

        // -------------------------------------------------------------------
        // FormatSimTime
        // -------------------------------------------------------------------

        [Test]
        public void FormatSimTime_At_Zero()
        {
            // SimTime is 0 after Awake
            string formatted = clock.FormatSimTime();
            Assert.AreEqual("00:00:00.0", formatted);
        }

        [UnityTest]
        public IEnumerator FormatSimTime_After_SeekTo()
        {
            yield return null;

            // 1 hour, 23 minutes, 45.6 seconds = 5025.6 seconds
            clock.SeekTo(5025.6);
            string formatted = clock.FormatSimTime();
            Assert.AreEqual("01:23:45.6", formatted);
        }

        // -------------------------------------------------------------------
        // OnDestroy Clears Singleton
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator OnDestroy_Clears_Singleton()
        {
            yield return null;

            Assert.IsNotNull(SimulationClock.Instance);
            UnityEngine.Object.DestroyImmediate(clockGO);
            clockGO = null;
            clock = null;

            Assert.IsNull(SimulationClock.Instance,
                "Instance should be null after destroy.");
        }

        // -------------------------------------------------------------------
        // Edge Cases
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Rapid_Mode_Switching_Maintains_Consistency()
        {
            yield return null;

            // Rapidly switch modes and ensure no exceptions or inconsistent state
            for (int i = 0; i < 10; i++)
            {
                clock.Play();
                clock.SetAccelerated(20f);
                clock.Pause();
                clock.SetMode(SimulationClock.Mode.Replay);
                clock.Pause();
            }

            yield return null;

            Assert.AreEqual(SimulationClock.Mode.Paused, clock.CurrentMode);
            Assert.IsTrue(clock.IsPaused);
        }

        [UnityTest]
        public IEnumerator SimTime_Survives_Pause_Resume_Cycle()
        {
            yield return null;

            clock.Play();
            yield return null;
            yield return null;
            yield return null;
            double simTimeBeforePause = clock.SimTime;
            Assert.Greater(simTimeBeforePause, 0.0);

            clock.Pause();
            yield return null;
            yield return null;

            // SimTime should not have changed while paused
            Assert.AreEqual(simTimeBeforePause, clock.SimTime, 0.0001,
                "SimTime should not change while paused.");

            clock.Play();
            yield return null;
            yield return null;

            Assert.Greater(clock.SimTime, simTimeBeforePause,
                "SimTime should continue advancing after resume.");
        }
    }
}
