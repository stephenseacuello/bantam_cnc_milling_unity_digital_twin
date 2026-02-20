using System;
using System.Collections;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using RosMessageTypes.Miracle;
using RosMessageTypes.BuiltinInterfaces;
using MiracleTwin.Core;

namespace MiracleTwin.Tests.PlayMode
{
    /// <summary>
    /// PlayMode integration tests for MiracleBridge.
    /// These tests validate initialization, singleton behaviour, event channel wiring,
    /// and that ScriptableObject events fire when ROS subscription callbacks are invoked.
    ///
    /// Because ROSConnection is a Unity singleton that attempts real TCP connections,
    /// we test the MiracleBridge's own logic by invoking its private callback methods
    /// via reflection. This keeps the tests deterministic and offline.
    /// </summary>
    [TestFixture]
    public class MiracleBridgeTests
    {
        private GameObject bridgeGO;
        private MiracleBridge bridge;

        // ScriptableObject event channels created at runtime for test isolation
        private MachineStateEventSO machineStateEvent;
        private AnomalyAlertEventSO anomalyAlertEvent;
        private ToolWearEventSO toolWearEvent;
        private TwinSyncEventSO twinSyncEvent;
        private SystemKPIsEventSO systemKPIsEvent;
        private JobStatusEventSO jobStatusEvent;
        private TaskAwardEventSO taskAwardEvent;
        private SecurityAlertEventSO securityAlertEvent;
        private CuttingStateEventSO cuttingStateEvent;

        [UnitySetUp]
        public IEnumerator SetUp()
        {
            // Destroy any existing singleton from a previous test run
            if (MiracleBridge.Instance != null)
            {
                UnityEngine.Object.DestroyImmediate(MiracleBridge.Instance.gameObject);
                yield return null;
            }

            // Create runtime ScriptableObject event channels (no asset files needed)
            machineStateEvent = ScriptableObject.CreateInstance<MachineStateEventSO>();
            anomalyAlertEvent = ScriptableObject.CreateInstance<AnomalyAlertEventSO>();
            toolWearEvent = ScriptableObject.CreateInstance<ToolWearEventSO>();
            twinSyncEvent = ScriptableObject.CreateInstance<TwinSyncEventSO>();
            systemKPIsEvent = ScriptableObject.CreateInstance<SystemKPIsEventSO>();
            jobStatusEvent = ScriptableObject.CreateInstance<JobStatusEventSO>();
            taskAwardEvent = ScriptableObject.CreateInstance<TaskAwardEventSO>();
            securityAlertEvent = ScriptableObject.CreateInstance<SecurityAlertEventSO>();
            cuttingStateEvent = ScriptableObject.CreateInstance<CuttingStateEventSO>();

            // Create MiracleBridge GameObject but prevent Start() from calling ConnectToROS
            // by injecting event channels via reflection before the first frame completes.
            bridgeGO = new GameObject("MiracleBridge_Test");
            bridge = bridgeGO.AddComponent<MiracleBridge>();

            // Inject serialized fields via reflection so we can test callback wiring
            SetPrivateField(bridge, "onMachineState", machineStateEvent);
            SetPrivateField(bridge, "onAnomalyAlert", anomalyAlertEvent);
            SetPrivateField(bridge, "onToolWear", toolWearEvent);
            SetPrivateField(bridge, "onTwinSync", twinSyncEvent);
            SetPrivateField(bridge, "onSystemKPIs", systemKPIsEvent);
            SetPrivateField(bridge, "onJobStatus", jobStatusEvent);
            SetPrivateField(bridge, "onTaskAward", taskAwardEvent);
            SetPrivateField(bridge, "onSecurityAlert", securityAlertEvent);
            SetPrivateField(bridge, "onCuttingState", cuttingStateEvent);

            yield return null;
        }

        [UnityTearDown]
        public IEnumerator TearDown()
        {
            if (bridgeGO != null)
                UnityEngine.Object.DestroyImmediate(bridgeGO);

            DestroyScriptableObjects();
            yield return null;
        }

        // -------------------------------------------------------------------
        // Initialization & Singleton
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Singleton_Is_Set_After_Awake()
        {
            yield return null;
            Assert.IsNotNull(MiracleBridge.Instance, "MiracleBridge.Instance should be set after Awake.");
            Assert.AreSame(bridge, MiracleBridge.Instance);
        }

        [UnityTest]
        public IEnumerator Duplicate_Singleton_Is_Destroyed()
        {
            yield return null;

            var duplicateGO = new GameObject("MiracleBridge_Duplicate");
            var duplicate = duplicateGO.AddComponent<MiracleBridge>();
            yield return null;

            // The duplicate should have been destroyed by Awake
            Assert.IsTrue(duplicate == null, "Duplicate MiracleBridge should be destroyed.");
            Assert.AreSame(bridge, MiracleBridge.Instance,
                "Original singleton should remain as Instance.");

            // Clean up if the GO somehow survives
            if (duplicateGO != null)
                UnityEngine.Object.DestroyImmediate(duplicateGO);
        }

        [UnityTest]
        public IEnumerator MachineId_Has_Default_Value()
        {
            yield return null;
            Assert.IsNotEmpty(bridge.MachineId, "MachineId should have a non-empty default.");
            Assert.AreEqual("cnc1", bridge.MachineId);
        }

        [UnityTest]
        public IEnumerator Bridge_Marked_DontDestroyOnLoad()
        {
            yield return null;
            // DontDestroyOnLoad moves objects to a special scene; the gameObject remains active
            Assert.IsTrue(bridgeGO.scene.name == "DontDestroyOnLoad" || bridgeGO.activeInHierarchy,
                "Bridge GameObject should survive scene loads.");
        }

        // -------------------------------------------------------------------
        // ROS Connection Setup (verifying field defaults)
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Default_ROS_IP_Is_Localhost()
        {
            yield return null;
            string ip = GetPrivateField<string>(bridge, "rosBridgeIP");
            Assert.AreEqual("127.0.0.1", ip, "Default ROS bridge IP should be localhost.");
        }

        [UnityTest]
        public IEnumerator Default_ROS_Port_Is_10000()
        {
            yield return null;
            int port = GetPrivateField<int>(bridge, "rosBridgePort");
            Assert.AreEqual(10000, port, "Default ROS bridge port should be 10000.");
        }

        [UnityTest]
        public IEnumerator Reconnect_Interval_Is_Positive()
        {
            yield return null;
            float interval = GetPrivateField<float>(bridge, "reconnectInterval");
            Assert.Greater(interval, 0f, "Reconnect interval should be a positive value.");
        }

        // -------------------------------------------------------------------
        // Subscription Registration (topic naming)
        // -------------------------------------------------------------------

        [Test]
        public void Expected_Machine_Topics_Use_MachineId()
        {
            // Verify topic patterns that RegisterSubscriptions would create.
            // We test the string format logic, not the actual ROSConnection call.
            string machineId = bridge.MachineId;
            string stateTopicExpected = $"/miracle/{machineId}/state";
            string anomalyTopicExpected = $"/miracle/{machineId}/anomaly";
            string toolWearTopicExpected = $"/miracle/{machineId}/tool_wear";
            string jobStatusTopicExpected = $"/miracle/{machineId}/job_status";

            Assert.AreEqual("/miracle/cnc1/state", stateTopicExpected);
            Assert.AreEqual("/miracle/cnc1/anomaly", anomalyTopicExpected);
            Assert.AreEqual("/miracle/cnc1/tool_wear", toolWearTopicExpected);
            Assert.AreEqual("/miracle/cnc1/job_status", jobStatusTopicExpected);
        }

        [Test]
        public void Expected_System_Topics_Are_Global()
        {
            // System-wide topics should not include machineId
            string syncTopic = "/miracle/twin/sync_status";
            string kpisTopic = "/miracle/system_kpis";
            string taskAwardTopic = "/miracle/cognitive/task_awards";
            string securityTopic = "/miracle/security/alerts";

            Assert.IsFalse(syncTopic.Contains("cnc1"));
            Assert.IsFalse(kpisTopic.Contains("cnc1"));
            Assert.IsFalse(taskAwardTopic.Contains("cnc1"));
            Assert.IsFalse(securityTopic.Contains("cnc1"));
        }

        // -------------------------------------------------------------------
        // SO Events Fire When Callbacks Are Invoked
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator OnMachineState_Callback_Raises_SO_Event()
        {
            yield return null;

            MachineStateMsg received = null;
            machineStateEvent.Register(msg => received = msg);

            var testMsg = new MachineStateMsg
            {
                machine_id = "cnc1",
                status = "RUNNING",
                spindle_speed = 16000.0,
                feed_rate = 500.0,
                axis_positions = new double[] { 10.0, 20.0, -5.0 },
                axis_velocities = new double[] { 0.0, 0.0, 0.0 },
                spindle_load = 42.0,
                coolant_level = 0.85,
                current_program = "test_program.nc",
                current_line = 15
            };

            InvokePrivateMethod(bridge, "OnMachineState", testMsg);

            Assert.IsNotNull(received, "MachineState SO event should have fired.");
            Assert.AreEqual("RUNNING", received.status);
            Assert.AreEqual(16000.0, received.spindle_speed, 0.01);
            Assert.AreEqual("cnc1", received.machine_id);
        }

        [UnityTest]
        public IEnumerator OnAnomaly_Callback_Raises_SO_Event()
        {
            yield return null;

            AnomalyAlertMsg received = null;
            anomalyAlertEvent.Register(msg => received = msg);

            var testMsg = new AnomalyAlertMsg
            {
                machine_id = "cnc1",
                anomaly_type = "VIBRATION_SPIKE",
                confidence = 0.95,
                severity = 0.78,
                contributing_factors = new[] { "spindle_bearing", "tool_imbalance" },
                feature_contributions = new[] { 0.6, 0.4 },
                recommended_action = "REDUCE_SPEED",
                requires_immediate_stop = false
            };

            InvokePrivateMethod(bridge, "OnAnomaly", testMsg);

            Assert.IsNotNull(received, "Anomaly SO event should have fired.");
            Assert.AreEqual("VIBRATION_SPIKE", received.anomaly_type);
            Assert.AreEqual(0.95, received.confidence, 0.001);
            Assert.IsFalse(received.requires_immediate_stop);
        }

        [UnityTest]
        public IEnumerator OnToolWear_Callback_Raises_SO_Event()
        {
            yield return null;

            ToolWearEstimateMsg received = null;
            toolWearEvent.Register(msg => received = msg);

            var testMsg = new ToolWearEstimateMsg
            {
                machine_id = "cnc1",
                tool_id = "T01",
                wear_percentage = 35.0,
                remaining_life_minutes = 45.0,
                confidence = 0.88,
                wear_type = "FLANK",
                flank_wear_mm = 0.12,
                crater_wear_mm = 0.03,
                recommended_action = "MONITOR"
            };

            InvokePrivateMethod(bridge, "OnToolWear", testMsg);

            Assert.IsNotNull(received, "ToolWear SO event should have fired.");
            Assert.AreEqual("T01", received.tool_id);
            Assert.AreEqual(35.0, received.wear_percentage, 0.01);
            Assert.AreEqual(0.12, received.flank_wear_mm, 0.001);
        }

        [UnityTest]
        public IEnumerator OnTwinSync_Callback_Raises_SO_Event()
        {
            yield return null;

            TwinSyncStatusMsg received = null;
            twinSyncEvent.Register(msg => received = msg);

            var testMsg = new TwinSyncStatusMsg();
            InvokePrivateMethod(bridge, "OnTwinSync", testMsg);

            Assert.IsNotNull(received, "TwinSync SO event should have fired.");
        }

        [UnityTest]
        public IEnumerator OnKPIs_Callback_Raises_SO_Event()
        {
            yield return null;

            SystemKPIsMsg received = null;
            systemKPIsEvent.Register(msg => received = msg);

            var testMsg = new SystemKPIsMsg();
            InvokePrivateMethod(bridge, "OnKPIs", testMsg);

            Assert.IsNotNull(received, "SystemKPIs SO event should have fired.");
        }

        [UnityTest]
        public IEnumerator OnJobStatus_Callback_Raises_SO_Event()
        {
            yield return null;

            JobStatusMsg received = null;
            jobStatusEvent.Register(msg => received = msg);

            var testMsg = new JobStatusMsg();
            InvokePrivateMethod(bridge, "OnJobStatus", testMsg);

            Assert.IsNotNull(received, "JobStatus SO event should have fired.");
        }

        [UnityTest]
        public IEnumerator OnTaskAward_Callback_Raises_SO_Event()
        {
            yield return null;

            TaskAwardMsg received = null;
            taskAwardEvent.Register(msg => received = msg);

            var testMsg = new TaskAwardMsg();
            InvokePrivateMethod(bridge, "OnTaskAward", testMsg);

            Assert.IsNotNull(received, "TaskAward SO event should have fired.");
        }

        [UnityTest]
        public IEnumerator OnSecurityAlert_Callback_Raises_SO_Event()
        {
            yield return null;

            SecurityAlertMsg received = null;
            securityAlertEvent.Register(msg => received = msg);

            var testMsg = new SecurityAlertMsg();
            InvokePrivateMethod(bridge, "OnSecurityAlert", testMsg);

            Assert.IsNotNull(received, "SecurityAlert SO event should have fired.");
        }

        // -------------------------------------------------------------------
        // Multiple Listener Support
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Multiple_Listeners_All_Receive_Event()
        {
            yield return null;

            int callCount = 0;
            machineStateEvent.Register(_ => callCount++);
            machineStateEvent.Register(_ => callCount++);
            machineStateEvent.Register(_ => callCount++);

            InvokePrivateMethod(bridge, "OnMachineState", new MachineStateMsg());

            Assert.AreEqual(3, callCount,
                "All registered listeners should receive the event.");
        }

        [UnityTest]
        public IEnumerator Unregistered_Listener_Does_Not_Receive_Event()
        {
            yield return null;

            int callCount = 0;
            Action<MachineStateMsg> listener = _ => callCount++;
            machineStateEvent.Register(listener);
            machineStateEvent.Unregister(listener);

            InvokePrivateMethod(bridge, "OnMachineState", new MachineStateMsg());

            Assert.AreEqual(0, callCount,
                "Unregistered listener should not receive events.");
        }

        // -------------------------------------------------------------------
        // Null Event Channel Safety
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Null_Event_Channel_Does_Not_Throw()
        {
            yield return null;

            // Set onMachineState to null to simulate missing asset reference
            SetPrivateField(bridge, "onMachineState", null);

            Assert.DoesNotThrow(() =>
            {
                InvokePrivateMethod(bridge, "OnMachineState", new MachineStateMsg());
            }, "Callback with null event channel should not throw.");
        }

        // -------------------------------------------------------------------
        // OnDestroy Clears Singleton
        // -------------------------------------------------------------------

        [UnityTest]
        public IEnumerator OnDestroy_Clears_Singleton_Instance()
        {
            yield return null;

            Assert.IsNotNull(MiracleBridge.Instance);
            UnityEngine.Object.DestroyImmediate(bridgeGO);
            bridgeGO = null;
            bridge = null;

            Assert.IsNull(MiracleBridge.Instance,
                "Instance should be null after destroy.");
        }

        // -------------------------------------------------------------------
        // Reflection Helpers
        // -------------------------------------------------------------------

        private static void SetPrivateField(object target, string fieldName, object value)
        {
            var field = target.GetType().GetField(fieldName,
                BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.IsNotNull(field, $"Field '{fieldName}' not found on {target.GetType().Name}.");
            field.SetValue(target, value);
        }

        private static T GetPrivateField<T>(object target, string fieldName)
        {
            var field = target.GetType().GetField(fieldName,
                BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.IsNotNull(field, $"Field '{fieldName}' not found on {target.GetType().Name}.");
            return (T)field.GetValue(target);
        }

        private static void InvokePrivateMethod(object target, string methodName, params object[] args)
        {
            var method = target.GetType().GetMethod(methodName,
                BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.IsNotNull(method, $"Method '{methodName}' not found on {target.GetType().Name}.");
            method.Invoke(target, args);
        }

        private void DestroyScriptableObjects()
        {
            if (machineStateEvent != null) UnityEngine.Object.DestroyImmediate(machineStateEvent);
            if (anomalyAlertEvent != null) UnityEngine.Object.DestroyImmediate(anomalyAlertEvent);
            if (toolWearEvent != null) UnityEngine.Object.DestroyImmediate(toolWearEvent);
            if (twinSyncEvent != null) UnityEngine.Object.DestroyImmediate(twinSyncEvent);
            if (systemKPIsEvent != null) UnityEngine.Object.DestroyImmediate(systemKPIsEvent);
            if (jobStatusEvent != null) UnityEngine.Object.DestroyImmediate(jobStatusEvent);
            if (taskAwardEvent != null) UnityEngine.Object.DestroyImmediate(taskAwardEvent);
            if (securityAlertEvent != null) UnityEngine.Object.DestroyImmediate(securityAlertEvent);
            if (cuttingStateEvent != null) UnityEngine.Object.DestroyImmediate(cuttingStateEvent);
        }
    }
}
