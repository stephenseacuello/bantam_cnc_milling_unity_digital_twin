#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEngine.UIElements;
using UnityEditor.SceneManagement;
using MiracleTwin.Core;
using MiracleTwin.UI;
using MiracleTwin.CNC;
using MiracleTwin.Testing;
using MiracleTwin.Cutting;
using MiracleTwin.Robots;
using MiracleTwin.Visualization;
using MiracleTwin.Audio;

namespace MiracleTwin.Editor
{
    public static class DashboardWiring
    {
        private const string EventFolder = "Assets/ScriptableObjects/Events";
        private const string ProfileFolder = "Assets/ScriptableObjects/CNCProfiles";
        private const string ToolFolder = "Assets/ScriptableObjects/Tools";
        private const string PanelSettingsPath = "Assets/UI/MiraclePanelSettings.asset";
        private const string TextSettingsPath = "Assets/UI/MiraclePanelTextSettings.asset";
        private const string DashboardUxmlPath = "Assets/UI/Dashboard.uxml";

        // Known CNC machine names in the scene hierarchy
        private static readonly string[] BantamNames = { "BantamExplorer_CNC", "BantamExplorer", "Bantam_CNC", "BantamExplorerCNC" };
        private static readonly string[] CR1Names = { "CoastRunnerCR1_CNC", "CoastRunnerCR1", "CoastRunner_CNC" };

        [MenuItem("MIRACLE/Wire Dashboard")]
        public static void WireDashboard()
        {
            // ── Step 1: Ensure all event assets exist ──
            EventAssetCreator.CreateAllEventAssets();

            // ── Step 2: Create PanelTextSettings + PanelSettings assets if missing ──
            EnsureFolderExists("Assets/UI");

            PanelTextSettings textSettings = AssetDatabase.LoadAssetAtPath<PanelTextSettings>(TextSettingsPath);
            if (textSettings == null)
            {
                textSettings = ScriptableObject.CreateInstance<PanelTextSettings>();
                AssetDatabase.CreateAsset(textSettings, TextSettingsPath);
                AssetDatabase.SaveAssets();
                Debug.Log($"[DashboardWiring] Created PanelTextSettings: {TextSettingsPath}");
            }

            PanelSettings panelSettings = AssetDatabase.LoadAssetAtPath<PanelSettings>(PanelSettingsPath);
            if (panelSettings == null)
            {
                panelSettings = ScriptableObject.CreateInstance<PanelSettings>();
                AssetDatabase.CreateAsset(panelSettings, PanelSettingsPath);
                AssetDatabase.SaveAssets();
                Debug.Log($"[DashboardWiring] Created PanelSettings: {PanelSettingsPath}");
            }

            panelSettings.textSettings = textSettings;
            EditorUtility.SetDirty(panelSettings);
            AssetDatabase.SaveAssets();

            // ── Step 3: Find or create _UI root GameObject ──
            GameObject uiRoot = GameObject.Find("_UI");
            if (uiRoot == null)
            {
                uiRoot = new GameObject("_UI");
                Undo.RegisterCreatedObjectUndo(uiRoot, "Create _UI");
                Debug.Log("[DashboardWiring] Created _UI root GameObject.");
            }

            // ── Step 4: Create DashboardUI child under _UI ──
            Transform dashboardTransform = uiRoot.transform.Find("DashboardUI");
            GameObject dashboardGO;

            if (dashboardTransform != null)
            {
                dashboardGO = dashboardTransform.gameObject;
                Debug.Log("[DashboardWiring] Found existing DashboardUI GameObject.");
            }
            else
            {
                dashboardGO = new GameObject("DashboardUI");
                dashboardGO.transform.SetParent(uiRoot.transform);
                Undo.RegisterCreatedObjectUndo(dashboardGO, "Create DashboardUI");
                Debug.Log("[DashboardWiring] Created DashboardUI GameObject under _UI.");
            }

            // ── UIDocument component ──
            UIDocument uiDocument = dashboardGO.GetComponent<UIDocument>();
            if (uiDocument == null)
            {
                uiDocument = dashboardGO.AddComponent<UIDocument>();
            }

            VisualTreeAsset uxmlAsset = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(DashboardUxmlPath);
            if (uxmlAsset != null)
            {
                uiDocument.visualTreeAsset = uxmlAsset;
                Debug.Log($"[DashboardWiring] Assigned UXML: {DashboardUxmlPath}");
            }
            else
            {
                Debug.LogWarning($"[DashboardWiring] UXML not found at: {DashboardUxmlPath} - assign manually.");
            }

            uiDocument.panelSettings = panelSettings;

            // ── DashboardOverlay component ──
            DashboardOverlay overlay = dashboardGO.GetComponent<DashboardOverlay>();
            if (overlay == null)
            {
                overlay = dashboardGO.AddComponent<DashboardOverlay>();
            }

            SerializedObject overlaySO = new SerializedObject(overlay);
            overlaySO.FindProperty("uiDocument").objectReferenceValue = uiDocument;
            overlaySO.FindProperty("machineStateEvent").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<MachineStateEventSO>($"{EventFolder}/MachineStateEvent.asset");
            overlaySO.FindProperty("systemKPIsEvent").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<SystemKPIsEventSO>($"{EventFolder}/SystemKPIsEvent.asset");
            overlaySO.FindProperty("cuttingStateEvent").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<CuttingStateEventSO>($"{EventFolder}/CuttingStateEvent.asset");
            overlaySO.ApplyModifiedProperties();
            Debug.Log("[DashboardWiring] Wired DashboardOverlay component.");

            // ── SimulationControlPanel component ──
            SimulationControlPanel simPanel = dashboardGO.GetComponent<SimulationControlPanel>();
            if (simPanel == null)
            {
                simPanel = dashboardGO.AddComponent<SimulationControlPanel>();
            }

            SerializedObject simPanelSO = new SerializedObject(simPanel);
            simPanelSO.FindProperty("uiDocument").objectReferenceValue = uiDocument;
            simPanelSO.ApplyModifiedProperties();
            Debug.Log("[DashboardWiring] Wired SimulationControlPanel component.");

            // ── EStopButton component ──
            EStopButton eStop = dashboardGO.GetComponent<EStopButton>();
            if (eStop == null)
            {
                eStop = dashboardGO.AddComponent<EStopButton>();
            }

            SerializedObject eStopSO = new SerializedObject(eStop);
            eStopSO.FindProperty("uiDocument").objectReferenceValue = uiDocument;
            eStopSO.ApplyModifiedProperties();
            Debug.Log("[DashboardWiring] Wired EStopButton component.");

            // ── Step 5: Ensure _Managers + MiracleBridge exist ──
            GameObject managersRoot = GameObject.Find("_Managers");
            if (managersRoot == null)
            {
                managersRoot = new GameObject("_Managers");
                Undo.RegisterCreatedObjectUndo(managersRoot, "Create _Managers");
                Debug.Log("[DashboardWiring] Created _Managers root GameObject.");
            }

            MiracleBridge bridge = managersRoot.GetComponentInChildren<MiracleBridge>();
            if (bridge == null)
            {
                GameObject bridgeGO = new GameObject("MiracleBridge");
                bridgeGO.transform.SetParent(managersRoot.transform);
                bridge = bridgeGO.AddComponent<MiracleBridge>();
                Undo.RegisterCreatedObjectUndo(bridgeGO, "Create MiracleBridge");
                Debug.Log("[DashboardWiring] Created MiracleBridge under _Managers.");
            }

            SerializedObject bridgeSO = new SerializedObject(bridge);
            bridgeSO.FindProperty("onMachineState").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<MachineStateEventSO>($"{EventFolder}/MachineStateEvent.asset");
            bridgeSO.FindProperty("onAnomalyAlert").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<AnomalyAlertEventSO>($"{EventFolder}/AnomalyAlertEvent.asset");
            bridgeSO.FindProperty("onToolWear").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<ToolWearEventSO>($"{EventFolder}/ToolWearEvent.asset");
            bridgeSO.FindProperty("onTwinSync").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<TwinSyncEventSO>($"{EventFolder}/TwinSyncEvent.asset");
            bridgeSO.FindProperty("onSystemKPIs").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<SystemKPIsEventSO>($"{EventFolder}/SystemKPIsEvent.asset");
            bridgeSO.FindProperty("onJobStatus").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<JobStatusEventSO>($"{EventFolder}/JobStatusEvent.asset");
            bridgeSO.FindProperty("onTaskAward").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<TaskAwardEventSO>($"{EventFolder}/TaskAwardEvent.asset");
            bridgeSO.FindProperty("onSecurityAlert").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<SecurityAlertEventSO>($"{EventFolder}/SecurityAlertEvent.asset");
            bridgeSO.FindProperty("onCuttingState").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<CuttingStateEventSO>($"{EventFolder}/CuttingStateEvent.asset");
            bridgeSO.FindProperty("onRobotJointState").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<RobotJointStateEventSO>($"{EventFolder}/RobotJointStateEvent.asset");
            bridgeSO.ApplyModifiedProperties();
            Debug.Log("[DashboardWiring] Wired all event SO fields on MiracleBridge.");

            // ── Step 5b: Ensure SensorDataBridge exists on _Managers ──
            SensorDataBridge sensorBridge = managersRoot.GetComponentInChildren<SensorDataBridge>();
            if (sensorBridge == null)
            {
                GameObject sensorGO = new GameObject("SensorDataBridge");
                sensorGO.transform.SetParent(managersRoot.transform);
                sensorBridge = sensorGO.AddComponent<SensorDataBridge>();
                Undo.RegisterCreatedObjectUndo(sensorGO, "Create SensorDataBridge");
                Debug.Log("[DashboardWiring] Created SensorDataBridge under _Managers (subscribes to micro-ROS raw sensors).");
            }

            // ── Step 6: Create CNC Profiles ──
            CNCProfileCreator.CreateAllProfiles();

            // ── Step 7: Find CNC GameObjects and wire controllers + ArticulationBody joints ──
            var machineStateAsset = AssetDatabase.LoadAssetAtPath<MachineStateEventSO>($"{EventFolder}/MachineStateEvent.asset");
            var bantamProfile = AssetDatabase.LoadAssetAtPath<CNCMachineProfileSO>($"{ProfileFolder}/BantamExplorerProfile.asset");
            var cr1Profile = AssetDatabase.LoadAssetAtPath<CNCMachineProfileSO>($"{ProfileFolder}/CoastRunnerCR1Profile.asset");

            var wiredControllers = new List<MonoBehaviour>();

            // -- Bantam Explorer --
            BantamExplorerController bantam = Object.FindFirstObjectByType<BantamExplorerController>();
            if (bantam == null)
            {
                GameObject bantamGO = FindByNames(BantamNames);
                if (bantamGO != null)
                {
                    bantam = bantamGO.AddComponent<BantamExplorerController>();
                    Debug.Log($"[DashboardWiring] Added BantamExplorerController to '{bantamGO.name}'.");
                }
            }

            if (bantam != null)
            {
                // Clean up any previously-created invisible AB overlay
                CleanupOverlayJoints(bantam.gameObject, "Bantam");

                // Wire ArticulationBody prismatic joints on Fusion FBX children
                EnsureArticulatedJoints(bantam.gameObject, bantamProfile, "Bantam");

                SerializedObject bantamSO = new SerializedObject(bantam);
                bantamSO.FindProperty("profile").objectReferenceValue = bantamProfile;
                bantamSO.FindProperty("machineStateEvent").objectReferenceValue = machineStateAsset;

                // Auto-assign movableHead as fallback if no ABs found
                if (bantamSO.FindProperty("movableHead").objectReferenceValue == null)
                {
                    Transform head = FindMovableHead(bantam.transform);
                    if (head != null)
                    {
                        bantamSO.FindProperty("movableHead").objectReferenceValue = head;
                        Debug.Log($"[DashboardWiring] Auto-assigned movableHead on Bantam → {head.name}");
                    }
                }

                bantamSO.ApplyModifiedProperties();
                wiredControllers.Add(bantam);
                Debug.Log("[DashboardWiring] Wired BantamExplorerController profile and event.");
            }
            else
            {
                Debug.LogWarning("[DashboardWiring] BantamExplorer CNC not found in scene. " +
                    $"Looked for GameObjects named: {string.Join(", ", BantamNames)}");
            }

            // -- CoastRunner CR-1 --
            CoastRunnerCR1Controller cr1 = Object.FindFirstObjectByType<CoastRunnerCR1Controller>();
            if (cr1 == null)
            {
                GameObject cr1GO = FindByNames(CR1Names);
                if (cr1GO != null)
                {
                    cr1 = cr1GO.AddComponent<CoastRunnerCR1Controller>();
                    Debug.Log($"[DashboardWiring] Added CoastRunnerCR1Controller to '{cr1GO.name}'.");
                }
            }

            if (cr1 != null)
            {
                // Clean up any previously-created invisible AB overlay
                CleanupOverlayJoints(cr1.gameObject, "CR-1");

                // Wire ArticulationBody prismatic joints on Fusion FBX children
                EnsureArticulatedJoints(cr1.gameObject, cr1Profile, "CoastRunner");

                SerializedObject cr1SO = new SerializedObject(cr1);
                cr1SO.FindProperty("profile").objectReferenceValue = cr1Profile;
                cr1SO.FindProperty("machineStateEvent").objectReferenceValue = machineStateAsset;

                // Auto-assign movableHead as fallback
                if (cr1SO.FindProperty("movableHead").objectReferenceValue == null)
                {
                    Transform head = FindMovableHead(cr1.transform);
                    if (head != null)
                    {
                        cr1SO.FindProperty("movableHead").objectReferenceValue = head;
                        Debug.Log($"[DashboardWiring] Auto-assigned movableHead on CR-1 → {head.name}");
                    }
                }

                cr1SO.ApplyModifiedProperties();
                wiredControllers.Add(cr1);
                Debug.Log("[DashboardWiring] Wired CoastRunnerCR1Controller profile and event.");
            }
            else
            {
                Debug.LogWarning("[DashboardWiring] CoastRunner CR-1 CNC not found in scene. " +
                    $"Looked for GameObjects named: {string.Join(", ", CR1Names)}");
            }

            // ── Step 8: Wire CNCMachineSelector with controller references ──
            CNCMachineSelector machineSelector = dashboardGO.GetComponent<CNCMachineSelector>();
            if (machineSelector == null)
            {
                machineSelector = dashboardGO.AddComponent<CNCMachineSelector>();
            }

            SerializedObject selectorSO = new SerializedObject(machineSelector);
            selectorSO.FindProperty("uiDocument").objectReferenceValue = uiDocument;

            SerializedProperty controllersProp = selectorSO.FindProperty("machineControllers");
            controllersProp.ClearArray();
            for (int i = 0; i < wiredControllers.Count; i++)
            {
                controllersProp.InsertArrayElementAtIndex(i);
                controllersProp.GetArrayElementAtIndex(i).objectReferenceValue = wiredControllers[i];
            }

            selectorSO.ApplyModifiedProperties();
            Debug.Log($"[DashboardWiring] Wired CNCMachineSelector with {wiredControllers.Count} machine(s).");

            // ── Step 9: Add LocalCNCTestDriver for testing without ROS2 ──
            LocalCNCTestDriver testDriver = dashboardGO.GetComponent<LocalCNCTestDriver>();
            if (testDriver == null)
            {
                testDriver = dashboardGO.AddComponent<LocalCNCTestDriver>();
            }

            SerializedObject testDriverSO = new SerializedObject(testDriver);
            testDriverSO.FindProperty("machineStateEvent").objectReferenceValue = machineStateAsset;
            testDriverSO.FindProperty("cuttingStateEvent").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<CuttingStateEventSO>($"{EventFolder}/CuttingStateEvent.asset");
            testDriverSO.FindProperty("systemKPIsEvent").objectReferenceValue =
                AssetDatabase.LoadAssetAtPath<SystemKPIsEventSO>($"{EventFolder}/SystemKPIsEvent.asset");
            testDriverSO.ApplyModifiedProperties();
            Debug.Log("[DashboardWiring] Wired LocalCNCTestDriver with all event channels.");

            // ── Step 9b: Wire RobotControllers on robot GameObjects ──
            var robotJointStateAsset = AssetDatabase.LoadAssetAtPath<RobotJointStateEventSO>(
                $"{EventFolder}/RobotJointStateEvent.asset");
            WireRobotControllers(robotJointStateAsset);

            // ── Step 10: Wire cutting physics pipeline on all CNC machines ──
            CuttingSimulationManager primarySimManager = null;
            if (bantam != null)
                primarySimManager = WireCuttingPhysicsOnMachine(bantam, "Bantam", overlay);
            if (cr1 != null)
            {
                var cr1SimManager = WireCuttingPhysicsOnMachine(cr1, "CoastRunner", null);
                if (primarySimManager == null)
                    primarySimManager = cr1SimManager;
            }

            // Wire the primary sim manager to DashboardOverlay (first machine found)
            if (overlay != null && primarySimManager != null)
            {
                var overlaySimSO = new SerializedObject(overlay);
                overlaySimSO.FindProperty("cuttingSimManager").objectReferenceValue = primarySimManager;
                overlaySimSO.ApplyModifiedProperties();
                Debug.Log("[DashboardWiring] Wired CuttingSimulationManager → DashboardOverlay.");
            }

            // ── Step 11: Wire GCodeExecutor on ALL CNC machines + GCodeEditor on DashboardUI ──
            GCodeExecutor primaryExecutor = null;
            foreach (var controller in wiredControllers)
            {
                var exec = controller.gameObject.GetComponent<GCodeExecutor>();
                if (exec == null)
                    exec = controller.gameObject.AddComponent<GCodeExecutor>();

                var simMgr = controller.gameObject.GetComponent<CuttingSimulationManager>();
                var execSO = new SerializedObject(exec);
                execSO.FindProperty("cncControllerObject").objectReferenceValue = controller;
                if (simMgr != null)
                    execSO.FindProperty("cuttingSimManager").objectReferenceValue = simMgr;
                execSO.ApplyModifiedProperties();

                if (primaryExecutor == null)
                    primaryExecutor = exec;

                Debug.Log($"[DashboardWiring] Wired GCodeExecutor on {(controller as ICNCController)?.MachineName ?? controller.name}.");
            }

            if (primaryExecutor != null)
            {
                // GCodeEditor on the DashboardUI
                GCodeEditor gCodeEditor = dashboardGO.GetComponent<GCodeEditor>();
                if (gCodeEditor == null)
                    gCodeEditor = dashboardGO.AddComponent<GCodeEditor>();

                var editorSO = new SerializedObject(gCodeEditor);
                editorSO.FindProperty("uiDocument").objectReferenceValue = uiDocument;
                editorSO.FindProperty("gCodeExecutor").objectReferenceValue = primaryExecutor;
                editorSO.ApplyModifiedProperties();
                Debug.Log("[DashboardWiring] Wired GCodeEditor on DashboardUI.");

                // ToolpathPreview under _Managers
                var toolpathPreviewGO = managersRoot.transform.Find("ToolpathPreview");
                GameObject tpGO;
                if (toolpathPreviewGO != null)
                    tpGO = toolpathPreviewGO.gameObject;
                else
                {
                    tpGO = new GameObject("ToolpathPreview");
                    tpGO.transform.SetParent(managersRoot.transform);
                    Undo.RegisterCreatedObjectUndo(tpGO, "Create ToolpathPreview");
                }

                var toolpathPreview = tpGO.GetComponent<Visualization.ToolpathPreview>();
                if (toolpathPreview == null)
                    toolpathPreview = tpGO.AddComponent<Visualization.ToolpathPreview>();

                var tpSO = new SerializedObject(toolpathPreview);
                tpSO.FindProperty("gCodeExecutor").objectReferenceValue = primaryExecutor;
                tpSO.ApplyModifiedProperties();
                Debug.Log("[DashboardWiring] Wired ToolpathPreview under _Managers.");
            }

            // ── Step 12: Wire audio controllers on CNC machines ──
            var cuttingStateAsset = AssetDatabase.LoadAssetAtPath<CuttingStateEventSO>($"{EventFolder}/CuttingStateEvent.asset");
            foreach (var controller in wiredControllers)
            {
                var machineGO = controller.gameObject;
                string mName = (controller as ICNCController)?.MachineName ?? machineGO.name;

                // SpindleSoundController
                var spindleSound = machineGO.GetComponent<SpindleSoundController>();
                if (spindleSound == null)
                    spindleSound = machineGO.AddComponent<SpindleSoundController>();

                var ssSO = new SerializedObject(spindleSound);
                ssSO.FindProperty("machineStateEvent").objectReferenceValue = machineStateAsset;
                ssSO.ApplyModifiedProperties();

                // CuttingSoundController
                var cuttingSound = machineGO.GetComponent<CuttingSoundController>();
                if (cuttingSound == null)
                    cuttingSound = machineGO.AddComponent<CuttingSoundController>();

                var csSO = new SerializedObject(cuttingSound);
                csSO.FindProperty("cuttingStateEvent").objectReferenceValue = cuttingStateAsset;
                csSO.ApplyModifiedProperties();

                Debug.Log($"[DashboardWiring] Wired SpindleSoundController + CuttingSoundController on {mName}.");
            }

            // ── Step 13: Wire live charts on DashboardUI ──
            WireLiveCharts(dashboardGO, uiDocument, cuttingStateAsset);

            // ── Step 14: Wire VoxelWorkpiece + HeatMapOverlay + ForceArrowRenderer on CNC machines ──
            foreach (var controller in wiredControllers)
            {
                WireVoxelWorkpiece(controller.gameObject, controller.name);
                WireHeatMapOverlay(controller.gameObject, cuttingStateAsset);
                WireForceArrowRenderer(controller.gameObject, cuttingStateAsset);
            }

            // Re-link VoxelWorkpiece into CuttingSimulationManagers (now that we created them)
            foreach (var controller in wiredControllers)
            {
                var vw = controller.GetComponentInChildren<VoxelWorkpiece>();
                var sm = controller.GetComponent<CuttingSimulationManager>();
                if (vw != null && sm != null)
                {
                    var smSO2 = new SerializedObject(sm);
                    smSO2.FindProperty("voxelWorkpiece").objectReferenceValue = vw;
                    smSO2.ApplyModifiedProperties();
                    Debug.Log($"[DashboardWiring] Linked VoxelWorkpiece → CuttingSimulationManager on {controller.name}.");
                }
            }

            // ── Step 15: Save scene ──
            EditorSceneManager.MarkSceneDirty(UnityEngine.SceneManagement.SceneManager.GetActiveScene());
            EditorSceneManager.SaveOpenScenes();
            Debug.Log("[DashboardWiring] Dashboard wiring complete. Scene saved.");
        }

        /// <summary>
        /// Find robot GameObjects in the scene and ensure they have wired RobotControllers.
        /// </summary>
        private static void WireRobotControllers(RobotJointStateEventSO jointStateAsset)
        {
            string[][] robotEntries = {
                new[] { "lite6", "lite6" },
                new[] { "niryo_ned2", "ned2" },
                new[] { "xarm6", "xarm6" },
                new[] { "xArm6", "xarm6" },
            };

            int wired = 0;
            foreach (var entry in robotEntries)
            {
                string goName = entry[0];
                string robotId = entry[1];

                var go = GameObject.Find(goName);
                if (go == null) continue;

                var ctrl = go.GetComponent<RobotController>();
                if (ctrl == null)
                {
                    ctrl = go.AddComponent<RobotController>();
                    Debug.Log($"[DashboardWiring] Added RobotController to '{goName}'.");
                }

                var ctrlSO = new SerializedObject(ctrl);
                ctrlSO.FindProperty("robotId").stringValue = robotId;
                if (jointStateAsset != null)
                    ctrlSO.FindProperty("jointStateEvent").objectReferenceValue = jointStateAsset;
                ctrlSO.ApplyModifiedProperties();
                wired++;
                Debug.Log($"[DashboardWiring] Wired RobotController on '{goName}' (id={robotId}, jointStateEvent={jointStateAsset != null}).");
            }

            if (wired > 0)
                Debug.Log($"[DashboardWiring] Wired {wired} robot controller(s) with jointStateEvent SO.");
            else
                Debug.LogWarning("[DashboardWiring] No robot GameObjects found (looked for: lite6, niryo_ned2, xarm6).");
        }

        /// <summary>
        /// Wire the cutting physics pipeline onto any CNC machine implementing ICNCController.
        /// Creates/wires: ToolDefinition assets, CuttingForceEngine, StabilityLobeChart,
        /// CuttingSimulationManager. Returns the created CuttingSimulationManager.
        /// </summary>
        private static CuttingSimulationManager WireCuttingPhysicsOnMachine(
            MonoBehaviour controller, string machineName, DashboardOverlay overlay)
        {
            // 1. Create tool definition assets (idempotent)
            ToolDefinitionCreator.CreateAllToolDefinitions();

            // 2. Load cutting event and tool definition
            var cuttingStateEvent = AssetDatabase.LoadAssetAtPath<CuttingStateEventSO>(
                $"{EventFolder}/CuttingStateEvent.asset");
            var toolDef = AssetDatabase.LoadAssetAtPath<ToolDefinition>(
                $"{ToolFolder}/HSS_QuarterInch_2Flute.asset");

            if (cuttingStateEvent == null)
            {
                Debug.LogError($"[DashboardWiring] CuttingStateEvent.asset not found — cannot wire physics on {machineName}.");
                return null;
            }

            GameObject machineGO = controller.gameObject;

            // 3. CuttingForceEngine component
            var forceEngine = machineGO.GetComponent<CuttingForceEngine>();
            if (forceEngine == null)
                forceEngine = machineGO.AddComponent<CuttingForceEngine>();

            if (toolDef != null)
            {
                var feSO = new SerializedObject(forceEngine);
                feSO.FindProperty("toolDiameter").floatValue = toolDef.diameter;
                feSO.FindProperty("fluteCount").intValue = toolDef.fluteCount;
                feSO.FindProperty("helixAngle").floatValue = toolDef.helixAngle;
                feSO.FindProperty("rakeAngle").floatValue = toolDef.rakeAngle;
                feSO.FindProperty("Ktc").floatValue = toolDef.Ktc;
                feSO.FindProperty("Krc").floatValue = toolDef.Krc;
                feSO.FindProperty("Kac").floatValue = toolDef.Kac;
                feSO.FindProperty("Kte").floatValue = toolDef.Kte;
                feSO.FindProperty("Kre").floatValue = toolDef.Kre;
                feSO.FindProperty("Kae").floatValue = toolDef.Kae;
                feSO.ApplyModifiedProperties();
            }
            Debug.Log($"[DashboardWiring] Wired CuttingForceEngine on {machineName}.");

            // 4. StabilityLobeChart on child GO
            Transform stabTransform = machineGO.transform.Find("StabilityLobeChart");
            GameObject stabGO;
            if (stabTransform != null)
            {
                stabGO = stabTransform.gameObject;
            }
            else
            {
                stabGO = new GameObject("StabilityLobeChart");
                stabGO.transform.SetParent(machineGO.transform);
                stabGO.transform.localPosition = Vector3.zero;
                Undo.RegisterCreatedObjectUndo(stabGO, $"Create StabilityLobeChart ({machineName})");
            }

            var stabilityChart = stabGO.GetComponent<StabilityLobeChart>();
            if (stabilityChart == null)
                stabilityChart = stabGO.AddComponent<StabilityLobeChart>();

            if (toolDef != null)
            {
                var slcSO = new SerializedObject(stabilityChart);
                slcSO.FindProperty("Ktc").floatValue = toolDef.Ktc;
                slcSO.FindProperty("fluteCount").intValue = toolDef.fluteCount;
                slcSO.ApplyModifiedProperties();
            }
            Debug.Log($"[DashboardWiring] Wired StabilityLobeChart on {machineName}.");

            // 5. Look for existing VoxelWorkpiece (don't create — needs compute shaders)
            var voxelWorkpiece = Object.FindFirstObjectByType<VoxelWorkpiece>();

            // 6. CuttingSimulationManager
            var simManager = machineGO.GetComponent<CuttingSimulationManager>();
            if (simManager == null)
                simManager = machineGO.AddComponent<CuttingSimulationManager>();

            var smSO = new SerializedObject(simManager);
            smSO.FindProperty("cncControllerObject").objectReferenceValue = controller;
            smSO.FindProperty("forceEngine").objectReferenceValue = forceEngine;
            smSO.FindProperty("cuttingStateEvent").objectReferenceValue = cuttingStateEvent;
            smSO.FindProperty("activeTool").objectReferenceValue = toolDef;
            smSO.FindProperty("stabilityLobeChart").objectReferenceValue = stabilityChart;
            if (voxelWorkpiece != null)
                smSO.FindProperty("voxelWorkpiece").objectReferenceValue = voxelWorkpiece;
            smSO.ApplyModifiedProperties();
            Debug.Log($"[DashboardWiring] Wired CuttingSimulationManager on {machineName}.");

            return simManager;
        }

        /// <summary>
        /// Remove previously-created invisible AB overlay GameObjects (X_Gantry, Y_Table, Z_Head)
        /// that have no mesh. These caused controllers to drive invisible joints instead of visible meshes.
        /// </summary>
        private static void CleanupOverlayJoints(GameObject root, string machineName)
        {
            string[] overlayNames = { "X_Gantry", "Y_Table", "Z_Head" };
            var toDestroy = new List<GameObject>();

            foreach (var name in overlayNames)
            {
                var t = root.transform.Find(name);
                if (t != null && t.GetComponentInChildren<MeshRenderer>() == null)
                {
                    toDestroy.Add(t.gameObject);
                }
            }

            // Also remove immovable root AB if it was added for the overlay chain
            var rootAB = root.GetComponent<ArticulationBody>();
            if (rootAB != null && rootAB.immovable && root.GetComponent<MeshRenderer>() == null)
            {
                // Only remove if no mesh-bearing children use it as chain root
                bool hasRealChildren = false;
                foreach (var ab in root.GetComponentsInChildren<ArticulationBody>())
                {
                    if (ab == rootAB) continue;
                    if (ab.GetComponentInChildren<MeshRenderer>() != null)
                    {
                        hasRealChildren = true;
                        break;
                    }
                }
                if (!hasRealChildren)
                    Object.DestroyImmediate(rootAB);
            }

            foreach (var go in toDestroy)
            {
                Debug.Log($"[DashboardWiring] Removing overlay joint '{go.name}' from {machineName} (no mesh).");
                Undo.DestroyObjectImmediate(go);
            }

            if (toDestroy.Count > 0)
                Debug.Log($"[DashboardWiring] Cleaned up {toDestroy.Count} overlay joints from {machineName}.");
        }

        /// <summary>
        /// Ensure the CNC machine root has ArticulationBody prismatic joints for 3 axes.
        /// If none exist, creates an invisible articulated overlay:
        ///   Root (immovable AB) → X_Gantry (prismatic X) → Y_Table (prismatic Z) → Z_Head (prismatic -Y)
        /// The CNC controllers auto-discover these at runtime.
        /// </summary>
        /// <summary>
        /// Search all descendants for a Transform whose name matches any of the candidates (case-insensitive).
        /// Fusion 360 may append ":1" suffixes, so we match with Contains after lowering.
        /// </summary>
        private static Transform FindChildByNames(Transform root, string[] candidates)
        {
            foreach (Transform child in root.GetComponentsInChildren<Transform>())
            {
                if (child == root) continue;
                string lower = child.name.ToLowerInvariant();
                foreach (var name in candidates)
                {
                    if (lower == name || lower.StartsWith(name + ":") || lower.StartsWith(name + " "))
                        return child;
                }
            }
            return null;
        }

        private static void EnsureArticulatedJoints(GameObject root, CNCMachineProfileSO profile, string machineName)
        {
            // Unpack prefab instance so we can reparent children and add components
            if (PrefabUtility.IsPartOfPrefabInstance(root))
            {
                var prefabRoot = PrefabUtility.GetNearestPrefabInstanceRoot(root);
                PrefabUtility.UnpackPrefabInstance(prefabRoot, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
                Debug.Log($"[DashboardWiring] Unpacked prefab instance '{prefabRoot.name}' for {machineName} — needed for reparenting and AB joints.");
            }

            // Check if this machine already has prismatic AB joints
            var existingBodies = root.GetComponentsInChildren<ArticulationBody>();
            int prismaticCount = 0;
            foreach (var ab in existingBodies)
            {
                if (ab.jointType == ArticulationJointType.PrismaticJoint)
                    prismaticCount++;
            }

            if (prismaticCount >= 3)
            {
                Debug.Log($"[DashboardWiring] {machineName} already has {prismaticCount} prismatic joints — skipping overlay.");
                return;
            }

            // Read travel limits from profile (mm → meters)
            Vector3 maxMM = profile != null ? profile.maxPosition : new Vector3(152.4f, 101.6f, 69.85f);
            float xTravel = maxMM.x / 1000f;
            float yTravel = maxMM.y / 1000f;
            float zTravel = maxMM.z / 1000f;

            // Ensure root has an immovable ArticulationBody (required as chain root)
            var rootAB = root.GetComponent<ArticulationBody>();
            if (rootAB == null)
            {
                rootAB = root.AddComponent<ArticulationBody>();
                rootAB.immovable = true;
            }
            else if (!rootAB.isRoot)
            {
                Debug.Log($"[DashboardWiring] {machineName} root already has AB chain.");
                return;
            }

            // --- Try to find Fusion 360 FBX children by name ---
            // Bantam style: "X-Axis", "Y-Axis", "Z-Axis"
            // CoastRunner style: "Subassembly X - Table", "Subassembly YZ - Translator", "Subassembly Z"
            string[] xNames = { "x-axis", "x_axis", "x_gantry", "xaxis", "x gantry", "subassembly x" };
            string[] yNames = { "y-axis", "y_axis", "y_table", "yaxis", "y table", "y_bed", "subassembly yz" };
            string[] zNames = { "z-axis", "z_axis", "z_head", "zaxis", "z head", "subassembly z" };

            Transform xChild = FindChildByNames(root.transform, xNames);
            Transform yChild = FindChildByNames(root.transform, yNames);
            Transform zChild = FindChildByNames(root.transform, zNames);

            if (xChild != null && yChild != null && zChild != null)
            {
                Debug.Log($"[DashboardWiring] {machineName}: Found Fusion FBX axis children: " +
                    $"X='{xChild.name}', Y='{yChild.name}', Z='{zChild.name}'");

                // Determine kinematic chain based on machine type:
                //   Bantam:      Root → X (gantry) → Y (table on gantry), Root → Z (head, separate column)
                //   CoastRunner: Root → X (table, independent), Root → Y (translator) → Z (head on translator)
                bool isCoastRunner = yChild.name.ToLowerInvariant().Contains("yz") ||
                                     machineName.ToLowerInvariant().Contains("coastrunner") ||
                                     machineName.ToLowerInvariant().Contains("coast runner") ||
                                     machineName.ToLowerInvariant().Contains("cr-1") ||
                                     machineName.ToLowerInvariant().Contains("cr1");

                if (isCoastRunner)
                {
                    // CoastRunner: X independent, Z rides on YZ translator
                    xChild.SetParent(root.transform);
                    yChild.SetParent(root.transform);
                    zChild.SetParent(yChild);
                    Debug.Log($"[DashboardWiring] {machineName}: CoastRunner chain — Root→X, Root→Y→Z");
                }
                else
                {
                    // Bantam: Y rides on X, Z independent
                    xChild.SetParent(root.transform);
                    yChild.SetParent(xChild);
                    zChild.SetParent(root.transform);
                    Debug.Log($"[DashboardWiring] {machineName}: Bantam chain — Root→X→Y, Root→Z");
                }

                // X axis (prismatic along Unity X)
                var xAB = xChild.GetComponent<ArticulationBody>() ?? xChild.gameObject.AddComponent<ArticulationBody>();
                xAB.jointType = ArticulationJointType.PrismaticJoint;
                xAB.linearLockX = ArticulationDofLock.LimitedMotion;
                xAB.linearLockY = ArticulationDofLock.LockedMotion;
                xAB.linearLockZ = ArticulationDofLock.LockedMotion;
                var xDrive = xAB.xDrive;
                xDrive.lowerLimit = 0f;
                xDrive.upperLimit = xTravel;
                xDrive.stiffness = 100000f;
                xDrive.damping = 10000f;
                xDrive.forceLimit = float.MaxValue;
                xAB.xDrive = xDrive;

                // Y axis (prismatic along Unity Z = machine Y front-back)
                var yAB = yChild.GetComponent<ArticulationBody>() ?? yChild.gameObject.AddComponent<ArticulationBody>();
                yAB.jointType = ArticulationJointType.PrismaticJoint;
                yAB.linearLockX = ArticulationDofLock.LockedMotion;
                yAB.linearLockY = ArticulationDofLock.LockedMotion;
                yAB.linearLockZ = ArticulationDofLock.LimitedMotion;
                var yDrive = yAB.xDrive;
                yDrive.lowerLimit = 0f;
                yDrive.upperLimit = yTravel;
                yDrive.stiffness = 100000f;
                yDrive.damping = 10000f;
                yDrive.forceLimit = float.MaxValue;
                yAB.xDrive = yDrive;

                // Z axis (prismatic along Unity -Y = machine Z downward)
                var zAB = zChild.GetComponent<ArticulationBody>() ?? zChild.gameObject.AddComponent<ArticulationBody>();
                zAB.jointType = ArticulationJointType.PrismaticJoint;
                zAB.linearLockX = ArticulationDofLock.LockedMotion;
                zAB.linearLockY = ArticulationDofLock.LimitedMotion;
                zAB.linearLockZ = ArticulationDofLock.LockedMotion;
                var zDrive = zAB.xDrive;
                zDrive.lowerLimit = -zTravel;
                zDrive.upperLimit = 0f;
                zDrive.stiffness = 100000f;
                zDrive.damping = 10000f;
                zDrive.forceLimit = float.MaxValue;
                zAB.xDrive = zDrive;

                Debug.Log($"[DashboardWiring] {machineName}: Wired ArticulationBody joints on FBX children.");
                LogHierarchy(root.transform, 0);
                return;
            }

            // --- Fallback: create invisible overlay GOs ---
            Debug.Log($"[DashboardWiring] Creating articulated overlay for {machineName}: " +
                $"X={maxMM.x}mm, Y={maxMM.y}mm, Z={maxMM.z}mm");

            // X_Gantry (prismatic along Unity X)
            var xGantry = new GameObject("X_Gantry");
            xGantry.transform.SetParent(root.transform);
            xGantry.transform.localPosition = Vector3.zero;
            xGantry.transform.localRotation = Quaternion.identity;
            var xGantryAB = xGantry.AddComponent<ArticulationBody>();
            xGantryAB.jointType = ArticulationJointType.PrismaticJoint;
            xGantryAB.linearLockX = ArticulationDofLock.LimitedMotion;
            xGantryAB.linearLockY = ArticulationDofLock.LockedMotion;
            xGantryAB.linearLockZ = ArticulationDofLock.LockedMotion;
            var xGDrive = xGantryAB.xDrive;
            xGDrive.lowerLimit = 0f;
            xGDrive.upperLimit = xTravel;
            xGDrive.stiffness = 100000f;
            xGDrive.damping = 10000f;
            xGDrive.forceLimit = float.MaxValue;
            xGantryAB.xDrive = xGDrive;

            // Y_Table (prismatic along Unity Z = machine Y front-back)
            var yTable = new GameObject("Y_Table");
            yTable.transform.SetParent(xGantry.transform);
            yTable.transform.localPosition = Vector3.zero;
            yTable.transform.localRotation = Quaternion.identity;
            var yTableAB = yTable.AddComponent<ArticulationBody>();
            yTableAB.jointType = ArticulationJointType.PrismaticJoint;
            yTableAB.linearLockX = ArticulationDofLock.LockedMotion;
            yTableAB.linearLockY = ArticulationDofLock.LockedMotion;
            yTableAB.linearLockZ = ArticulationDofLock.LimitedMotion;
            var yTDrive = yTableAB.xDrive;
            yTDrive.lowerLimit = 0f;
            yTDrive.upperLimit = yTravel;
            yTDrive.stiffness = 100000f;
            yTDrive.damping = 10000f;
            yTDrive.forceLimit = float.MaxValue;
            yTableAB.xDrive = yTDrive;

            // Z_Head (prismatic along Unity -Y = machine Z downward)
            var zHead = new GameObject("Z_Head");
            zHead.transform.SetParent(yTable.transform);
            zHead.transform.localPosition = Vector3.zero;
            zHead.transform.localRotation = Quaternion.identity;
            var zHeadAB = zHead.AddComponent<ArticulationBody>();
            zHeadAB.jointType = ArticulationJointType.PrismaticJoint;
            zHeadAB.linearLockX = ArticulationDofLock.LockedMotion;
            zHeadAB.linearLockY = ArticulationDofLock.LimitedMotion;
            zHeadAB.linearLockZ = ArticulationDofLock.LockedMotion;
            var zHDrive = zHeadAB.xDrive;
            zHDrive.lowerLimit = -zTravel;
            zHDrive.upperLimit = 0f;
            zHDrive.stiffness = 100000f;
            zHDrive.damping = 10000f;
            zHDrive.forceLimit = float.MaxValue;
            zHeadAB.xDrive = zHDrive;

            Undo.RegisterCreatedObjectUndo(xGantry, $"Create {machineName} articulated overlay");

            Debug.Log($"[DashboardWiring] {machineName} hierarchy after overlay:");
            LogHierarchy(root.transform, 0);
        }

        /// <summary>
        /// Find the best child transform to use as the movable head / gantry fallback.
        /// </summary>
        private static Transform FindMovableHead(Transform root)
        {
            string[] headNames = { "head", "spindle", "gantry", "carriage", "slide",
                                    "toolholder", "z_axis", "z-axis", "zaxis",
                                    "headstock", "quill", "ram", "z_head",
                                    "subassembly z" };
            string[] excludePatterns = { "header", "pin", "screw", "bolt", "nut", "washer",
                                          "connector", "pcb", "cable", "wire", "led", "label" };

            Transform bestMatch = null;
            float bestVolume = 0f;

            foreach (Transform child in root.GetComponentsInChildren<Transform>())
            {
                if (child == root) continue;
                var mr = child.GetComponentInChildren<MeshRenderer>();
                if (mr == null) continue;

                string lower = child.name.ToLowerInvariant();

                bool excluded = false;
                foreach (var ex in excludePatterns)
                    if (lower.Contains(ex)) { excluded = true; break; }
                if (excluded) continue;

                foreach (var name in headNames)
                {
                    if (lower.Contains(name))
                    {
                        float vol = mr.bounds.size.x * mr.bounds.size.y * mr.bounds.size.z;
                        if (bestMatch == null || vol > bestVolume)
                        {
                            bestMatch = child;
                            bestVolume = vol;
                        }
                        break;
                    }
                }
            }

            if (bestMatch != null) return bestMatch;

            // Fallback: largest mesh child (CNC body / spindle assembly)
            Transform largest = null;
            float largestVol = 0f;
            foreach (var mr in root.GetComponentsInChildren<MeshRenderer>())
            {
                if (mr.transform == root) continue;
                float vol = mr.bounds.size.x * mr.bounds.size.y * mr.bounds.size.z;
                if (vol > largestVol)
                {
                    largestVol = vol;
                    largest = mr.transform;
                }
            }

            if (largest == null && root.childCount > 0)
                largest = root.GetChild(0);

            return largest;
        }

        private static void LogHierarchy(Transform t, int depth)
        {
            string indent = new string(' ', depth * 2);
            string components = "";
            var ab = t.GetComponent<ArticulationBody>();
            if (ab != null)
                components += $" [AB: {ab.jointType}]";
            var meshFilter = t.GetComponent<MeshFilter>();
            if (meshFilter != null && meshFilter.sharedMesh != null)
                components += $" [Mesh: {meshFilter.sharedMesh.name}]";
            Debug.Log($"  {indent}{t.name}{components}");

            foreach (Transform child in t)
                LogHierarchy(child, depth + 1);
        }

        /// <summary>
        /// Wire ForceChart, ThermalChart, ToolWearChart, and PowerChart onto the DashboardUI.
        /// Each chart finds its target VisualElement by name in the UIDocument.
        /// </summary>
        private static void WireLiveCharts(GameObject dashboardGO, UIDocument uiDocument,
            CuttingStateEventSO cuttingStateEvent)
        {
            if (cuttingStateEvent == null)
            {
                Debug.LogWarning("[DashboardWiring] CuttingStateEvent missing — cannot wire charts.");
                return;
            }

            // ForceChart
            var forceChart = dashboardGO.GetComponent<ForceChart>();
            if (forceChart == null)
                forceChart = dashboardGO.AddComponent<ForceChart>();
            var fcSO = new SerializedObject(forceChart);
            fcSO.FindProperty("cuttingStateEvent").objectReferenceValue = cuttingStateEvent;
            fcSO.FindProperty("uiDocument").objectReferenceValue = uiDocument;
            fcSO.ApplyModifiedProperties();

            // ThermalChart
            var thermalChart = dashboardGO.GetComponent<ThermalChart>();
            if (thermalChart == null)
                thermalChart = dashboardGO.AddComponent<ThermalChart>();
            var tcSO = new SerializedObject(thermalChart);
            tcSO.FindProperty("cuttingStateEvent").objectReferenceValue = cuttingStateEvent;
            tcSO.FindProperty("uiDocument").objectReferenceValue = uiDocument;
            tcSO.ApplyModifiedProperties();

            // ToolWearChart
            var toolWearChart = dashboardGO.GetComponent<ToolWearChart>();
            if (toolWearChart == null)
                toolWearChart = dashboardGO.AddComponent<ToolWearChart>();
            var twSO = new SerializedObject(toolWearChart);
            twSO.FindProperty("cuttingStateEvent").objectReferenceValue = cuttingStateEvent;
            twSO.FindProperty("uiDocument").objectReferenceValue = uiDocument;
            twSO.ApplyModifiedProperties();

            // PowerChart
            var powerChart = dashboardGO.GetComponent<PowerChart>();
            if (powerChart == null)
                powerChart = dashboardGO.AddComponent<PowerChart>();
            var pcSO = new SerializedObject(powerChart);
            pcSO.FindProperty("cuttingStateEvent").objectReferenceValue = cuttingStateEvent;
            pcSO.FindProperty("uiDocument").objectReferenceValue = uiDocument;
            pcSO.ApplyModifiedProperties();

            Debug.Log("[DashboardWiring] Wired 4 live charts (Force, Thermal, ToolWear, Power) on DashboardUI.");
        }

        /// <summary>
        /// Create a Workpiece child GameObject under the CNC machine with VoxelWorkpiece component.
        /// Assigns compute shaders and creates a ProceduralWorkpiece material instance.
        /// </summary>
        private static void WireVoxelWorkpiece(GameObject machineGO, string machineName)
        {
            // Check if VoxelWorkpiece already exists on this machine
            var existing = machineGO.GetComponentInChildren<VoxelWorkpiece>();
            if (existing != null)
            {
                Debug.Log($"[DashboardWiring] VoxelWorkpiece already exists on {machineName} — skipping.");
                return;
            }

            // Load compute shaders
            var subtractShader = AssetDatabase.LoadAssetAtPath<ComputeShader>("Assets/Compute/VoxelSubtraction.compute");
            var marchingCubesShader = AssetDatabase.LoadAssetAtPath<ComputeShader>("Assets/Compute/MarchingCubes.compute");
            var engagementShader = AssetDatabase.LoadAssetAtPath<ComputeShader>("Assets/Compute/VoxelEngagement.compute");

            if (subtractShader == null || marchingCubesShader == null)
            {
                Debug.LogWarning($"[DashboardWiring] Compute shaders not found — cannot create VoxelWorkpiece on {machineName}. " +
                    "Ensure Assets/Compute/VoxelSubtraction.compute and MarchingCubes.compute exist.");
                return;
            }

            // Load or create the procedural workpiece material
            const string materialPath = "Assets/Materials/ProceduralWorkpiece.mat";
            Material workpieceMat = AssetDatabase.LoadAssetAtPath<Material>(materialPath);
            if (workpieceMat == null)
            {
                var shader = Shader.Find("MIRACLE/ProceduralWorkpiece");
                if (shader == null)
                {
                    Debug.LogWarning("[DashboardWiring] Shader 'MIRACLE/ProceduralWorkpiece' not found. " +
                        "Ensure Assets/Shaders/ProceduralWorkpiece.shader exists and compiles.");
                    return;
                }
                workpieceMat = new Material(shader);
                workpieceMat.name = "ProceduralWorkpiece";
                EnsureFolderExists("Assets/Materials");
                AssetDatabase.CreateAsset(workpieceMat, materialPath);
                AssetDatabase.SaveAssets();
                Debug.Log($"[DashboardWiring] Created ProceduralWorkpiece material at {materialPath}.");
            }

            // Create Workpiece child GameObject
            Transform workpieceTransform = machineGO.transform.Find("Workpiece");
            GameObject workpieceGO;
            if (workpieceTransform != null)
            {
                workpieceGO = workpieceTransform.gameObject;
            }
            else
            {
                workpieceGO = new GameObject("Workpiece");
                workpieceGO.transform.SetParent(machineGO.transform);
                workpieceGO.transform.localPosition = Vector3.zero;
                workpieceGO.transform.localRotation = Quaternion.identity;
                Undo.RegisterCreatedObjectUndo(workpieceGO, $"Create Workpiece ({machineName})");
            }

            // Add VoxelWorkpiece component
            var voxelWorkpiece = workpieceGO.GetComponent<VoxelWorkpiece>();
            if (voxelWorkpiece == null)
                voxelWorkpiece = workpieceGO.AddComponent<VoxelWorkpiece>();

            var vwSO = new SerializedObject(voxelWorkpiece);
            vwSO.FindProperty("subtractShader").objectReferenceValue = subtractShader;
            vwSO.FindProperty("marchingCubesShader").objectReferenceValue = marchingCubesShader;
            vwSO.FindProperty("engagementShader").objectReferenceValue = engagementShader;
            vwSO.FindProperty("workpieceMaterial").objectReferenceValue = workpieceMat;
            vwSO.ApplyModifiedProperties();

            Debug.Log($"[DashboardWiring] Created VoxelWorkpiece on {machineName}/Workpiece with compute shaders + ProceduralWorkpiece material.");
        }

        /// <summary>
        /// Wire HeatMapOverlay onto a CNC machine. Creates a child MeshRenderer quad
        /// that the overlay shader renders onto, positioned over the workpiece area.
        /// </summary>
        private static void WireHeatMapOverlay(GameObject machineGO, CuttingStateEventSO cuttingStateEvent)
        {
            var existing = machineGO.GetComponentInChildren<HeatMapOverlay>();
            if (existing != null) return;

            // Load heat map material
            const string heatMatPath = "Assets/Materials/WorkpieceHeatMap.mat";
            var heatShader = Shader.Find("MIRACLE/WorkpieceHeatMap");
            Material heatMat = AssetDatabase.LoadAssetAtPath<Material>(heatMatPath);
            if (heatMat == null && heatShader != null)
            {
                heatMat = new Material(heatShader);
                heatMat.name = "WorkpieceHeatMap";
                EnsureFolderExists("Assets/Materials");
                AssetDatabase.CreateAsset(heatMat, heatMatPath);
                AssetDatabase.SaveAssets();
                Debug.Log($"[DashboardWiring] Created WorkpieceHeatMap material at {heatMatPath}.");
            }

            if (heatMat == null)
            {
                Debug.LogWarning("[DashboardWiring] WorkpieceHeatMap material/shader not available — skipping HeatMapOverlay.");
                return;
            }

            // Find the workpiece renderer (VoxelWorkpiece child, or any mesh on the machine)
            Renderer workpieceRenderer = null;
            var voxelWP = machineGO.GetComponentInChildren<VoxelWorkpiece>();
            if (voxelWP != null)
                workpieceRenderer = voxelWP.GetComponent<Renderer>();
            if (workpieceRenderer == null)
                workpieceRenderer = machineGO.GetComponentInChildren<MeshRenderer>();

            // Create HeatMapOverlay on the machine root
            var heatMapOverlay = machineGO.AddComponent<HeatMapOverlay>();
            var cuttingSimManager = machineGO.GetComponent<CuttingSimulationManager>();

            var hmSO = new SerializedObject(heatMapOverlay);
            hmSO.FindProperty("cuttingStateEvent").objectReferenceValue = cuttingStateEvent;
            hmSO.FindProperty("heatMapMaterial").objectReferenceValue = heatMat;
            if (workpieceRenderer != null)
                hmSO.FindProperty("workpieceRenderer").objectReferenceValue = workpieceRenderer;
            if (cuttingSimManager != null)
                hmSO.FindProperty("cuttingSimManager").objectReferenceValue = cuttingSimManager;
            hmSO.ApplyModifiedProperties();

            Debug.Log($"[DashboardWiring] Wired HeatMapOverlay on {machineGO.name}.");
        }

        /// <summary>
        /// Wire ForceArrowRenderer onto a CNC machine for 3D force vector visualization.
        /// </summary>
        private static void WireForceArrowRenderer(GameObject machineGO, CuttingStateEventSO cuttingStateEvent)
        {
            var existing = machineGO.GetComponentInChildren<ForceArrowRenderer>();
            if (existing != null) return;

            // Load force arrow material
            Material arrowMat = AssetDatabase.LoadAssetAtPath<Material>("Assets/Materials/ForceArrow.mat");
            if (arrowMat == null)
            {
                var arrowShader = Shader.Find("MIRACLE/ForceArrow");
                if (arrowShader != null)
                {
                    arrowMat = new Material(arrowShader);
                    arrowMat.name = "ForceArrow";
                    EnsureFolderExists("Assets/Materials");
                    AssetDatabase.CreateAsset(arrowMat, "Assets/Materials/ForceArrow.mat");
                    AssetDatabase.SaveAssets();
                }
            }

            // Create ForceArrows child
            Transform faTransform = machineGO.transform.Find("ForceArrows");
            GameObject faGO;
            if (faTransform != null)
                faGO = faTransform.gameObject;
            else
            {
                faGO = new GameObject("ForceArrows");
                faGO.transform.SetParent(machineGO.transform);
                faGO.transform.localPosition = Vector3.zero;
                Undo.RegisterCreatedObjectUndo(faGO, $"Create ForceArrows ({machineGO.name})");
            }

            var forceArrowRenderer = faGO.GetComponent<ForceArrowRenderer>();
            if (forceArrowRenderer == null)
                forceArrowRenderer = faGO.AddComponent<ForceArrowRenderer>();

            var faSO = new SerializedObject(forceArrowRenderer);
            faSO.FindProperty("cuttingStateEvent").objectReferenceValue = cuttingStateEvent;
            if (arrowMat != null)
                faSO.FindProperty("arrowMaterial").objectReferenceValue = arrowMat;
            faSO.ApplyModifiedProperties();

            Debug.Log($"[DashboardWiring] Wired ForceArrowRenderer on {machineGO.name}/ForceArrows.");
        }

        private static GameObject FindByNames(string[] names)
        {
            foreach (string name in names)
            {
                GameObject go = GameObject.Find(name);
                if (go != null) return go;
            }
            return null;
        }

        private static void EnsureFolderExists(string folderPath)
        {
            string[] parts = folderPath.Split('/');
            string current = parts[0];

            for (int i = 1; i < parts.Length; i++)
            {
                string next = current + "/" + parts[i];
                if (!AssetDatabase.IsValidFolder(next))
                {
                    AssetDatabase.CreateFolder(current, parts[i]);
                }
                current = next;
            }
        }
    }
}
#endif
