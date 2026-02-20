#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using MiracleTwin.Core;
using MiracleTwin.CNC;
using MiracleTwin.Cutting;
using UnityEngine.VFX;

namespace MiracleTwin.Editor
{
    /// <summary>
    /// Editor utility to auto-create the full manufacturing cell scene hierarchy.
    /// MenuItem: MIRACLE > Build Manufacturing Cell Scene
    ///
    /// Creates:
    /// - Manager singletons (Clock, Bridge, Input, PerformanceMonitor)
    /// - Environment (floor, workbench, trays)
    /// - CNC machine placeholder with VoxelWorkpiece on workpiece mount point
    /// - Robot bases
    /// - Camera with CameraController
    /// - 3-point lighting (key, fill, rim) plus workshop ambient
    /// - VFX Graph objects for chip particles and coolant spray
    /// - UI root
    /// </summary>
    public static class SceneBuilder
    {
        [MenuItem("MIRACLE/Build Manufacturing Cell Scene")]
        public static void BuildScene()
        {
            // ---- Root Objects ----
            var cellRoot = new GameObject("ManufacturingCell");

            // ---- Managers ----
            var managers = new GameObject("_Managers");
            managers.transform.SetParent(cellRoot.transform);

            var clockGO = new GameObject("SimulationClock");
            clockGO.transform.SetParent(managers.transform);
            clockGO.AddComponent<SimulationClock>();

            var dispatcherGO = new GameObject("MessageDispatcher");
            dispatcherGO.transform.SetParent(managers.transform);
            dispatcherGO.AddComponent<MessageDispatcher>();

            var bridgeGO = new GameObject("MiracleBridge");
            bridgeGO.transform.SetParent(managers.transform);
            bridgeGO.AddComponent<MiracleBridge>();

            var inputGO = new GameObject("InputManager");
            inputGO.transform.SetParent(managers.transform);
            inputGO.AddComponent<InputManager>();

            var perfMonGO = new GameObject("PerformanceMonitor");
            perfMonGO.transform.SetParent(managers.transform);
            perfMonGO.AddComponent<PerformanceMonitor>();

            // ---- Environment ----
            var environment = new GameObject("Environment");
            environment.transform.SetParent(cellRoot.transform);

            // Floor
            var floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
            floor.name = "Floor";
            floor.transform.SetParent(environment.transform);
            floor.transform.localPosition = new Vector3(0, -0.05f, 0);
            floor.transform.localScale = new Vector3(0.3f, 1f, 0.3f);

            // Workbench
            var bench = GameObject.CreatePrimitive(PrimitiveType.Cube);
            bench.name = "Workbench";
            bench.transform.SetParent(environment.transform);
            bench.transform.localPosition = new Vector3(0, -0.025f, 0);
            bench.transform.localScale = new Vector3(1.5f, 0.05f, 0.8f);

            // Stock tray
            var stockTray = GameObject.CreatePrimitive(PrimitiveType.Cube);
            stockTray.name = "RawStockTray";
            stockTray.transform.SetParent(environment.transform);
            stockTray.transform.localPosition = new Vector3(-0.6f, 0.01f, -0.3f);
            stockTray.transform.localScale = new Vector3(0.2f, 0.02f, 0.15f);

            // Finished tray
            var finishedTray = GameObject.CreatePrimitive(PrimitiveType.Cube);
            finishedTray.name = "FinishedPartTray";
            finishedTray.transform.SetParent(environment.transform);
            finishedTray.transform.localPosition = new Vector3(0.6f, 0.01f, -0.3f);
            finishedTray.transform.localScale = new Vector3(0.2f, 0.02f, 0.15f);

            // ---- CNC Machine ----
            // (Use BantamExplorerBuilder.BuildCNC() for detailed model)
            var cncPlaceholder = new GameObject("BantamExplorer");
            cncPlaceholder.transform.SetParent(cellRoot.transform);
            cncPlaceholder.transform.localPosition = Vector3.zero;

            // Workpiece mount point (where the vise holds the stock)
            var workpieceMount = new GameObject("WorkpieceMount");
            workpieceMount.transform.SetParent(cncPlaceholder.transform);
            // Position at center of spoilboard, slightly above the table surface
            workpieceMount.transform.localPosition = new Vector3(0, 0.005f, 0);

            // VoxelWorkpiece on the mount point
            var workpieceGO = new GameObject("VoxelWorkpiece");
            workpieceGO.transform.SetParent(workpieceMount.transform);
            workpieceGO.transform.localPosition = Vector3.zero;
            workpieceGO.AddComponent<VoxelWorkpiece>();

            // ---- VFX Graph Objects ----
            var vfxRoot = new GameObject("VFX");
            vfxRoot.transform.SetParent(cncPlaceholder.transform);

            // Chip particles — positioned at the cutting zone
            var chipVFX = new GameObject("ChipParticles_VFX");
            chipVFX.transform.SetParent(vfxRoot.transform);
            chipVFX.transform.localPosition = new Vector3(0, 0.03f, 0);
            var chipVfxComponent = chipVFX.AddComponent<VisualEffect>();
            // VisualEffectAsset must be assigned in Inspector after importing the VFX Graph asset
            Debug.Log("  [VFX] ChipParticles_VFX created. Assign a VisualEffectAsset in Inspector.");

            // Coolant spray — positioned near the spindle nozzle area
            var coolantVFX = new GameObject("CoolantSpray_VFX");
            coolantVFX.transform.SetParent(vfxRoot.transform);
            coolantVFX.transform.localPosition = new Vector3(0.02f, 0.06f, 0);
            var coolantVfxComponent = coolantVFX.AddComponent<VisualEffect>();
            Debug.Log("  [VFX] CoolantSpray_VFX created. Assign a VisualEffectAsset in Inspector.");

            // ---- Robot Bases ----
            var ned2Base = new GameObject("Ned2_Base");
            ned2Base.transform.SetParent(cellRoot.transform);
            ned2Base.transform.localPosition = new Vector3(-0.4f, 0f, 0f);

            var xarm6Base = new GameObject("XArm6_Base");
            xarm6Base.transform.SetParent(cellRoot.transform);
            xarm6Base.transform.localPosition = new Vector3(0.4f, 0f, 0f);

            // ---- Camera ----
            var mainCam = Camera.main;
            if (mainCam == null)
            {
                var camGO = new GameObject("Main Camera");
                mainCam = camGO.AddComponent<Camera>();
                camGO.AddComponent<AudioListener>();
            }
            mainCam.transform.position = new Vector3(0.5f, 0.5f, 0.5f);
            mainCam.transform.LookAt(Vector3.zero);
            mainCam.gameObject.AddComponent<CameraController>();

            // ---- 3-Point Lighting ----
            var lightingRoot = new GameObject("Lighting");
            lightingRoot.transform.SetParent(cellRoot.transform);

            // Key Light (main directional — warm, strong, from upper-left-front)
            var keyLight = CreateDirectionalLight(
                "Key_Light", lightingRoot.transform,
                Quaternion.Euler(50f, -30f, 0f),
                intensity: 1.2f,
                color: new Color(1f, 0.97f, 0.92f) // Warm white
            );
            keyLight.GetComponent<Light>().shadows = LightShadows.Soft;

            // Fill Light (softer, from the opposite side to reduce harsh shadows)
            var fillLight = CreateDirectionalLight(
                "Fill_Light", lightingRoot.transform,
                Quaternion.Euler(30f, 150f, 0f),
                intensity: 0.4f,
                color: new Color(0.85f, 0.9f, 1f) // Cool/blue tint
            );
            fillLight.GetComponent<Light>().shadows = LightShadows.None;

            // Rim Light (backlight, from behind and above to separate subject from background)
            var rimLight = CreateDirectionalLight(
                "Rim_Light", lightingRoot.transform,
                Quaternion.Euler(20f, -160f, 0f),
                intensity: 0.6f,
                color: new Color(1f, 0.95f, 0.85f) // Warm accent
            );
            rimLight.GetComponent<Light>().shadows = LightShadows.None;

            // Workshop ambient point lights
            CreatePointLight("Workshop_Light_1", lightingRoot.transform, new Vector3(-0.3f, 0.8f, 0.2f), 0.5f);
            CreatePointLight("Workshop_Light_2", lightingRoot.transform, new Vector3(0.3f, 0.8f, -0.2f), 0.5f);

            // ---- UI ----
            var uiRoot = new GameObject("UI");
            uiRoot.transform.SetParent(cellRoot.transform);

            // Select root
            Selection.activeGameObject = cellRoot;
            Undo.RegisterCreatedObjectUndo(cellRoot, "Build Manufacturing Cell Scene");

            Debug.Log("[SceneBuilder] Manufacturing cell scene created successfully!");
            Debug.Log("  Hierarchy:");
            Debug.Log("    ManufacturingCell/");
            Debug.Log("      _Managers/ (Clock, Bridge, Input, PerformanceMonitor)");
            Debug.Log("      Environment/ (Floor, Workbench, Trays)");
            Debug.Log("      BantamExplorer/ (CNC placeholder + VoxelWorkpiece + VFX)");
            Debug.Log("      Lighting/ (Key + Fill + Rim + Workshop lights)");
            Debug.Log("      Ned2_Base/, XArm6_Base/");
            Debug.Log("      UI/");
            Debug.Log("  Next steps:");
            Debug.Log("  1. MIRACLE > Build Bantam Explorer CNC (replace placeholder)");
            Debug.Log("  2. Import Ned2 and xArm6 URDF models");
            Debug.Log("  3. Create ScriptableObject event assets (Assets > Create > MIRACLE > Events)");
            Debug.Log("  4. Assign event references in Inspector");
            Debug.Log("  5. Assign VFX Graph assets to ChipParticles_VFX and CoolantSpray_VFX");
            Debug.Log("  6. Assign compute shaders to VoxelWorkpiece component");
        }

        private static GameObject CreateDirectionalLight(string name, Transform parent,
            Quaternion rotation, float intensity, Color color)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent);
            go.transform.rotation = rotation;
            var light = go.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = intensity;
            light.color = color;
            return go;
        }

        private static void CreatePointLight(string name, Transform parent, Vector3 pos, float intensity)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent);
            go.transform.localPosition = pos;
            var light = go.AddComponent<Light>();
            light.type = LightType.Point;
            light.intensity = intensity;
            light.range = 2f;
            light.color = new Color(1f, 0.95f, 0.85f);
        }
    }
}
#endif
