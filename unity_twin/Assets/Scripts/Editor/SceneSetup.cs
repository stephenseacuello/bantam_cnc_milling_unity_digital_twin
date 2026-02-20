#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

namespace MiracleTwin.Editor
{
    /// <summary>
    /// One-click scene setup: finds imported URDF robots and FBX models,
    /// fixes physics, positions everything, and adds manager scripts.
    /// </summary>
    public static class SceneSetup
    {
        [MenuItem("MIRACLE/Setup Scene From Imported Models")]
        public static void SetupScene()
        {
            // ---- Find imported robots and CNC ----
            var ned2 = FindRootByName("niryo_ned2");
            var lite6 = FindRootByName("lite6");

            // Find the Bantam FBX - look for the imported FBX model instance
            GameObject bantam = null;
            // Check common names from FBX import
            var names = new[] {
                "Arduino Bantam CNC G-Code Fingerprinting",
                "BantamExplorerCNC",
                "Bantam Tools Explorer CNC Milling Machine",
                "BantamExplorer"
            };
            foreach (var n in names)
            {
                bantam = FindRootByName(n);
                if (bantam != null) break;
            }

            Debug.Log($"[SceneSetup] Found: Ned2={ned2 != null}, Lite6={lite6 != null}, Bantam={bantam != null}");

            // ---- Fix robot physics (prevent falling) ----
            if (ned2 != null) FixRobotPhysics(ned2);
            if (lite6 != null) FixRobotPhysics(lite6);

            // ---- Position everything ----
            if (bantam != null)
            {
                // Don't reposition or rescale the Bantam if user already placed it
                // Just log its current position for reference
                Debug.Log($"[SceneSetup] Bantam found at position {bantam.transform.position}, scale {bantam.transform.localScale}");
                Debug.Log($"[SceneSetup] Bantam NOT moved (keeping user placement). Reposition manually if needed.");
            }

            if (ned2 != null)
            {
                // Ned2 to the left of CNC
                ned2.transform.position = new Vector3(-0.4f, 0f, 0f);
                ned2.transform.rotation = Quaternion.identity;
            }

            if (lite6 != null)
            {
                // xArm Lite6 to the right of CNC
                lite6.transform.position = new Vector3(0.4f, 0f, 0f);
                lite6.transform.rotation = Quaternion.identity;
            }

            // ---- Create environment if not present ----
            if (GameObject.Find("Environment") == null)
            {
                var env = new GameObject("Environment");

                // Floor
                var floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
                floor.name = "Floor";
                floor.transform.SetParent(env.transform);
                floor.transform.localPosition = new Vector3(0, -0.001f, 0);
                floor.transform.localScale = new Vector3(0.3f, 1f, 0.3f);

                // Workbench
                var bench = GameObject.CreatePrimitive(PrimitiveType.Cube);
                bench.name = "Workbench";
                bench.transform.SetParent(env.transform);
                bench.transform.localPosition = new Vector3(0, -0.025f, 0);
                bench.transform.localScale = new Vector3(1.5f, 0.05f, 0.8f);

                Undo.RegisterCreatedObjectUndo(env, "Create Environment");
            }

            // ---- Create managers if not present ----
            EnsureManagers();

            // ---- Setup lighting if only default exists ----
            SetupLighting();

            // ---- Set camera ----
            var mainCam = Camera.main;
            if (mainCam != null)
            {
                mainCam.transform.position = new Vector3(0.6f, 0.5f, 0.6f);
                mainCam.transform.LookAt(new Vector3(0f, 0.15f, 0f));
            }

            Debug.Log("[SceneSetup] Scene setup complete!");
            Debug.Log("  - Robot physics fixed (immovable roots, no gravity)");
            Debug.Log("  - Models positioned (Ned2 left, CNC center, xArm right)");
            Debug.Log("  - Press Play to test!");
        }

        [MenuItem("MIRACLE/Fix Robot Physics Only")]
        public static void FixPhysicsOnly()
        {
            int fixed_count = 0;
            // Find all ArticulationBody roots and make them immovable
            var allABs = Object.FindObjectsByType<ArticulationBody>(FindObjectsSortMode.None);
            foreach (var ab in allABs)
            {
                // Disable gravity on ALL articulation bodies
                if (ab.useGravity)
                {
                    Undo.RecordObject(ab, "Disable Gravity");
                    ab.useGravity = false;
                    fixed_count++;
                }

                // Make root bodies immovable
                if (ab.isRoot)
                {
                    Undo.RecordObject(ab, "Make Immovable");
                    ab.immovable = true;
                    fixed_count++;
                }
            }
            Debug.Log($"[SceneSetup] Fixed physics on {fixed_count} ArticulationBody components");
        }

        private static void FixRobotPhysics(GameObject robot)
        {
            var articulationBodies = robot.GetComponentsInChildren<ArticulationBody>();
            foreach (var ab in articulationBodies)
            {
                Undo.RecordObject(ab, "Fix Robot Physics");
                ab.useGravity = false;

                if (ab.isRoot)
                {
                    ab.immovable = true;
                }
            }
            Debug.Log($"[SceneSetup] Fixed physics on {robot.name}: {articulationBodies.Length} bodies, gravity off, root immovable");
        }

        private static GameObject FindRootByName(string name)
        {
            // Search all root GameObjects in the scene
            var roots = UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects();
            foreach (var root in roots)
            {
                if (root.name == name) return root;
                // Also check children one level deep
                foreach (Transform child in root.transform)
                {
                    if (child.name == name) return child.gameObject;
                }
            }
            // Fallback: search everywhere
            return GameObject.Find(name);
        }

        private static void EnsureManagers()
        {
            if (GameObject.Find("_Managers") != null) return;

            var managers = new GameObject("_Managers");

            var clockGO = new GameObject("SimulationClock");
            clockGO.transform.SetParent(managers.transform);
            clockGO.AddComponent<Core.SimulationClock>();

            var dispatcherGO = new GameObject("MessageDispatcher");
            dispatcherGO.transform.SetParent(managers.transform);
            dispatcherGO.AddComponent<Core.MessageDispatcher>();

            var bridgeGO = new GameObject("MiracleBridge");
            bridgeGO.transform.SetParent(managers.transform);
            bridgeGO.AddComponent<Core.MiracleBridge>();

            var inputGO = new GameObject("InputManager");
            inputGO.transform.SetParent(managers.transform);
            inputGO.AddComponent<Core.InputManager>();

            Undo.RegisterCreatedObjectUndo(managers, "Create Managers");
            Debug.Log("[SceneSetup] Created _Managers (Clock, Bridge, Input)");
        }

        private static void SetupLighting()
        {
            if (GameObject.Find("Lighting_Setup") != null) return;

            var lightRoot = new GameObject("Lighting_Setup");

            // Key light
            var keyGO = new GameObject("Key_Light");
            keyGO.transform.SetParent(lightRoot.transform);
            keyGO.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            var keyLight = keyGO.AddComponent<Light>();
            keyLight.type = LightType.Directional;
            keyLight.intensity = 1.2f;
            keyLight.color = new Color(1f, 0.97f, 0.92f);
            keyLight.shadows = LightShadows.Soft;

            // Fill light
            var fillGO = new GameObject("Fill_Light");
            fillGO.transform.SetParent(lightRoot.transform);
            fillGO.transform.rotation = Quaternion.Euler(30f, 150f, 0f);
            var fillLight = fillGO.AddComponent<Light>();
            fillLight.type = LightType.Directional;
            fillLight.intensity = 0.4f;
            fillLight.color = new Color(0.85f, 0.9f, 1f);

            Undo.RegisterCreatedObjectUndo(lightRoot, "Create Lighting");
        }
    }
}
#endif
