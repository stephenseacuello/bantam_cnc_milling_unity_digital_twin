using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace MiracleTwin.CNC
{
    /// <summary>
    /// Editor utility to procedurally build the CoastRunner CR-1 CNC
    /// with correct dimensions, ArticulationBody joints, and hierarchy.
    ///
    /// Machine dimensions (meters):
    ///   Overall: ~0.508 x 0.406 x 0.330 (W x H x D)
    ///   Work volume: 0.089 x 0.242 x 0.079 (X x Y x Z)
    ///   Horizontal form factor with VFD spindle 1500-8000 RPM, ER-11
    ///
    /// Use this when no FBX model is available, or to create a quick
    /// placeholder with proper physics joints. When using the imported
    /// FBX model instead, use "MIRACLE/Setup CR-1 From FBX" menu item.
    /// </summary>
    public static class CoastRunnerCR1Builder
    {
#if UNITY_EDITOR
        [MenuItem("MIRACLE/Build CoastRunner CR-1 CNC (Procedural)")]
        public static void BuildCNC()
        {
            var root = new GameObject("CoastRunnerCR1");
            root.transform.position = Vector3.zero;

            // Base Frame (static body) — horizontal orientation
            var baseFrame = CreateBox("Base_Frame", root.transform,
                new Vector3(0, 0.075f, 0), new Vector3(0.508f, 0.150f, 0.330f),
                "MachineBodyDarkGray");
            var baseRB = baseFrame.AddComponent<ArticulationBody>();
            baseRB.immovable = true;

            // Enclosure walls
            CreateBox("Enclosure_Back", baseFrame.transform,
                new Vector3(0, 0.128f, -0.1625f), new Vector3(0.508f, 0.256f, 0.005f),
                "MachineBodyDarkGray");
            CreateBox("Enclosure_Left", baseFrame.transform,
                new Vector3(-0.2515f, 0.128f, 0), new Vector3(0.005f, 0.256f, 0.330f),
                "MachineBodyDarkGray");
            CreateBox("Enclosure_Right", baseFrame.transform,
                new Vector3(0.2515f, 0.128f, 0), new Vector3(0.005f, 0.256f, 0.330f),
                "MachineBodyDarkGray");

            // Enclosure Lid
            var lid = CreateBox("Enclosure_Lid", baseFrame.transform,
                new Vector3(0, 0.256f, 0), new Vector3(0.508f, 0.005f, 0.330f),
                "MachineBodyDarkGray");
            lid.AddComponent<EnclosureLid>();

            // X-Axis Gantry (prismatic along X) — 89mm travel
            var xGantry = CreateBox("X_Gantry", baseFrame.transform,
                new Vector3(0, 0.160f, 0), new Vector3(0.120f, 0.020f, 0.200f),
                "MachineBodyDarkGray");
            var xAB = xGantry.AddComponent<ArticulationBody>();
            xAB.jointType = ArticulationJointType.PrismaticJoint;
            xAB.linearLockX = ArticulationDofLock.LimitedMotion;
            xAB.linearLockY = ArticulationDofLock.LockedMotion;
            xAB.linearLockZ = ArticulationDofLock.LockedMotion;
            var xDrive = xAB.xDrive;
            xDrive.lowerLimit = 0f;
            xDrive.upperLimit = 0.089f; // 89mm
            xDrive.stiffness = 100000f;
            xDrive.damping = 10000f;
            xDrive.forceLimit = float.MaxValue;
            xAB.xDrive = xDrive;

            // Y-Table (prismatic along Z in Unity = Y in machine coords) — 242mm travel
            var yTable = CreateBox("Y_Table", xGantry.transform,
                new Vector3(0, -0.015f, 0), new Vector3(0.100f, 0.010f, 0.260f),
                "ViseSteel");
            var yAB = yTable.AddComponent<ArticulationBody>();
            yAB.jointType = ArticulationJointType.PrismaticJoint;
            yAB.linearLockX = ArticulationDofLock.LockedMotion;
            yAB.linearLockY = ArticulationDofLock.LockedMotion;
            yAB.linearLockZ = ArticulationDofLock.LimitedMotion;
            var yDrive = yAB.xDrive;
            yDrive.lowerLimit = 0f;
            yDrive.upperLimit = 0.242f; // 242mm
            yDrive.stiffness = 100000f;
            yDrive.damping = 10000f;
            yDrive.forceLimit = float.MaxValue;
            yAB.xDrive = yDrive;

            // Vise on Y-Table
            CreateBox("Vise_Fixed_Jaw", yTable.transform,
                new Vector3(0, 0.010f, -0.050f), new Vector3(0.070f, 0.020f, 0.008f),
                "ViseSteel");
            CreateBox("Vise_Moving_Jaw", yTable.transform,
                new Vector3(0, 0.010f, 0.050f), new Vector3(0.070f, 0.020f, 0.008f),
                "ViseSteel");

            // Workpiece mount point
            var mountPoint = new GameObject("WorkpieceMountPoint");
            mountPoint.transform.SetParent(yTable.transform);
            mountPoint.transform.localPosition = new Vector3(0, 0.020f, 0);

            // Z-Column (static pillar attached to base)
            var zColumn = CreateBox("Z_Column", baseFrame.transform,
                new Vector3(0, 0.200f, -0.140f), new Vector3(0.060f, 0.160f, 0.040f),
                "MachineBodyDarkGray");

            // Z-Head (prismatic along -Y in Unity = Z down in machine) — 79mm travel
            var zHead = CreateBox("Z_Head", zColumn.transform,
                new Vector3(0, -0.020f, 0.050f), new Vector3(0.050f, 0.070f, 0.050f),
                "MachineBodyDarkGray");
            var zAB = zHead.AddComponent<ArticulationBody>();
            zAB.jointType = ArticulationJointType.PrismaticJoint;
            zAB.linearLockX = ArticulationDofLock.LockedMotion;
            zAB.linearLockY = ArticulationDofLock.LimitedMotion;
            zAB.linearLockZ = ArticulationDofLock.LockedMotion;
            var zDrive = zAB.xDrive;
            zDrive.lowerLimit = -0.079f; // 79mm
            zDrive.upperLimit = 0f;
            zDrive.stiffness = 100000f;
            zDrive.damping = 10000f;
            zDrive.forceLimit = float.MaxValue;
            zAB.xDrive = zDrive;

            // Spindle Motor (VFD 1500-8000 RPM)
            var spindleMotor = CreateBox("Spindle_Motor", zHead.transform,
                new Vector3(0, -0.045f, 0), new Vector3(0.035f, 0.055f, 0.035f),
                "MachineBodyDarkGray");

            // Collet (ER-11)
            var collet = new GameObject("Collet_ER11");
            collet.transform.SetParent(spindleMotor.transform);
            collet.transform.localPosition = new Vector3(0, -0.030f, 0);

            // End Mill (1/4" = 6.35mm diameter, 25mm cutting length)
            var endMill = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            endMill.name = "EndMill";
            endMill.transform.SetParent(collet.transform);
            endMill.transform.localPosition = new Vector3(0, -0.0125f, 0);
            endMill.transform.localScale = new Vector3(0.00635f, 0.0125f, 0.00635f);

            // Add controller component
            root.AddComponent<CoastRunnerCR1Controller>();

            // Add SpindleVisualizer
            collet.AddComponent<SpindleVisualizer>();

            // Add WorkpieceManager
            root.AddComponent<WorkpieceManager>();

            // Add ViseController
            yTable.AddComponent<ViseController>();

            Debug.Log("[CoastRunnerCR1Builder] CR-1 CNC model built successfully (procedural)");

            Selection.activeGameObject = root;
            Undo.RegisterCreatedObjectUndo(root, "Build CoastRunner CR-1 CNC");
        }

        [MenuItem("MIRACLE/Setup CR-1 From FBX")]
        public static void SetupFromFBX()
        {
            // Find the imported CR-1 model in the scene
            GameObject cr1Root = null;

            // First check if user has selected a GameObject
            if (Selection.activeGameObject != null)
            {
                cr1Root = Selection.activeGameObject;
                Debug.Log($"[CoastRunnerCR1Builder] Using selected GameObject: {cr1Root.name}");
            }
            else
            {
                // Try to find by common names
                string[] searchNames = { "CoastRunnerCR1", "coast_runner", "CR-1", "CR1" };
                foreach (string name in searchNames)
                {
                    cr1Root = GameObject.Find(name);
                    if (cr1Root != null) break;
                }
            }

            if (cr1Root == null)
            {
                Debug.LogError("[CoastRunnerCR1Builder] No CR-1 model found. " +
                    "Either select the CR-1 root GameObject or drag the FBX into the scene first.");
                return;
            }

            // Add controller if missing
            var controller = cr1Root.GetComponent<CoastRunnerCR1Controller>();
            if (controller == null)
                controller = cr1Root.AddComponent<CoastRunnerCR1Controller>();

            // Add WorkpieceManager if missing
            if (cr1Root.GetComponent<WorkpieceManager>() == null)
                cr1Root.AddComponent<WorkpieceManager>();

            // Log the hierarchy for manual joint assignment
            Debug.Log("[CoastRunnerCR1Builder] CR-1 FBX hierarchy:");
            LogHierarchy(cr1Root.transform, 0);

            Debug.Log("[CoastRunnerCR1Builder] Setup complete. " +
                "Assign ArticulationBody joints to X/Y/Z axis transforms " +
                "and SpindleRotor/EndMill transforms in the Inspector.");

            Undo.RegisterCompleteObjectUndo(cr1Root, "Setup CR-1 From FBX");
            EditorUtility.SetDirty(cr1Root);
        }

        private static void LogHierarchy(Transform t, int depth)
        {
            string indent = new string(' ', depth * 2);
            string components = "";
            var meshFilter = t.GetComponent<MeshFilter>();
            if (meshFilter != null && meshFilter.sharedMesh != null)
                components = $" [Mesh: {meshFilter.sharedMesh.name}]";
            Debug.Log($"  {indent}{t.name}{components}");

            foreach (Transform child in t)
                LogHierarchy(child, depth + 1);
        }

        private static GameObject CreateBox(string name, Transform parent,
            Vector3 localPos, Vector3 size, string materialName)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            go.transform.SetParent(parent);
            go.transform.localPosition = localPos;
            go.transform.localScale = size;
            go.transform.localRotation = Quaternion.identity;
            return go;
        }
#endif
    }
}
