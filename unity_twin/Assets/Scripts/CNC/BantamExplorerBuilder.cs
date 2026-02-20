using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace MiracleTwin.CNC
{
    /// <summary>
    /// Editor utility to procedurally build the Bantam Desktop Explorer CNC
    /// with correct dimensions, ArticulationBody joints, and hierarchy.
    ///
    /// Machine dimensions (meters):
    ///   Overall: ~0.400 × 0.387 × 0.260 (W × H × D)
    ///   Work volume: 0.1524 × 0.1016 × 0.06985 (X × Y × Z)
    /// </summary>
    public static class BantamExplorerBuilder
    {
#if UNITY_EDITOR
        [MenuItem("MIRACLE/Build Bantam Explorer CNC")]
        public static void BuildCNC()
        {
            var root = new GameObject("BantamExplorer");
            root.transform.position = Vector3.zero;

            // Base Frame (static body)
            var baseFrame = CreateBox("Base_Frame", root.transform,
                new Vector3(0, 0.100f, 0), new Vector3(0.400f, 0.200f, 0.260f),
                "MachineBodyGray");
            var baseRB = baseFrame.AddComponent<ArticulationBody>();
            baseRB.immovable = true;

            // Enclosure walls
            CreateBox("Enclosure_Back", baseFrame.transform,
                new Vector3(0, 0.194f, -0.1275f), new Vector3(0.400f, 0.387f, 0.005f),
                "MachineBodyGray");
            CreateBox("Enclosure_Left", baseFrame.transform,
                new Vector3(-0.1975f, 0.194f, 0), new Vector3(0.005f, 0.387f, 0.260f),
                "MachineBodyGray");
            CreateBox("Enclosure_Right", baseFrame.transform,
                new Vector3(0.1975f, 0.194f, 0), new Vector3(0.005f, 0.387f, 0.260f),
                "MachineBodyGray");

            // Enclosure Lid (hinged at back)
            var lid = CreateBox("Enclosure_Lid", baseFrame.transform,
                new Vector3(0, 0.387f, 0), new Vector3(0.400f, 0.005f, 0.260f),
                "MachineBodyGray");
            lid.AddComponent<EnclosureLid>();

            // X-Axis Gantry (prismatic along X)
            var xGantry = CreateBox("X_Gantry", baseFrame.transform,
                new Vector3(0, 0.210f, 0), new Vector3(0.180f, 0.020f, 0.130f),
                "MachineBodyGray");
            var xAB = xGantry.AddComponent<ArticulationBody>();
            xAB.jointType = ArticulationJointType.PrismaticJoint;
            xAB.linearLockX = ArticulationDofLock.LimitedMotion;
            xAB.linearLockY = ArticulationDofLock.LockedMotion;
            xAB.linearLockZ = ArticulationDofLock.LockedMotion;
            var xDrive = xAB.xDrive;
            xDrive.lowerLimit = 0f;
            xDrive.upperLimit = 0.1524f;
            xDrive.stiffness = 100000f;
            xDrive.damping = 10000f;
            xDrive.forceLimit = float.MaxValue;
            xAB.xDrive = xDrive;

            // Y-Table (prismatic along Z in Unity = Y in machine coords)
            var yTable = CreateBox("Y_Table", xGantry.transform,
                new Vector3(0, -0.015f, 0), new Vector3(0.130f, 0.010f, 0.110f),
                "ViseSteel");
            var yAB = yTable.AddComponent<ArticulationBody>();
            yAB.jointType = ArticulationJointType.PrismaticJoint;
            yAB.linearLockX = ArticulationDofLock.LockedMotion;
            yAB.linearLockY = ArticulationDofLock.LockedMotion;
            yAB.linearLockZ = ArticulationDofLock.LimitedMotion;
            var yDrive = yAB.xDrive;
            yDrive.lowerLimit = 0f;
            yDrive.upperLimit = 0.1016f;
            yDrive.stiffness = 100000f;
            yDrive.damping = 10000f;
            yDrive.forceLimit = float.MaxValue;
            yAB.xDrive = yDrive;

            // Vise on Y-Table
            CreateBox("Vise_Fixed_Jaw", yTable.transform,
                new Vector3(0, 0.010f, -0.030f), new Vector3(0.090f, 0.020f, 0.008f),
                "ViseSteel");
            var movingJaw = CreateBox("Vise_Moving_Jaw", yTable.transform,
                new Vector3(0, 0.010f, 0.0482f), new Vector3(0.090f, 0.020f, 0.008f),
                "ViseSteel");

            // Workpiece mount point
            var mountPoint = new GameObject("WorkpieceMountPoint");
            mountPoint.transform.SetParent(yTable.transform);
            mountPoint.transform.localPosition = new Vector3(0, 0.020f, 0);

            // Z-Column (static pillar attached to base)
            var zColumn = CreateBox("Z_Column", baseFrame.transform,
                new Vector3(0, 0.290f, -0.110f), new Vector3(0.060f, 0.180f, 0.040f),
                "MachineBodyGray");

            // Z-Head (prismatic along -Y in Unity = Z down in machine)
            var zHead = CreateBox("Z_Head", zColumn.transform,
                new Vector3(0, -0.020f, 0.050f), new Vector3(0.050f, 0.080f, 0.050f),
                "MachineBodyGray");
            var zAB = zHead.AddComponent<ArticulationBody>();
            zAB.jointType = ArticulationJointType.PrismaticJoint;
            zAB.linearLockX = ArticulationDofLock.LockedMotion;
            zAB.linearLockY = ArticulationDofLock.LimitedMotion;
            zAB.linearLockZ = ArticulationDofLock.LockedMotion;
            var zDrive = zAB.xDrive;
            zDrive.lowerLimit = -0.06985f;
            zDrive.upperLimit = 0f;
            zDrive.stiffness = 100000f;
            zDrive.damping = 10000f;
            zDrive.forceLimit = float.MaxValue;
            zAB.xDrive = zDrive;

            // Spindle Motor
            var spindleMotor = CreateBox("Spindle_Motor", zHead.transform,
                new Vector3(0, -0.050f, 0), new Vector3(0.040f, 0.060f, 0.040f),
                "MachineBodyGray");

            // Collet (revolute joint for spindle rotation)
            var collet = new GameObject("Collet_ER11");
            collet.transform.SetParent(spindleMotor.transform);
            collet.transform.localPosition = new Vector3(0, -0.035f, 0);

            // End Mill (1/4" = 6.35mm diameter, 30mm cutting length)
            var endMill = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            endMill.name = "EndMill";
            endMill.transform.SetParent(collet.transform);
            endMill.transform.localPosition = new Vector3(0, -0.015f, 0);
            endMill.transform.localScale = new Vector3(0.00635f, 0.015f, 0.00635f);

            // Add controller component
            var controller = root.AddComponent<BantamExplorerController>();

            // Add SpindleVisualizer
            var spindleViz = collet.AddComponent<SpindleVisualizer>();

            // Add WorkpieceManager
            var wpManager = root.AddComponent<WorkpieceManager>();

            // Add ViseController
            var viseCtrl = yTable.AddComponent<ViseController>();

            Debug.Log("[BantamExplorerBuilder] CNC model built successfully");

            #if UNITY_EDITOR
            Selection.activeGameObject = root;
            Undo.RegisterCreatedObjectUndo(root, "Build Bantam Explorer CNC");
            #endif
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
