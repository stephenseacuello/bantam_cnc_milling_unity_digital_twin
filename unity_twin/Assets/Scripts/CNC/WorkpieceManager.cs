using UnityEngine;

namespace MiracleTwin.CNC
{
    /// <summary>
    /// Manages workpiece mounting, unmounting, and resetting.
    /// The workpiece is a 3"×3"×2" (76.2×76.2×50.8mm) 6061-T6 aluminum block.
    /// </summary>
    public class WorkpieceManager : MonoBehaviour
    {
        [SerializeField] private Transform mountPoint;
        [SerializeField] private GameObject workpiecePrefab;

        /// <summary>Workpiece dimensions in meters.</summary>
        public static readonly Vector3 WorkpieceSizeMeters = new(0.0762f, 0.0508f, 0.0762f);

        /// <summary>Workpiece dimensions in mm.</summary>
        public static readonly Vector3 WorkpieceSizeMM = new(76.2f, 50.8f, 76.2f);

        public GameObject CurrentWorkpiece { get; private set; }
        public bool IsWorkpieceMounted => CurrentWorkpiece != null;

        public void MountWorkpiece()
        {
            if (IsWorkpieceMounted) return;

            if (workpiecePrefab != null)
            {
                CurrentWorkpiece = Instantiate(workpiecePrefab, mountPoint);
            }
            else
            {
                // Create default aluminum block
                CurrentWorkpiece = GameObject.CreatePrimitive(PrimitiveType.Cube);
                CurrentWorkpiece.name = "Workpiece_6061T6";
                CurrentWorkpiece.transform.SetParent(mountPoint);
                CurrentWorkpiece.transform.localPosition = new Vector3(0, WorkpieceSizeMeters.y / 2f, 0);
                CurrentWorkpiece.transform.localScale = WorkpieceSizeMeters;
                CurrentWorkpiece.transform.localRotation = Quaternion.identity;

                // Remove default collider (voxel system handles collision)
                var col = CurrentWorkpiece.GetComponent<Collider>();
                if (col != null) Object.Destroy(col);
            }

            Debug.Log("[WorkpieceManager] Workpiece mounted");
        }

        public void UnmountWorkpiece()
        {
            if (!IsWorkpieceMounted) return;

            Destroy(CurrentWorkpiece);
            CurrentWorkpiece = null;
            Debug.Log("[WorkpieceManager] Workpiece unmounted");
        }

        public void ResetWorkpiece()
        {
            UnmountWorkpiece();
            MountWorkpiece();
        }

        void Start()
        {
            if (mountPoint == null)
            {
                mountPoint = transform.Find("X_Gantry/Y_Table/WorkpieceMountPoint");
            }
        }
    }
}
