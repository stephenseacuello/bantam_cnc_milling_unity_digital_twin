using UnityEngine;

namespace MiracleTwin.CNC
{
    /// <summary>
    /// Controls the CNC enclosure lid open/close animation.
    /// Lid hinges at the back, rotating 90 degrees to open.
    /// Opens during robot loading/unloading sequences.
    /// </summary>
    public class EnclosureLid : MonoBehaviour
    {
        [SerializeField] private float openAngle = 90f;
        [SerializeField] private float rotateSpeed = 120f;
        [SerializeField] private Vector3 hingeAxis = Vector3.right;

        public bool IsOpen { get; private set; }
        public bool IsMoving { get; private set; }
        public float CurrentAngle { get; private set; }

        private float targetAngle;
        private Quaternion closedRotation;

        void Start()
        {
            closedRotation = transform.localRotation;
        }

        void Update()
        {
            if (Mathf.Abs(CurrentAngle - targetAngle) > 0.1f)
            {
                IsMoving = true;
                CurrentAngle = Mathf.MoveTowards(CurrentAngle, targetAngle, rotateSpeed * Time.deltaTime);
                transform.localRotation = closedRotation * Quaternion.AngleAxis(CurrentAngle, hingeAxis);
            }
            else
            {
                IsMoving = false;
            }
        }

        public void Open()
        {
            targetAngle = openAngle;
            IsOpen = true;
        }

        public void Close()
        {
            targetAngle = 0f;
            IsOpen = false;
        }

        public void Toggle()
        {
            if (IsOpen) Close(); else Open();
        }
    }
}
