using UnityEngine;

namespace MiracleTwin.CNC
{
    /// <summary>
    /// Controls the vise jaw open/close animation.
    /// Moving jaw slides to grip the workpiece (76.2mm wide).
    /// </summary>
    public class ViseController : MonoBehaviour
    {
        [SerializeField] private Transform movingJaw;
        [SerializeField] private float openPosition = 0.060f;
        [SerializeField] private float closedPosition = 0.0381f; // Half of 76.2mm workpiece
        [SerializeField] private float moveSpeed = 0.05f;

        public bool IsOpen { get; private set; } = true;
        public bool IsMoving { get; private set; }

        private float targetPosition;
        private float currentPosition;

        void Start()
        {
            if (movingJaw == null)
                movingJaw = transform.Find("Vise_Moving_Jaw");

            currentPosition = openPosition;
            targetPosition = openPosition;
        }

        void Update()
        {
            if (Mathf.Abs(currentPosition - targetPosition) > 0.0001f)
            {
                IsMoving = true;
                currentPosition = Mathf.MoveTowards(currentPosition, targetPosition, moveSpeed * Time.deltaTime);

                if (movingJaw != null)
                {
                    var pos = movingJaw.localPosition;
                    pos.z = currentPosition;
                    movingJaw.localPosition = pos;
                }
            }
            else
            {
                IsMoving = false;
            }
        }

        public void Open()
        {
            targetPosition = openPosition;
            IsOpen = true;
        }

        public void Close()
        {
            targetPosition = closedPosition;
            IsOpen = false;
        }

        public void Toggle()
        {
            if (IsOpen) Close(); else Open();
        }
    }
}
