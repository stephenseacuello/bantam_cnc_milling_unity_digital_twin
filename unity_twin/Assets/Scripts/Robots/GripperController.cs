using UnityEngine;

namespace MiracleTwin.Robots
{
    /// <summary>
    /// Controls robot gripper with symmetric finger mimic joint.
    /// Both fingers move simultaneously in opposite directions.
    /// </summary>
    public class GripperController : MonoBehaviour
    {
        [SerializeField] private Transform leftFinger;
        [SerializeField] private Transform rightFinger;
        [SerializeField] private float openWidth = 0.04f;
        [SerializeField] private float closedWidth = 0.005f;
        [SerializeField] private float moveSpeed = 0.1f;
        [SerializeField] private Vector3 fingerAxis = Vector3.right;

        public enum GripperState { Open, Closing, Closed, Opening }

        public GripperState State { get; private set; } = GripperState.Open;
        public bool IsGripping => State == GripperState.Closed;
        public float CurrentWidth { get; private set; }

        private float targetWidth;
        private Transform grippedObject;
        private Vector3 gripOffset;

        void Start()
        {
            CurrentWidth = openWidth;
            targetWidth = openWidth;
        }

        void Update()
        {
            if (Mathf.Abs(CurrentWidth - targetWidth) > 0.0001f)
            {
                CurrentWidth = Mathf.MoveTowards(CurrentWidth, targetWidth, moveSpeed * Time.deltaTime);

                State = CurrentWidth > targetWidth ? GripperState.Closing : GripperState.Opening;

                float halfWidth = CurrentWidth / 2f;
                if (leftFinger != null)
                    leftFinger.localPosition = fingerAxis * halfWidth;
                if (rightFinger != null)
                    rightFinger.localPosition = fingerAxis * -halfWidth;
            }
            else
            {
                State = CurrentWidth <= closedWidth + 0.001f
                    ? GripperState.Closed
                    : GripperState.Open;
            }

            // Update gripped object position
            if (grippedObject != null && IsGripping)
            {
                grippedObject.position = transform.position + transform.TransformDirection(gripOffset);
                grippedObject.rotation = transform.rotation;
            }
        }

        public void Open()
        {
            targetWidth = openWidth;
            if (grippedObject != null)
            {
                grippedObject.SetParent(null);
                grippedObject = null;
            }
        }

        public void Close()
        {
            targetWidth = closedWidth;
        }

        public void Grip(Transform obj)
        {
            Close();
            grippedObject = obj;
            gripOffset = transform.InverseTransformDirection(obj.position - transform.position);
            obj.SetParent(transform);
        }

        public void Release()
        {
            Open();
        }
    }
}
