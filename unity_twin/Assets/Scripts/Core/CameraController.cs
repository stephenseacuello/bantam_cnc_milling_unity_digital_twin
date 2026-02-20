using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Multi-mode camera system for the manufacturing cell.
    /// Supports orbit, follow-tool, and preset views.
    /// </summary>
    public class CameraController : MonoBehaviour
    {
        public enum CameraMode { Orbit, FollowTool, Front, Side, TopDown, Isometric }

        [Header("Orbit Settings")]
        [SerializeField] private Transform orbitTarget;
        [SerializeField] private float orbitDistance = 0.8f;
        [SerializeField] private float orbitSpeed = 5f;
        [SerializeField] private float zoomSpeed = 0.1f;
        [SerializeField] private float panSpeed = 0.005f;
        [SerializeField] private float minDistance = 0.1f;
        [SerializeField] private float maxDistance = 3f;

        [Header("Follow Tool")]
        [SerializeField] private Transform toolTip;
        [SerializeField] private Vector3 followOffset = new(0.15f, 0.15f, 0.15f);

        public CameraMode CurrentMode { get; private set; } = CameraMode.Orbit;

        private float orbitX = 30f;
        private float orbitY = 45f;
        private Vector3 panOffset;

        void LateUpdate()
        {
            switch (CurrentMode)
            {
                case CameraMode.Orbit:
                    UpdateOrbit();
                    break;
                case CameraMode.FollowTool:
                    UpdateFollowTool();
                    break;
                default:
                    // Preset views are static
                    break;
            }

            HandleInput();
        }

        private void UpdateOrbit()
        {
            if (orbitTarget == null) return;

            if (Input.GetMouseButton(0))
            {
                orbitX += Input.GetAxis("Mouse Y") * orbitSpeed;
                orbitY += Input.GetAxis("Mouse X") * orbitSpeed;
                orbitX = Mathf.Clamp(orbitX, -80f, 80f);
            }

            if (Input.GetMouseButton(2))
            {
                panOffset += transform.right * -Input.GetAxis("Mouse X") * panSpeed;
                panOffset += transform.up * -Input.GetAxis("Mouse Y") * panSpeed;
            }

            orbitDistance -= Input.GetAxis("Mouse ScrollWheel") * zoomSpeed;
            orbitDistance = Mathf.Clamp(orbitDistance, minDistance, maxDistance);

            Quaternion rotation = Quaternion.Euler(orbitX, orbitY, 0);
            Vector3 position = orbitTarget.position + panOffset + rotation * new Vector3(0, 0, -orbitDistance);

            transform.position = position;
            transform.LookAt(orbitTarget.position + panOffset);
        }

        private void UpdateFollowTool()
        {
            if (toolTip == null) return;
            transform.position = toolTip.position + followOffset;
            transform.LookAt(toolTip.position);
        }

        private void HandleInput()
        {
            if (Input.GetKeyDown(KeyCode.Alpha1)) SetPreset(CameraMode.Front);
            if (Input.GetKeyDown(KeyCode.Alpha2)) SetPreset(CameraMode.Side);
            if (Input.GetKeyDown(KeyCode.Alpha3)) SetPreset(CameraMode.TopDown);
            if (Input.GetKeyDown(KeyCode.Alpha4)) SetPreset(CameraMode.Isometric);
            if (Input.GetKeyDown(KeyCode.Alpha5)) SetMode(CameraMode.FollowTool);
            if (Input.GetKeyDown(KeyCode.Alpha0)) SetMode(CameraMode.Orbit);
        }

        public void SetMode(CameraMode mode)
        {
            CurrentMode = mode;
        }

        public void SetPreset(CameraMode preset)
        {
            CurrentMode = preset;
            Vector3 target = orbitTarget != null ? orbitTarget.position : Vector3.zero;

            switch (preset)
            {
                case CameraMode.Front:
                    transform.position = target + new Vector3(0, 0.2f, 0.8f);
                    break;
                case CameraMode.Side:
                    transform.position = target + new Vector3(0.8f, 0.2f, 0);
                    break;
                case CameraMode.TopDown:
                    transform.position = target + new Vector3(0, 0.8f, 0.001f);
                    break;
                case CameraMode.Isometric:
                    transform.position = target + new Vector3(0.5f, 0.5f, 0.5f);
                    break;
            }
            transform.LookAt(target);
        }

        public void ResetOrbit()
        {
            orbitX = 30f;
            orbitY = 45f;
            orbitDistance = 0.8f;
            panOffset = Vector3.zero;
            CurrentMode = CameraMode.Orbit;
        }
    }
}
