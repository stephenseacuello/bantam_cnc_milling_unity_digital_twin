using UnityEngine;
using MiracleTwin.Core;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// GPU-instanced force arrows at the tool tip showing Fx (red), Fy (green), Fz (blue).
    /// Arrow length proportional to force magnitude: 1mm per 50N.
    /// </summary>
    public class ForceArrowRenderer : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private Mesh arrowMesh;
        [SerializeField] private Material arrowMaterial;

        [Header("Settings")]
        [SerializeField] private float scaleFactor = 0.001f / 50f; // 1mm per 50N
        [SerializeField] private float minimumForce = 5f;          // Don't show below this
        [SerializeField] private bool showPeakForces = false;

        private static readonly Color FxColor = new(1f, 0.2f, 0.2f, 0.9f);
        private static readonly Color FyColor = new(0.2f, 1f, 0.2f, 0.9f);
        private static readonly Color FzColor = new(0.3f, 0.3f, 1f, 0.9f);

        private Matrix4x4[] matrices = new Matrix4x4[3];
        private MaterialPropertyBlock[] propBlocks = new MaterialPropertyBlock[3];
        private Vector3 toolPosition;
        private Vector3 forces;
        private bool isActive;

        void Start()
        {
            for (int i = 0; i < 3; i++)
                propBlocks[i] = new MaterialPropertyBlock();

            propBlocks[0].SetColor("_Color", FxColor);
            propBlocks[1].SetColor("_Color", FyColor);
            propBlocks[2].SetColor("_Color", FzColor);
        }

        void OnEnable()
        {
            if (cuttingStateEvent != null)
                cuttingStateEvent.Register(OnCuttingState);
        }

        void OnDisable()
        {
            if (cuttingStateEvent != null)
                cuttingStateEvent.Unregister(OnCuttingState);
        }

        private void OnCuttingState(CuttingStateData state)
        {
            isActive = state.isCutting;
            toolPosition = state.toolPosition;
            forces = new Vector3(state.forceFx, state.forceFy, state.forceFz);
        }

        void Update()
        {
            if (!isActive || arrowMesh == null || arrowMaterial == null) return;

            DrawForceArrow(0, Vector3.right, forces.x);
            DrawForceArrow(1, Vector3.up, forces.y);
            DrawForceArrow(2, Vector3.forward, forces.z);
        }

        private void DrawForceArrow(int index, Vector3 direction, float magnitude)
        {
            float absMag = Mathf.Abs(magnitude);
            if (absMag < minimumForce) return;

            float length = absMag * scaleFactor;
            Vector3 dir = magnitude >= 0 ? direction : -direction;

            matrices[index] = Matrix4x4.TRS(
                toolPosition,
                Quaternion.LookRotation(dir),
                new Vector3(length * 0.1f, length * 0.1f, length)
            );

            Graphics.DrawMesh(arrowMesh, matrices[index], arrowMaterial, 0, null, 0, propBlocks[index]);
        }

        public void SetVisible(bool visible)
        {
            enabled = visible;
        }
    }
}
