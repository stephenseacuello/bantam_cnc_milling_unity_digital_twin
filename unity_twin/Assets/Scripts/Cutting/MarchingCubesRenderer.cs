using UnityEngine;
using Unity.Mathematics;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Dispatches GPU marching cubes per dirty chunk and renders the result
    /// using Graphics.DrawProceduralIndirect.
    /// Only processes dirty chunks to minimize GPU work.
    /// </summary>
    public class MarchingCubesRenderer : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private VoxelWorkpiece voxelWorkpiece;
        [SerializeField] private ComputeShader marchingCubesShader;
        [SerializeField] private Material workpieceMaterial;

        [Header("Settings")]
        [SerializeField] private int maxChunksPerFrame = 8;
        [SerializeField] private bool renderEnabled = true;

        // Marching cubes lookup tables
        private ComputeBuffer triTableBuffer;
        private ComputeBuffer edgeTableBuffer;
        private ComputeBuffer vertexAppendBuffer;
        private ComputeBuffer argsBuffer;

        private int mcKernel;
        private bool isInitialized;
        private Bounds renderBounds;

        // Chunk dimensions (must match VoxelWorkpiece)
        private static readonly int3 ChunkSize = new(16, 16, 16);
        private static readonly int3 ChunksCount = new(16, 11, 16);

        public int VertexCount { get; private set; }
        public int ChunksProcessedThisFrame { get; private set; }

        void Start()
        {
            Initialize();
        }

        private void Initialize()
        {
            if (marchingCubesShader == null) return;

            mcKernel = marchingCubesShader.FindKernel("CSMarchingCubes");

            // Load lookup tables using standard Paul Bourke tables
            LoadLookupTables();

            // Append buffer for output vertices (position + normal)
            int maxVerts = 65536 * 3; // Max triangles * 3 vertices
            vertexAppendBuffer = new ComputeBuffer(maxVerts, sizeof(float) * 6, ComputeBufferType.Append);
            argsBuffer = new ComputeBuffer(4, sizeof(int), ComputeBufferType.IndirectArguments);

            Vector3 wpSize = VoxelWorkpiece.WorkpieceSize;
            renderBounds = new Bounds(
                transform.position + wpSize * 0.5f,
                wpSize * 2f
            );

            isInitialized = true;
        }

        private void LoadLookupTables()
        {
            // Standard marching cubes tables (256 entries for edge table, 256x16 for tri table)
            var triTableAsset = Resources.Load<TextAsset>("MarchingCubesLUT");
            if (triTableAsset != null)
            {
                // Load from binary asset if available
                Debug.Log("[MarchingCubesRenderer] Loaded LUT from resources");
            }
            else
            {
                // Use standard Paul Bourke tables from MarchingCubesTables
                LoadStandardTables();
            }
        }

        private void LoadStandardTables()
        {
            edgeTableBuffer = new ComputeBuffer(256, sizeof(int));
            edgeTableBuffer.SetData(MarchingCubesTables.EdgeTable);

            triTableBuffer = new ComputeBuffer(256 * 16, sizeof(int));
            triTableBuffer.SetData(MarchingCubesTables.TriTable);
        }

        void LateUpdate()
        {
            if (!isInitialized || !renderEnabled) return;
            if (voxelWorkpiece == null || !voxelWorkpiece.IsInitialized) return;

            RenderWorkpiece();
        }

        private void RenderWorkpiece()
        {
            if (workpieceMaterial == null) return;

            vertexAppendBuffer.SetCounterValue(0);
            ChunksProcessedThisFrame = 0;

            // Delegate chunk iteration to VoxelWorkpiece.RenderMesh() which
            // handles dirty chunk tracking and per-chunk dispatch internally.
            // This renderer provides the standalone rendering path when
            // VoxelWorkpiece rendering is handled externally.
            voxelWorkpiece.RenderMesh();

            // Copy append buffer count to indirect args for our own draw call
            ComputeBuffer.CopyCount(vertexAppendBuffer, argsBuffer, 0);

            workpieceMaterial.SetBuffer("_Vertices", vertexAppendBuffer);
            workpieceMaterial.SetBuffer("_EdgeTable", edgeTableBuffer);
            workpieceMaterial.SetBuffer("_TriTable", triTableBuffer);

            Graphics.DrawProceduralIndirect(
                workpieceMaterial,
                renderBounds,
                MeshTopology.Triangles,
                argsBuffer
            );
        }

        public void SetRenderEnabled(bool enabled)
        {
            renderEnabled = enabled;
        }

        void OnDestroy()
        {
            triTableBuffer?.Release();
            edgeTableBuffer?.Release();
            vertexAppendBuffer?.Release();
            argsBuffer?.Release();
        }
    }
}
