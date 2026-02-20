using UnityEngine;
using Unity.Mathematics;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// GPU-accelerated voxel grid representing the workpiece.
    /// Workpiece: 3"×3"×2" = 76.2mm × 76.2mm × 50.8mm
    /// Grid: 256×170×256 (~0.3mm resolution), bit-packed into uints.
    /// Total: ~1.36 MB GPU memory.
    ///
    /// Pipeline: SubtractTool → Mark dirty chunks → MarchingCubes on dirty chunks → Render
    /// </summary>
    public class VoxelWorkpiece : MonoBehaviour
    {
        /// <summary>Workpiece dimensions in meters.</summary>
        public static readonly Vector3 WorkpieceSize = new(0.0762f, 0.0508f, 0.0762f);

        /// <summary>Voxel grid dimensions.</summary>
        public static readonly int3 GridSize = new(256, 170, 256);

        /// <summary>Size of one voxel in meters.</summary>
        public static readonly float3 VoxelSize = new(
            WorkpieceSize.x / GridSize.x,
            WorkpieceSize.y / GridSize.y,
            WorkpieceSize.z / GridSize.z
        );

        private static readonly int3 ChunkSize = new(16, 16, 16);
        private static readonly int3 ChunksCount = new(16, 11, 16); // ceil(GridSize / ChunkSize)

        [Header("Compute Shaders")]
        [SerializeField] private ComputeShader subtractShader;
        [SerializeField] private ComputeShader marchingCubesShader;
        [SerializeField] private ComputeShader engagementShader;

        [Header("Rendering")]
        [SerializeField] private Material workpieceMaterial;
        [SerializeField] private int maxTrianglesPerChunk = 32768;
        [SerializeField] private int maxDirtyChunksPerFrame = 8;

        // GPU Buffers
        private ComputeBuffer voxelBuffer;          // Bit-packed voxel grid
        private ComputeBuffer dirtyChunkBuffer;     // Per-chunk dirty flags
        private ComputeBuffer vertexBuffer;         // Marching cubes output vertices
        private ComputeBuffer triCountBuffer;       // Indirect draw args
        private ComputeBuffer removedCountBuffer;   // Voxels removed this frame
        private ComputeBuffer engagedCountBuffer;   // Engaged voxels for force calc

        // Kernel IDs
        private int subtractKernel;
        private int engagementKernel;
        private int marchingCubesKernel;

        // State
        public int TotalVoxels { get; private set; }
        public int RemovedVoxels { get; private set; }
        public int EngagedVoxelCount { get; private set; }
        public float MaterialRemovedFraction => TotalVoxels > 0 ? (float)RemovedVoxels / TotalVoxels : 0f;
        public bool IsInitialized { get; private set; }

        /// <summary>Total workpiece volume in mm^3.</summary>
        public float TotalVolumeMM3 => WorkpieceSize.x * WorkpieceSize.y * WorkpieceSize.z * 1e9f;

        /// <summary>Volume of a single voxel in mm^3.</summary>
        public float VoxelVolumeMM3 => VoxelSize.x * VoxelSize.y * VoxelSize.z * 1e9f;

        /// <summary>Total volume of removed material in mm^3.</summary>
        public float RemovedVolumeMM3 => RemovedVoxels * VoxelVolumeMM3;

        /// <summary>Remaining material volume in mm^3.</summary>
        public float RemainingVolumeMM3 => (TotalVoxels - RemovedVoxels) * VoxelVolumeMM3;

        private int totalBitWords;
        private int totalChunks;
        private Bounds renderBounds;
        private int[] removedCountReadback = new int[1];
        private int[] engagedCountReadback = new int[1];

        void Start()
        {
            Initialize();
        }

        public void Initialize()
        {
            if (IsInitialized) return;

            TotalVoxels = GridSize.x * GridSize.y * GridSize.z;
            totalBitWords = (TotalVoxels + 31) / 32;
            totalChunks = ChunksCount.x * ChunksCount.y * ChunksCount.z;

            // Allocate GPU buffers
            voxelBuffer = new ComputeBuffer(totalBitWords, sizeof(uint));
            dirtyChunkBuffer = new ComputeBuffer(totalChunks, sizeof(uint));
            removedCountBuffer = new ComputeBuffer(1, sizeof(uint));
            engagedCountBuffer = new ComputeBuffer(1, sizeof(uint));

            // Vertex buffer for marching cubes output (position + normal = 6 floats per vertex)
            int maxVerts = maxTrianglesPerChunk * 3 * maxDirtyChunksPerFrame;
            vertexBuffer = new ComputeBuffer(maxVerts, sizeof(float) * 6, ComputeBufferType.Append);
            triCountBuffer = new ComputeBuffer(4, sizeof(int), ComputeBufferType.IndirectArguments);

            // Fill voxel grid (all bits = 1 = solid)
            uint[] solidGrid = new uint[totalBitWords];
            for (int i = 0; i < totalBitWords; i++)
                solidGrid[i] = 0xFFFFFFFF;
            voxelBuffer.SetData(solidGrid);

            // Clear dirty chunks
            uint[] clearChunks = new uint[totalChunks];
            dirtyChunkBuffer.SetData(clearChunks);

            // Find kernels
            if (subtractShader != null)
            {
                subtractKernel = subtractShader.FindKernel("CSSubtractTool");
                engagementKernel = subtractShader.FindKernel("CSCountEngaged");
            }
            if (marchingCubesShader != null)
            {
                marchingCubesKernel = marchingCubesShader.FindKernel("CSMarchingCubes");
            }

            renderBounds = new Bounds(transform.position + (Vector3)(float3)WorkpieceSize * 0.5f,
                                      (Vector3)(float3)WorkpieceSize * 1.5f);

            RemovedVoxels = 0;
            IsInitialized = true;

            Debug.Log($"[VoxelWorkpiece] Initialized: {GridSize.x}x{GridSize.y}x{GridSize.z} " +
                      $"({TotalVoxels:N0} voxels, {totalBitWords * 4 / 1024f:F1} KB)");
        }

        /// <summary>
        /// Subtract tool volume from the voxel grid along a line segment.
        /// Call once per simulation step with previous and current tool tip positions.
        /// </summary>
        public int SubtractTool(Vector3 prevTipWorld, Vector3 currTipWorld, float toolRadius)
        {
            if (!IsInitialized || subtractShader == null) return 0;

            // Transform to local space
            Vector3 prevLocal = transform.InverseTransformPoint(prevTipWorld);
            Vector3 currLocal = transform.InverseTransformPoint(currTipWorld);

            // Reset removed count
            removedCountBuffer.SetData(new uint[] { 0 });

            // Set shader parameters
            subtractShader.SetBuffer(subtractKernel, "_VoxelGrid", voxelBuffer);
            subtractShader.SetBuffer(subtractKernel, "_DirtyChunks", dirtyChunkBuffer);
            subtractShader.SetBuffer(subtractKernel, "_RemovedCount", removedCountBuffer);
            subtractShader.SetVector("_PrevToolTip", prevLocal);
            subtractShader.SetVector("_CurrToolTip", currLocal);
            subtractShader.SetFloat("_ToolRadius", toolRadius);
            subtractShader.SetVector("_GridOrigin", Vector3.zero);
            subtractShader.SetVector("_VoxelSize", (Vector3)(float3)VoxelSize);
            subtractShader.SetInts("_GridDim", GridSize.x, GridSize.y, GridSize.z);
            subtractShader.SetInts("_ChunkDim", ChunksCount.x, ChunksCount.y, ChunksCount.z);

            // Dispatch
            int groupsX = Mathf.CeilToInt(GridSize.x / 8f);
            int groupsY = Mathf.CeilToInt(GridSize.y / 8f);
            int groupsZ = Mathf.CeilToInt(GridSize.z / 8f);
            subtractShader.Dispatch(subtractKernel, groupsX, groupsY, groupsZ);

            // Read back removed count
            removedCountBuffer.GetData(removedCountReadback);
            int newlyRemoved = removedCountReadback[0];
            RemovedVoxels += newlyRemoved;

            return newlyRemoved;
        }

        /// <summary>
        /// Count voxels currently engaged with the tool (for force calculation).
        /// Returns the count of solid voxels within the tool swept volume.
        /// </summary>
        public int CountEngagedVoxels(Vector3 prevTipWorld, Vector3 currTipWorld, float toolRadius)
        {
            if (!IsInitialized || subtractShader == null) return 0;

            Vector3 prevLocal = transform.InverseTransformPoint(prevTipWorld);
            Vector3 currLocal = transform.InverseTransformPoint(currTipWorld);

            engagedCountBuffer.SetData(new uint[] { 0 });

            subtractShader.SetBuffer(engagementKernel, "_VoxelGrid", voxelBuffer);
            subtractShader.SetBuffer(engagementKernel, "_RemovedCount", engagedCountBuffer);
            subtractShader.SetVector("_PrevToolTip", prevLocal);
            subtractShader.SetVector("_CurrToolTip", currLocal);
            subtractShader.SetFloat("_ToolRadius", toolRadius);
            subtractShader.SetVector("_GridOrigin", Vector3.zero);
            subtractShader.SetVector("_VoxelSize", (Vector3)(float3)VoxelSize);
            subtractShader.SetInts("_GridDim", GridSize.x, GridSize.y, GridSize.z);

            int groupsX = Mathf.CeilToInt(GridSize.x / 8f);
            int groupsY = Mathf.CeilToInt(GridSize.y / 8f);
            int groupsZ = Mathf.CeilToInt(GridSize.z / 8f);
            subtractShader.Dispatch(engagementKernel, groupsX, groupsY, groupsZ);

            engagedCountBuffer.GetData(engagedCountReadback);
            EngagedVoxelCount = engagedCountReadback[0];
            return EngagedVoxelCount;
        }

        /// <summary>Render the voxel mesh using marching cubes on dirty chunks.</summary>
        public void RenderMesh()
        {
            if (!IsInitialized || marchingCubesShader == null || workpieceMaterial == null) return;

            vertexBuffer.SetCounterValue(0);

            // TODO: Iterate dirty chunks and dispatch marching cubes per chunk
            // For now, render using DrawProceduralIndirect

            // Copy append buffer count to indirect args
            ComputeBuffer.CopyCount(vertexBuffer, triCountBuffer, 0);

            workpieceMaterial.SetBuffer("_Vertices", vertexBuffer);
            Graphics.DrawProceduralIndirect(
                workpieceMaterial,
                renderBounds,
                MeshTopology.Triangles,
                triCountBuffer
            );
        }

        /// <summary>
        /// Get the percentage of material removed from the workpiece.
        /// Returns a value from 0 (untouched) to 100 (fully removed).
        /// </summary>
        public float GetMaterialRemovalPercentage()
        {
            if (TotalVoxels <= 0) return 0f;
            return (float)RemovedVoxels / TotalVoxels * 100f;
        }

        /// <summary>
        /// Get a summary of volume statistics for the workpiece.
        /// </summary>
        public VolumeStats GetVolumeStats()
        {
            return new VolumeStats
            {
                totalVolumeMM3 = TotalVolumeMM3,
                removedVolumeMM3 = RemovedVolumeMM3,
                remainingVolumeMM3 = RemainingVolumeMM3,
                removalPercentage = GetMaterialRemovalPercentage(),
                totalVoxels = TotalVoxels,
                removedVoxels = RemovedVoxels,
                remainingVoxels = TotalVoxels - RemovedVoxels
            };
        }

        /// <summary>Volume statistics snapshot.</summary>
        public struct VolumeStats
        {
            public float totalVolumeMM3;
            public float removedVolumeMM3;
            public float remainingVolumeMM3;
            public float removalPercentage;
            public int totalVoxels;
            public int removedVoxels;
            public int remainingVoxels;
        }

        /// <summary>Reset the workpiece to a solid block.</summary>
        public void Reset()
        {
            if (!IsInitialized) return;

            uint[] solidGrid = new uint[totalBitWords];
            for (int i = 0; i < totalBitWords; i++)
                solidGrid[i] = 0xFFFFFFFF;
            voxelBuffer.SetData(solidGrid);

            uint[] clearChunks = new uint[totalChunks];
            dirtyChunkBuffer.SetData(clearChunks);

            RemovedVoxels = 0;
            EngagedVoxelCount = 0;

            Debug.Log("[VoxelWorkpiece] Reset to solid block");
        }

        /// <summary>
        /// Release all GPU ComputeBuffers. Called automatically on destroy.
        /// Safe to call multiple times.
        /// </summary>
        private void ReleaseGPUResources()
        {
            if (voxelBuffer != null) { voxelBuffer.Release(); voxelBuffer = null; }
            if (dirtyChunkBuffer != null) { dirtyChunkBuffer.Release(); dirtyChunkBuffer = null; }
            if (vertexBuffer != null) { vertexBuffer.Release(); vertexBuffer = null; }
            if (triCountBuffer != null) { triCountBuffer.Release(); triCountBuffer = null; }
            if (removedCountBuffer != null) { removedCountBuffer.Release(); removedCountBuffer = null; }
            if (engagedCountBuffer != null) { engagedCountBuffer.Release(); engagedCountBuffer = null; }

            IsInitialized = false;
        }

        void OnDestroy()
        {
            ReleaseGPUResources();
        }

        void OnApplicationQuit()
        {
            // Ensure GPU resources are released even if OnDestroy is not called
            ReleaseGPUResources();
        }

        void OnDrawGizmosSelected()
        {
            Gizmos.color = new Color(0, 1, 0, 0.2f);
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawWireCube(
                (Vector3)(float3)WorkpieceSize * 0.5f,
                (Vector3)(float3)WorkpieceSize
            );
        }
    }
}
