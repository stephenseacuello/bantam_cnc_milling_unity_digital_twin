using UnityEngine;
using Unity.Mathematics;
using Unity.Profiling;

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

#if DEVELOPMENT_BUILD || UNITY_EDITOR
        private static readonly ProfilerMarker s_SubtractToolMarker = new("VoxelWorkpiece.SubtractTool");
        private static readonly ProfilerMarker s_RenderMeshMarker = new("VoxelWorkpiece.RenderMesh");
        private static readonly ProfilerMarker s_CountEngagedMarker = new("VoxelWorkpiece.CountEngaged");
#endif

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
        private uint[] dirtyChunkReadback;

        void Start()
        {
            Initialize();
        }

        public void Initialize()
        {
            if (IsInitialized) return;

            if (!SystemInfo.supportsComputeShaders)
            {
                Debug.LogWarning("[VoxelWorkpiece] Compute shaders not supported on this GPU. Using fallback mesh.");
                CreateFallbackMesh();
                return;
            }

            try
            {
                InitializeGPU();
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[VoxelWorkpiece] GPU initialization failed: {ex.Message}. Using fallback mesh.");
                ReleaseGPUResources();
                CreateFallbackMesh();
            }
        }

        private void InitializeGPU()
        {
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

            dirtyChunkReadback = new uint[totalChunks];

            RemovedVoxels = 0;
            IsInitialized = true;

            Debug.Log($"[VoxelWorkpiece] Initialized: {GridSize.x}x{GridSize.y}x{GridSize.z} " +
                      $"({TotalVoxels:N0} voxels, {totalBitWords * 4 / 1024f:F1} KB)");
        }

        /// <summary>
        /// Fallback for systems without compute shader support.
        /// Creates a simple box mesh matching the workpiece dimensions.
        /// </summary>
        private void CreateFallbackMesh()
        {
            TotalVoxels = GridSize.x * GridSize.y * GridSize.z;

            var meshFilter = gameObject.GetComponent<MeshFilter>();
            if (meshFilter == null)
                meshFilter = gameObject.AddComponent<MeshFilter>();

            var meshRenderer = gameObject.GetComponent<MeshRenderer>();
            if (meshRenderer == null)
                meshRenderer = gameObject.AddComponent<MeshRenderer>();

            if (workpieceMaterial != null)
                meshRenderer.material = workpieceMaterial;

            // Create a box mesh matching WorkpieceSize
            var mesh = new Mesh { name = "VoxelWorkpiece_Fallback" };
            Vector3 size = WorkpieceSize;
            Vector3 half = size * 0.5f;

            mesh.vertices = new Vector3[]
            {
                // Front face
                new(-half.x, -half.y, -half.z), new(half.x, -half.y, -half.z),
                new(half.x, half.y, -half.z), new(-half.x, half.y, -half.z),
                // Back face
                new(-half.x, -half.y, half.z), new(half.x, -half.y, half.z),
                new(half.x, half.y, half.z), new(-half.x, half.y, half.z),
            };

            mesh.triangles = new int[]
            {
                0,2,1, 0,3,2,  // Front
                5,6,4, 4,6,7,  // Back
                4,3,0, 4,7,3,  // Left
                1,2,5, 5,2,6,  // Right
                3,6,2, 3,7,6,  // Top
                4,1,5, 4,0,1,  // Bottom
            };

            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            meshFilter.mesh = mesh;

            IsInitialized = true;
            Debug.Log("[VoxelWorkpiece] Fallback mesh created (compute shaders unavailable).");
        }

        /// <summary>
        /// Subtract tool volume from the voxel grid along a line segment.
        /// Call once per simulation step with previous and current tool tip positions.
        /// </summary>
        public int SubtractTool(Vector3 prevTipWorld, Vector3 currTipWorld, float toolRadius)
        {
            if (!IsInitialized || subtractShader == null) return 0;

#if DEVELOPMENT_BUILD || UNITY_EDITOR
            using (s_SubtractToolMarker.Auto())
            {
#endif

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

#if DEVELOPMENT_BUILD || UNITY_EDITOR
            }
#endif
            return newlyRemoved;
        }

        /// <summary>
        /// Count voxels currently engaged with the tool (for force calculation).
        /// Returns the count of solid voxels within the tool swept volume.
        /// </summary>
        public int CountEngagedVoxels(Vector3 prevTipWorld, Vector3 currTipWorld, float toolRadius)
        {
            if (!IsInitialized || subtractShader == null) return 0;

#if DEVELOPMENT_BUILD || UNITY_EDITOR
            using (s_CountEngagedMarker.Auto())
            {
#endif

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

#if DEVELOPMENT_BUILD || UNITY_EDITOR
            }
#endif
            return EngagedVoxelCount;
        }

        /// <summary>Render the voxel mesh using marching cubes on dirty chunks.</summary>
        public void RenderMesh()
        {
            if (!IsInitialized || marchingCubesShader == null || workpieceMaterial == null) return;

#if DEVELOPMENT_BUILD || UNITY_EDITOR
            using (s_RenderMeshMarker.Auto())
            {
#endif

            vertexBuffer.SetCounterValue(0);

            // Read dirty chunk flags from GPU
            dirtyChunkBuffer.GetData(dirtyChunkReadback);

            // Collect dirty chunk indices, capped at maxDirtyChunksPerFrame
            int chunksDispatched = 0;
            for (int i = 0; i < totalChunks && chunksDispatched < maxDirtyChunksPerFrame; i++)
            {
                if (dirtyChunkReadback[i] == 0) continue;

                // Compute 3D chunk coordinates from flat index
                int cx = i % ChunksCount.x;
                int cy = (i / ChunksCount.x) % ChunksCount.y;
                int cz = i / (ChunksCount.x * ChunksCount.y);

                // Set chunk origin in voxel-space for the compute shader
                marchingCubesShader.SetInts("_ChunkOrigin",
                    cx * ChunkSize.x, cy * ChunkSize.y, cz * ChunkSize.z);
                marchingCubesShader.SetInts("_ChunkSize", ChunkSize.x, ChunkSize.y, ChunkSize.z);
                marchingCubesShader.SetInts("_GridDim", GridSize.x, GridSize.y, GridSize.z);
                marchingCubesShader.SetVector("_VoxelSize", (Vector3)(float3)VoxelSize);
                marchingCubesShader.SetBuffer(marchingCubesKernel, "_VoxelGrid", voxelBuffer);
                marchingCubesShader.SetBuffer(marchingCubesKernel, "_Vertices", vertexBuffer);

                // Dispatch per-chunk: thread groups = ChunkSize / 8
                marchingCubesShader.Dispatch(marchingCubesKernel,
                    Mathf.CeilToInt(ChunkSize.x / 8f),
                    Mathf.CeilToInt(ChunkSize.y / 8f),
                    Mathf.CeilToInt(ChunkSize.z / 8f));

                // Clear dirty flag for this chunk
                dirtyChunkReadback[i] = 0;
                chunksDispatched++;
            }

            // Write cleared dirty flags back to GPU
            if (chunksDispatched > 0)
                dirtyChunkBuffer.SetData(dirtyChunkReadback);

            // Copy append buffer count to indirect args
            ComputeBuffer.CopyCount(vertexBuffer, triCountBuffer, 0);

            workpieceMaterial.SetBuffer("_Vertices", vertexBuffer);
            Graphics.DrawProceduralIndirect(
                workpieceMaterial,
                renderBounds,
                MeshTopology.Triangles,
                triCountBuffer
            );

#if DEVELOPMENT_BUILD || UNITY_EDITOR
            }
#endif
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
