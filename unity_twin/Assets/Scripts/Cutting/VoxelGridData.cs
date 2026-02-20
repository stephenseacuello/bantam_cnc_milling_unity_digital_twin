using Unity.Mathematics;
using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// CPU-side voxel grid metadata and dirty chunk tracking.
    /// Used for CPU-fallback rendering and spatial queries.
    /// </summary>
    public class VoxelGridData
    {
        public int3 GridSize { get; }
        public int3 ChunkSize { get; }
        public int3 ChunkCount { get; }
        public float3 VoxelSize { get; }
        public float3 WorldOrigin { get; }
        public int TotalVoxels { get; }

        private readonly bool[] dirtyChunks;

        public VoxelGridData(int3 gridSize, int3 chunkSize, float3 voxelSize, float3 worldOrigin)
        {
            GridSize = gridSize;
            ChunkSize = chunkSize;
            VoxelSize = voxelSize;
            WorldOrigin = worldOrigin;

            ChunkCount = new int3(
                (gridSize.x + chunkSize.x - 1) / chunkSize.x,
                (gridSize.y + chunkSize.y - 1) / chunkSize.y,
                (gridSize.z + chunkSize.z - 1) / chunkSize.z
            );

            TotalVoxels = gridSize.x * gridSize.y * gridSize.z;
            dirtyChunks = new bool[ChunkCount.x * ChunkCount.y * ChunkCount.z];
        }

        public void MarkChunkDirty(int3 chunkCoord)
        {
            int idx = chunkCoord.x + chunkCoord.y * ChunkCount.x +
                      chunkCoord.z * ChunkCount.x * ChunkCount.y;
            if (idx >= 0 && idx < dirtyChunks.Length)
                dirtyChunks[idx] = true;
        }

        public bool IsChunkDirty(int3 chunkCoord)
        {
            int idx = chunkCoord.x + chunkCoord.y * ChunkCount.x +
                      chunkCoord.z * ChunkCount.x * ChunkCount.y;
            return idx >= 0 && idx < dirtyChunks.Length && dirtyChunks[idx];
        }

        public void ClearDirtyFlags()
        {
            System.Array.Clear(dirtyChunks, 0, dirtyChunks.Length);
        }

        /// <summary>Convert world position to voxel grid coordinate.</summary>
        public int3 WorldToVoxel(float3 worldPos)
        {
            float3 local = (worldPos - WorldOrigin) / VoxelSize;
            return new int3(
                Mathf.FloorToInt(local.x),
                Mathf.FloorToInt(local.y),
                Mathf.FloorToInt(local.z)
            );
        }

        /// <summary>Convert voxel coordinate to world position (center of voxel).</summary>
        public float3 VoxelToWorld(int3 voxelCoord)
        {
            return WorldOrigin + ((float3)voxelCoord + 0.5f) * VoxelSize;
        }

        /// <summary>Convert voxel coordinate to chunk coordinate.</summary>
        public int3 VoxelToChunk(int3 voxelCoord)
        {
            return voxelCoord / ChunkSize;
        }

        public bool IsValidVoxel(int3 coord)
        {
            return coord.x >= 0 && coord.x < GridSize.x &&
                   coord.y >= 0 && coord.y < GridSize.y &&
                   coord.z >= 0 && coord.z < GridSize.z;
        }
    }
}
