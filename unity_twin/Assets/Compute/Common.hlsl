// Common.hlsl
// Shared functions and constants for MIRACLE Digital Twin compute shaders.

#ifndef MIRACLE_COMMON_HLSL
#define MIRACLE_COMMON_HLSL

// Convert voxel grid coordinate to flat index
uint VoxelFlatIndex(uint3 coord, int3 gridDim)
{
    return coord.x + coord.y * (uint)gridDim.x + coord.z * (uint)gridDim.x * (uint)gridDim.y;
}

// Convert flat index back to 3D coordinate
uint3 FlatToVoxel(uint flatIdx, int3 gridDim)
{
    uint z = flatIdx / ((uint)gridDim.x * (uint)gridDim.y);
    uint rem = flatIdx % ((uint)gridDim.x * (uint)gridDim.y);
    uint y = rem / (uint)gridDim.x;
    uint x = rem % (uint)gridDim.x;
    return uint3(x, y, z);
}

// Check if voxel coordinate is within grid bounds
bool IsInGrid(int3 coord, int3 gridDim)
{
    return all(coord >= 0) && all(coord < gridDim);
}

// Convert voxel coordinate to world position (center of voxel)
float3 VoxelToWorld(int3 coord, float3 origin, float3 voxelSize)
{
    return origin + (float3(coord) + 0.5) * voxelSize;
}

// Convert world position to voxel coordinate
int3 WorldToVoxel(float3 worldPos, float3 origin, float3 voxelSize)
{
    return int3(floor((worldPos - origin) / voxelSize));
}

// Distance from point to line segment (clamped)
float PointSegmentDistance(float3 p, float3 a, float3 b)
{
    float3 ab = b - a;
    float abLenSq = dot(ab, ab);

    if (abLenSq < 1e-10)
        return length(p - a);

    float t = saturate(dot(p - a, ab) / abLenSq);
    return length(p - (a + t * ab));
}

// Smooth minimum for SDF blending
float SmoothMin(float a, float b, float k)
{
    float h = saturate(0.5 + 0.5 * (b - a) / k);
    return lerp(b, a, h) - k * h * (1.0 - h);
}

#endif // MIRACLE_COMMON_HLSL
