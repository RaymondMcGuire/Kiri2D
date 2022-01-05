/*
 * @Author: Xu.WANG
 * @Date: 2021-02-28 02:57:46
 * @LastEditTime: 2021-03-20 00:04:00
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri\KiriPBSCuda\include\kiri_pbs_cuda\sampler\cuda_sampler_struct.cuh
 */

#ifndef _CUDA_SAMPLER_STRUCT_CUH_
#define _CUDA_SAMPLER_STRUCT_CUH_

#pragma once

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace KIRI
{
    template <typename T>
    struct CudaAxisAlignedBox
    {
        T Min;
        T Max;
        __device__ __host__ CudaAxisAlignedBox(
            T min,
            T max)
            : Min(min),
              Max(max) {}
    };

    struct LevelSetShapeInfo
    {
        CudaAxisAlignedBox<float3> BBox;
        int3 GridSize;
        float CellSize;
        size_t NumOfFaces;

        LevelSetShapeInfo(CudaAxisAlignedBox<float3> boundingBox, float cellSize, size_t numOfFaces)
            : CellSize(cellSize), BBox(boundingBox), NumOfFaces(numOfFaces)
        {
            GridSize = float3_to_int3((BBox.Max - BBox.Min) / cellSize);
        }
    };
}

#endif