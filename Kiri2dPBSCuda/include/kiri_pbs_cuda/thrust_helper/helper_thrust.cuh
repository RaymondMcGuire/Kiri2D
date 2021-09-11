/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 20:17:45
 * @LastEditTime: 2021-09-03 17:30:01
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\thrust_helper\helper_thrust.cuh
 */

#ifndef _THRUST_HELPER_CUH_
#define _THRUST_HELPER_CUH_

#pragma once

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace ThrustHelper
{
    template <typename T>
    struct Plus
    {
        T B;
        Plus(const T b) : B(b) {}
        __host__ __device__
            T
            operator()(const T &a) const
        {
            return a + B;
        }
    };

    template <typename T>
    struct AbsPlus
    {
        __host__ __device__
            T
            operator()(const T &a, const T &b) const
        {
            return abs(a) + abs(b);
        }
    };

    template <typename T>
    struct CompareLengthCuda
    {
        static_assert(
            KIRI::IsSame_Float2<T>::value || KIRI::IsSame_Float3<T>::value || KIRI::IsSame_Float4<T>::value,
            "data type is not correct");

        __host__ __device__ bool operator()(T f1, T f2)
        {
            return length(f1) < length(f2);
        }
    };

    static inline __host__ __device__ int2 ComputeGridXYByPos2(const float2 &pos, const float cellSize, const int2 &gridSize)
    {
        int x = min(max((int)(pos.x / cellSize), 0), gridSize.x - 1),
            y = min(max((int)(pos.y / cellSize), 0), gridSize.y - 1);

        return make_int2(x, y);
    }

    struct Pos2GridHash
    {
        float2 mLowestPoint;
        float mCellSize;
        int2 mGridSize;
        __host__ __device__ Pos2GridHash(
            const float2 lowestPoint,
            const float cellSize,
            const int2 &gridSize)
            : mLowestPoint(lowestPoint),
              mCellSize(cellSize),
              mGridSize(gridSize) {}

        __host__ __device__ uint operator()(const float2 &pos)
        {
            float2 relPos = make_float2(pos.x, pos.y) - mLowestPoint;
            int2 grid_xy = ComputeGridXYByPos2(relPos, mCellSize, mGridSize);
            return grid_xy.x * mGridSize.y  + grid_xy.y;
        }
    };

    struct Pos2GridXY
    {
        float2 mLowestPoint;
        float mCellSize;
        int2 mGridSize;

        __host__ __device__ Pos2GridXY(
            const float2 lowestPoint,
            const float cellSize,
            const int2 &gridSize)
            : mLowestPoint(lowestPoint),
              mCellSize(cellSize),
              mGridSize(gridSize) {}

        __host__ __device__ int2 operator()(const float2 &pos)
        {
            float2 relPos = make_float2(pos.x, pos.y) - mLowestPoint;
            return ComputeGridXYByPos2(relPos, mCellSize, mGridSize);
        }
    };

    struct GridXY2GridHash
    {
        int2 mGridSize;
        __host__ __device__ GridXY2GridHash(const int2 &gridSize)
            : mGridSize(gridSize) {}

        __host__ __device__ uint operator()(int x, int y)
        {
            return (x >= 0 && x < mGridSize.x && y >= 0 && y < mGridSize.y)
                       ? (x * mGridSize.y + y)
                       : (mGridSize.x * mGridSize.y);
        }
    };

} // namespace ThrustHelper

#endif