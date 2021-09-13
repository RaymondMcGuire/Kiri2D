/*
 * @Author: Xu.WANG
 * @Date: 2020-07-04 14:48:23
 * @LastEditTime: 2021-09-13 15:46:49
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_dem_solver_gpu.cuh
 */

#ifndef _CUDA_DEM_SOLVER_GPU_CUH_
#define _CUDA_DEM_SOLVER_GPU_CUH_

#pragma once

#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver_common_gpu.cuh>

namespace KIRI
{

    /**
     * @description: Uniform radius DEM method: normal force + shear force
     */
    static __device__ void ComputeUniRadiusDemForces(
        float2 *f,
        const size_t i,
        float2 *pos,
        float2 *vel,
        const float radius,
        const float young,
        const float poisson,
        const float tanFrictionAngle,
        size_t j,
        const size_t cellEnd)
    {
        while (j < cellEnd)
        {

            if (i != j)
            {
                float2 dij = pos[i] - pos[j];
                float rij = 2.f * radius;
                float2 vij = vel[j] - vel[i];
                float kn = young * radius;
                float ks = kn * poisson;
                
                *f += ComputeDemForces(dij, vij, rij, kn, ks, tanFrictionAngle);
            }
            ++j;
        }
        return;
    }

    template <typename AttenuFunc>
    static __device__ void ComputeUniRadiusDemCapillaryForces(
        float2 *f,
        const size_t i,
        float2 *pos,
        float2 *vel,
        const float radius,
        const float sr,
        size_t j,
        const size_t cellEnd,
        AttenuFunc G)
    {
        while (j < cellEnd)
        {

            if (i != j)
            {
                float2 dij = pos[i] - pos[j];
                float2 vij = vel[i] - vel[j];

                *f += ComputeDemCapillaryForces(dij, vij, radius, sr, G);
            }
            ++j;
        }
        return;
    }

    template <typename Pos2GridXY, typename GridXY2GridHash, typename AttenuFunc>
    __global__ void ComputeDemLinearMomentum_CUDA(
        float2 *pos,
        float2 *vel,
        float2 *acc,
        float *mass,
        const float radius,
        const float young,
        const float poisson,
        const float tanFrictionAngle,
        const float sr,
        const size_t num,
        const float2 lowestPoint,
        const float2 highestPoint,
        size_t *cellStart,
        const int2 gridSize,
        Pos2GridXY p2xy,
        GridXY2GridHash xy2hash,
        AttenuFunc G)
    {
        const size_t i = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
        if (i >= num)
            return;

        auto f = make_float2(0.f);
        int2 grid_xy = p2xy(pos[i]);
        
        // if(id[i] == i)
        //     printf("i= %d,pid= %d; pos= %.8f,%.8f\n", i,id[i],pos[i].x, pos[i].y);
        // else
        //     printf("i= %d,pid= %d\n", i,id[i]);

#pragma unroll
        for (int m = 0; m < 9; ++m)
        {
            int2 cur_grid_xy = grid_xy + make_int2(m / 3 - 1, m % 3 - 1);
            const size_t hash_idx = xy2hash(cur_grid_xy.x, cur_grid_xy.y);
            if (hash_idx == (gridSize.x * gridSize.y))
                continue;
            
            ComputeUniRadiusDemForces(&f, i, pos, vel, radius, young, poisson, tanFrictionAngle, cellStart[hash_idx], cellStart[hash_idx + 1]);
            ComputeUniRadiusDemCapillaryForces(&f, i, pos, vel, radius, sr, cellStart[hash_idx], cellStart[hash_idx + 1], G);
        }

        ComputeDemWorldBoundaryForces(&f, pos[i], -vel[i], radius, radius, young, poisson, tanFrictionAngle, num, lowestPoint, highestPoint);

        acc[i] += f / mass[i];
        return;
    }

} // namespace KIRI

#endif /* _CUDA_DEM_SOLVER_GPU_CUH_ */