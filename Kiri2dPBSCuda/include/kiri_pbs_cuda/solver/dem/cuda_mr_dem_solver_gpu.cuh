/*
 * @Author: Xu.WANG
 * @Date: 2020-07-04 14:48:23
 * @LastEditTime: 2021-11-10 01:53:28
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_mr_dem_solver_gpu.cuh
 */

#ifndef _CUDA_MRDEM_SOLVER_GPU_CUH_
#define _CUDA_MRDEM_SOLVER_GPU_CUH_

#pragma once

#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver_common_gpu.cuh>

namespace KIRI {

/**
 * @description: Multi-radius DEM method: normal force
 */
static __device__ void
ComputeMRDemForces(float2 *f, const size_t i, const float2 *pos,
                   const float2 *vel, const float *radius, const float young,
                   const float poisson, const float tanFrictionAngle,
                   const float dt, size_t j, const size_t cellEnd) {
  while (j < cellEnd) {

    if (i != j) {
      float2 dij = pos[i] - pos[j];
      float rij = radius[i] + radius[j];
      float2 vij = vel[j] - vel[i];
      float kni = young * radius[i];
      float knj = young * radius[j];
      float ksi = kni * poisson;
      float ksj = knj * poisson;

      float kn = 2.f * kni * knj / (kni + knj);
      float ks = 2.f * ksi * ksj / (ksi + ksj);

      ComputeDemForces(f, dij, vij, rij, kn, ks, tanFrictionAngle, dt);
    }
    ++j;
  }
  return;
}

template <typename AttenuFunc>
static __device__ void
ComputeMRDemCapillaryForces(float2 *f, const size_t i, const float2 *pos,
                            const float2 *vel, const float *radius,
                            const float sr, size_t j, const size_t cellEnd,
                            AttenuFunc G) {
  while (j < cellEnd) {

    if (i != j) {
      float2 dij = pos[i] - pos[j];
      float2 vij = vel[i] - vel[j];

      *f += ComputeMRDemCapillaryForces(dij, vij, radius[i], radius[j], sr, G);
    }
    ++j;
  }
  return;
}

template <typename Pos2GridXY, typename GridXY2GridHash, typename AttenuFunc>
__global__ void ComputeMRDemLinearMomentum_CUDA(
    const float2 *pos, const float2 *vel, float2 *acc, const float *mass,
    const float *radius, const float young, const float poisson,
    const float tanFrictionAngle, const float sr, const size_t num,
    const float2 lowestPoint, const float2 highestPoint, const float dt,
    size_t *cellStart, const int2 gridSize, Pos2GridXY p2xy,
    GridXY2GridHash xy2hash, AttenuFunc G) {
  const size_t i = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  if (i >= num)
    return;

  auto f = make_float2(0.f);
  int2 grid_xy = p2xy(pos[i]);

#pragma unroll
  for (int m = 0; m < 9; ++m) {
    int2 cur_grid_xy = grid_xy + make_int2(m / 3 - 1, m % 3 - 1);
    const size_t hash_idx = xy2hash(cur_grid_xy.x, cur_grid_xy.y);
    if (hash_idx == (gridSize.x * gridSize.y))
      continue;

    ComputeMRDemForces(&f, i, pos, vel, radius, young, poisson,
                       tanFrictionAngle, dt, cellStart[hash_idx],
                       cellStart[hash_idx + 1]);
    ComputeMRDemCapillaryForces(&f, i, pos, vel, radius, sr,
                                cellStart[hash_idx], cellStart[hash_idx + 1],
                                G);
  }

  ComputeDemWorldBoundaryForces(&f, pos[i], vel[i], radius[i], radius[i], young,
                                poisson, tanFrictionAngle, num, lowestPoint,
                                highestPoint, dt);

  acc[i] += 2.f * f / mass[i];
  return;
}

} // namespace KIRI

#endif /* _CUDA_DEM_SOLVER_GPU_CUH_ */