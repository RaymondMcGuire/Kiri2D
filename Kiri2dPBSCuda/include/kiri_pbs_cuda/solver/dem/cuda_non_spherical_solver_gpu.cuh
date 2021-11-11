/*
 * @Author: Xu.WANG
 * @Date: 2020-07-04 14:48:23
 * @LastEditTime: 2021-11-11 20:32:41
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_mr_dem_solver_gpu.cuh
 */

#ifndef _CUDA_NON_SPHERICAL_SOLVER_GPU_CUH_
#define _CUDA_NON_SPHERICAL_SOLVER_GPU_CUH_

#pragma once

#include <kiri_pbs_cuda/data/cuda_dem_params.h>
#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver_common_gpu.cuh>

namespace KIRI {

static __device__ void
ComputeNSDemForces(float2 *f, float *torque, const size_t i,
                   const ns_mapping *map, const float2 *pos, const float2 *vel,
                   const float *radius, const float young, const float poisson,
                   const float tanFrictionAngle, const float dt, size_t j,
                   const size_t cellEnd) {
  while (j < cellEnd) {

    if (map[i].ns_id != map[j].ns_id) {
      float2 dij = pos[i] - pos[j];
      float rij = radius[i] + radius[j];
      float2 vij = vel[j] - vel[i];
      float kni = young * radius[i];
      float knj = young * radius[j];
      float ksi = kni * poisson;
      float ksj = knj * poisson;

      float kn = 2.f * kni * knj / (kni + knj);
      float ks = 2.f * ksi * ksj / (ksi + ksj);

      ComputeDemForcesAndTorque(f, torque, dij, vij, radius[i], rij, kn, ks,
                                tanFrictionAngle, dt);
    }
    ++j;
  }
  return;
}

template <typename Pos2GridXY, typename GridXY2GridHash>
__global__ void ComputeNSDemLinearMomentum_CUDA(
    non_spherical_particles *nsParticles, const ns_mapping *map,
    const float2 *pos, const float2 *vel, const float *radius,
    const float young, const float poisson, const float tanFrictionAngle,
    const size_t num, const float2 lowestPoint, const float2 highestPoint,
    const float dt, size_t *cellStart, const int2 gridSize, Pos2GridXY p2xy,
    GridXY2GridHash xy2hash) {
  const size_t i = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  if (i >= num)
    return;

  auto f = make_float2(0.f);
  auto torque = 0.f;

  int2 grid_xy = p2xy(pos[i]);

#pragma unroll
  for (int m = 0; m < 9; ++m) {
    int2 cur_grid_xy = grid_xy + make_int2(m / 3 - 1, m % 3 - 1);
    const size_t hash_idx = xy2hash(cur_grid_xy.x, cur_grid_xy.y);
    if (hash_idx == (gridSize.x * gridSize.y))
      continue;

    ComputeNSDemForces(&f, &torque, i, map, pos, vel, radius, young, poisson,
                       tanFrictionAngle, dt, cellStart[hash_idx],
                       cellStart[hash_idx + 1]);
  }

  ComputeNSDemWorldBoundaryForces(&f, &torque, pos[i], vel[i], radius[i],
                                  radius[i], young, poisson, tanFrictionAngle,
                                  num, lowestPoint, highestPoint, dt);

  nsParticles[map[i].ns_id].force_list[map[i].sub_id] = f;
  nsParticles[map[i].ns_id].torque_list[map[i].sub_id] = torque;

  return;
}

__global__ void ComputeNSMomentum_CUDA(non_spherical_particles *nsParticles,
                                       CudaDemNonSphericalParams params) {
  int i = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  if (i >= params.group_num)
    return;

  float2 force = make_float2(0.f);
  float torque = 0.f;
  for (size_t sub_idx = 0; sub_idx < nsParticles[i].sub_num; sub_idx++) {
    force += nsParticles[i].force_list[sub_idx];
    torque += nsParticles[i].torque_list[sub_idx];
  }

  __syncthreads();

  // rigidBody Velocity
  float2 acc = force / nsParticles[i].mass + params.gravity;

  printf("acc=%.3f,%.3f \n", acc.x, acc.y);

  float2 sgn_val = sgn(acc * (nsParticles[i].vel + 0.5f * params.dt * acc));
  acc *= make_float2(1.f) - params.damping * sgn_val;

  // angular Velocity
  float angle_acc = torque / nsParticles[i].inertia;
  float sgn_ang_val = sgn(
      angle_acc * (nsParticles[i].angle_vel + 0.5f * params.dt * angle_acc));
  angle_acc *= 1.f - params.damping * sgn_ang_val;

  nsParticles[i].angle_vel += params.dt * angle_acc;

  rotation2 rot2 = make_rotation2(nsParticles[i].angle_vel * params.dt);
  nsParticles[i].rot = rot2.mat * nsParticles[i].rot;

  //
  nsParticles[i].vel += acc * params.dt;
  // rigidBody Position
  nsParticles[i].centroid += params.dt * nsParticles[i].vel;
}

__global__ void
NSTimeIntegration_CUDA(float2 *pos, float2 *vel,
                       const non_spherical_particles *nsParticles,
                       const ns_mapping *map, const size_t num,
                       const CudaDemNonSphericalParams params) {
  int i = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  if (i >= num)
    return;

  non_spherical_particles ns = nsParticles[map[i].ns_id];

  // printf("cen=%.3f,%.3f \n", ns.centroid.x, ns.centroid.y);

  pos[i] = ns.centroid + ns.rot * map[i].rel_pos;

  vel[i] = ns.vel + ns.angle_vel * (pos[i] - ns.centroid);
}

} // namespace KIRI

#endif /* _CUDA_DEM_SOLVER_GPU_CUH_ */