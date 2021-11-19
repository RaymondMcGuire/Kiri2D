/*
 * @Author: Xu.WANG
 * @Date: 2020-07-04 14:48:23
 * @LastEditTime: 2021-11-19 20:47:03
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_dem_solver_common_gpu.cuh
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_dem_solver_common_gpu.cuh
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_dem_solver_common_gpu.cuh
 */

#ifndef _CUDA_DEM_SOLVER_COMMON_GPU_CUH_
#define _CUDA_DEM_SOLVER_COMMON_GPU_CUH_

#pragma once

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace KIRI {

static __device__ void ComputeDemForces(float2 *f, float2 dij, float2 vij,
                                        float rij, float kn, float ks,
                                        float tanFrictionAngle, float dt) {
  float dist = length(dij);

  float penetration_depth = rij - dist;
  float2 n = dij / dist;
  float2 normal_force = kn * max(penetration_depth, 0.f) * n;

  float dot_epslion = dot(vij, n);
  float2 vij_tangential = vij - dot_epslion * n;
  float2 shear_inc = vij_tangential * dt;
  float2 shear_force = -ks * shear_inc;

  float max_fs = lengthSquared(normal_force) * std::powf(tanFrictionAngle, 2.f);

  if (lengthSquared(shear_force) > max_fs) {
    float ratio = sqrt(max_fs) / length(shear_force);
    shear_force *= ratio;
  }

  *f += normal_force + shear_force;
}

static __device__ void ComputeDemForcesAndTorque(
    float2 *f, float *torque, const float ang_veli, float ang_velj,
    const float2 dij, const float2 vij, const float ri, const float rij,
    const float kn, const float ks, const float tanFrictionAngle, float dt) {
  float dist = length(dij);

  float penetration_depth = rij - dist;

  if (penetration_depth <= 0.f)
    return;

  float2 n = dij / dist;
  float2 normal_force = kn * penetration_depth * n;

  float alpha = rij / penetration_depth;
  float2 del = make_float2(-n.y, n.x);
  float2 n_vij =
      vij * alpha - ang_veli * ri * del - ang_velj * (rij - ri) * del;

  float dot_epslion = dot(n_vij, n);
  float2 vij_tangential = n_vij - dot_epslion * n;
  float2 shear_inc = vij_tangential * dt;
  float2 shear_force = -ks * shear_inc;

  float max_fs = lengthSquared(normal_force) * std::powf(tanFrictionAngle, 2.f);

  if (lengthSquared(shear_force) > max_fs) {
    float ratio = sqrt(max_fs) / length(shear_force);
    shear_force *= ratio;
  }

  float2 tf = normal_force + shear_force;

  float theta = atan2(tf.y, tf.x) - atan2(n.y, n.x);
  theta = length(tf) * sin(theta) * 1e-3f;
  // theta = sin(theta);

  // if(theta!=theta)
  // printf("rij=%.3f, dist=%.3f \n", rij,dist);

  *f += tf;
  *torque += (ri - 0.5f * penetration_depth) * theta;
}

template <typename AttenuFunc>
static __device__ float2 ComputeDemCapillaryForces(float2 dij, float2 vij,
                                                   const float radiusi,
                                                   const float sr,
                                                   AttenuFunc G) {
  float2 f = make_float2(0.f);
  // rupture distance
  float contact_angle = 30.f / 180.f * KIRI_PI;
  float volume_liquid_bridge =
      4.f / 3.f * KIRI_PI * powf(radiusi, 3.f) * 0.01f * 0.01f;
  float s_rupture = (1.f + 0.5f * contact_angle) *
                    (powf(volume_liquid_bridge, 1.f / 3.f) +
                     0.1f * powf(volume_liquid_bridge, 2.f / 3.f));

  float dist = length(dij);
  float H = dist - (radiusi + radiusi);
  if (H < s_rupture && H > 0.f) {
    float2 N = dij / dist;
    float dot_epslion = dot(vij, N);

    float2 vij_normal = dot_epslion * N;
    float2 vij_tangential = vij - dot_epslion * N;

    // float coeff_c = csat + (1.f - sr) * (c0 - csat);
    float coeff_c = G(sr);

    float d = -H + sqrtf(H * H + volume_liquid_bridge / (KIRI_PI * radiusi));
    float phi = sqrtf(2.f * H / radiusi *
                      (-1.f + sqrtf(1.f + volume_liquid_bridge /
                                              (KIRI_PI * radiusi * H * H))));
    float neck_curvature_pressure = -2.f * KIRI_PI * coeff_c * radiusi *
                                    cosf(contact_angle) / (1.f + H / (2.f * d));
    float surface_tension_force =
        -2.f * KIRI_PI * coeff_c * radiusi * phi * sinf(contact_angle);

    f = N * (neck_curvature_pressure + surface_tension_force);
  }
  return f;
}

template <typename AttenuFunc>
static __device__ float2 ComputeMRDemCapillaryForces(float2 dij, float2 vij,
                                                     const float radiusi,
                                                     const float radiusj,
                                                     const float sr,
                                                     AttenuFunc G) {
  float2 f = make_float2(0.f);
  // rupture distance
  float contact_angle = 30.f / 180.f * KIRI_PI;
  float volume_liquid_bridge = 4.f / 3.f * KIRI_PI *
                               powf((radiusi + radiusj) / 2.f, 3.f) * 0.01f *
                               0.01f;
  float s_rupture = (1.f + 0.5f * contact_angle) *
                    (powf(volume_liquid_bridge, 1.f / 3.f) +
                     0.1f * powf(volume_liquid_bridge, 2.f / 3.f));

  float avg_radius = (radiusi + radiusj) / 2.f;

  float dist = length(dij);
  float H = dist - (radiusi + radiusj);
  if (H < s_rupture && H > 0.f) {
    float2 N = dij / dist;
    float dot_epslion = dot(vij, N);

    float2 vij_normal = dot_epslion * N;
    float2 vij_tangential = vij - dot_epslion * N;

    // float coeff_c = csat + (1.f - sr) * (c0 - csat);
    float coeff_c = G(sr);

    float d = -H + sqrtf(H * H + volume_liquid_bridge / (KIRI_PI * avg_radius));
    float phi = sqrtf(2.f * H / avg_radius *
                      (-1.f + sqrtf(1.f + volume_liquid_bridge /
                                              (KIRI_PI * avg_radius * H * H))));
    float neck_curvature_pressure = -2.f * KIRI_PI * coeff_c * avg_radius *
                                    cosf(contact_angle) / (1.f + H / (2.f * d));
    float surface_tension_force =
        -2.f * KIRI_PI * coeff_c * avg_radius * phi * sinf(contact_angle);

    f = N * (neck_curvature_pressure + surface_tension_force);
  }
  return f;
}

static __device__ void ComputeDemWorldBoundaryForces(
    float2 *f, float2 posi, float2 veli, const float radiusi,
    const float boundaryRadius, const float young, const float poisson,
    const float tanFrictionAngle, const size_t num, const float2 lowestPoint,
    const float2 highestPoint, const float dt) {
  const size_t i = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  if (i >= num)
    return;

  float rij = boundaryRadius + radiusi;
  float ri2 = radiusi + radiusi;
  float kn = young * radiusi;
  float ks = kn * poisson;

  float2 N = make_float2(0.f);
  float diff = 0.f;

  if (posi.x > highestPoint.x - rij) {
    N = make_float2(-1.f, 0.f);
    diff = abs(posi.x - highestPoint.x);
    ComputeDemForces(f, N * diff, veli, ri2, kn, ks, tanFrictionAngle, dt);
  }

  if (posi.x < lowestPoint.x + rij) {
    N = make_float2(1.f, 0.f);
    diff = abs(posi.x - lowestPoint.x);
    ComputeDemForces(f, N * diff, veli, ri2, kn, ks, tanFrictionAngle, dt);
  }

  if (posi.y > highestPoint.y - rij) {
    N = make_float2(0.f, -1.f);
    diff = abs(posi.y - highestPoint.y);
    ComputeDemForces(f, N * diff, veli, ri2, kn, ks, tanFrictionAngle, dt);
  }

  if (posi.y < lowestPoint.y + rij) {
    N = make_float2(0.f, 1.f);
    diff = abs(posi.y - lowestPoint.y);
    ComputeDemForces(f, N * diff, veli, ri2, kn, ks, tanFrictionAngle, dt);
  }

  return;
}

static __device__ void ComputeNSDemWorldBoundaryForces(
    float2 *f, float *torque, const float2 posi, const float2 veli,
    const float ang_veli, const float radiusi, const float boundaryRadius,
    const float young, const float poisson, const float tanFrictionAngle,
    const size_t num, const float2 lowestPoint, const float2 highestPoint,
    const float dt) {
  const size_t i = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  if (i >= num)
    return;

  float rij = boundaryRadius + radiusi;
  float ri2 = radiusi + radiusi;
  float kn = young * radiusi;
  float ks = kn * poisson;

  float2 N = make_float2(0.f);
  float diff = 0.f;

  if (posi.x > highestPoint.x - rij) {
    N = make_float2(-1.f, 0.f);
    diff = abs(posi.x - (highestPoint.x - boundaryRadius));
    ComputeDemForcesAndTorque(f, torque, ang_veli, 0.f, N * diff, veli, radiusi,
                              ri2, kn, ks, tanFrictionAngle, dt);
  }

  if (posi.x < lowestPoint.x + rij) {
    N = make_float2(1.f, 0.f);
    diff = abs(posi.x - (lowestPoint.x + boundaryRadius));
    ComputeDemForcesAndTorque(f, torque, ang_veli, 0.f, N * diff, veli, radiusi,
                              ri2, kn, ks, tanFrictionAngle, dt);
  }

  if (posi.y > highestPoint.y - rij) {
    N = make_float2(0.f, -1.f);
    diff = abs(posi.y - (highestPoint.y - boundaryRadius));
    ComputeDemForcesAndTorque(f, torque, ang_veli, 0.f, N * diff, veli, radiusi,
                              ri2, kn, ks, tanFrictionAngle, dt);
  }

  if (posi.y < lowestPoint.y + rij) {
    N = make_float2(0.f, 1.f);
    diff = abs(posi.y - (lowestPoint.y + boundaryRadius));
    ComputeDemForcesAndTorque(f, torque, ang_veli, 0.f, N * diff, veli, radiusi,
                              ri2, kn, ks, tanFrictionAngle, dt);
  }

  return;
}

} // namespace KIRI

#endif /* _CUDA_DEM_SOLVER_COMMON_GPU_CUH_ */