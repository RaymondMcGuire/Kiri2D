/*
 * @Author: Xu.WANG
 * @Date: 2021-02-25 01:18:29
 * @LastEditTime: 2022-01-05 22:51:32
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri\KiriPBSCuda\include\kiri_pbs_cuda\sampler\cuda_shape_sampler_common_gpu.cuh
 */

#ifndef _CUDA_SHAPE_SAMPLER_COMMON_GPU_CUH_
#define _CUDA_SHAPE_SAMPLER_COMMON_GPU_CUH_

#pragma once

#include <curand.h>
#include <curand_kernel.h>
#include <kiri_pbs_cuda/sampler/cuda_sampler_struct.cuh>

namespace KIRI2D {

// generates a random float between 0 and 1
static __device__ float RndFloat(curandState *globalState, int ind) {
  curandState local_state = globalState[ind];
  float val = curand_uniform(&local_state);
  globalState[ind] = local_state;
  return val;
}

static __global__ void SetUpRndGen_CUDA(curandState *state,
                                        unsigned long seed) {
  int id = threadIdx.x;
  curand_init(seed, id, 0, &state[id]);
}

// Set a bit in the giant voxel table. This involves doing an atomic operation
// on a 32-bit word in memory. Blocking other threads writing to it for a very
// short time
__device__ __inline__ void setBit(size_t *samplerTable, size_t index) {
  size_t int_location = index / size_t(32);
  size_t bit_pos =
      size_t(31) -
      (index % size_t(32)); // we count bit positions RtL, but array indices LtR
  size_t mask = 1 << bit_pos;
  atomicOr(&(samplerTable[int_location]), mask);
}

// use Xor for voxels whose corresponding bits have to flipped
__device__ __inline__ void setBitXor(size_t *samplerTable, size_t index) {
  size_t int_location = index / size_t(32);
  size_t bitPos =
      size_t(31) -
      (index % size_t(32)); // we count bit positions RtL, but array indices LtR
  size_t mask = 1 << bitPos;
  atomicXor(&(samplerTable[int_location]), mask);
}

__host__ __device__ __inline__ bool
CheckSamplerTable(size_t x, size_t y, size_t z, const int3 gridSize,
                  const size_t *samplerTable) {
  size_t idx = x + (y * gridSize.y) + (z * gridSize.y * gridSize.z);
  size_t intIdx = idx / size_t(32);

  unsigned int bitPos =
      size_t(31) -
      (idx % size_t(32)); // we count bit positions RtL, but array indices LtR
  if ((samplerTable[intIdx]) & (1 << bitPos)) {
    return true;
  }
  return false;
}

// check the triangle is counterclockwise or not
__device__ inline bool checkCCW(float2 v0, float2 v1, float2 v2) {
  float2 e0 = v1 - v0;
  float2 e1 = v2 - v0;
  float result = e0.x * e1.y - e1.x * e0.y;
  if (result > 0)
    return true;
  else
    return false;
}

// check the location with point and triangle
__device__ inline int check_point_triangle(float2 v0, float2 v1, float2 v2,
                                           float2 point) {
  float2 PA = point - v0;
  float2 PB = point - v1;
  float2 PC = point - v2;

  float t1 = PA.x * PB.y - PA.y * PB.x;
  if (fabs(t1) < KIRI_EPSILON && PA.x * PB.x <= 0 && PA.y * PB.y <= 0)
    return 1;

  float t2 = PB.x * PC.y - PB.y * PC.x;
  if (fabs(t2) < KIRI_EPSILON && PB.x * PC.x <= 0 && PB.y * PC.y <= 0)
    return 2;

  float t3 = PC.x * PA.y - PC.y * PA.x;
  if (fabs(t3) < KIRI_EPSILON && PC.x * PA.x <= 0 && PC.y * PA.y <= 0)
    return 3;

  if (t1 * t2 > 0 && t1 * t3 > 0)
    return 0;
  else
    return -1;
}

// top-left rule
__device__ inline bool TopLeftEdge(float2 v0, float2 v1) {
  return ((v1.y < v0.y) || (v1.y == v0.y && v0.x > v1.x));
}

// find the x coordinate of the voxel
__device__ inline float get_x_coordinate(float3 n, float3 v0, float2 point) {
  return (-(n.y * (point.x - v0.y) + n.z * (point.y - v0.z)) / n.x + v0.x);
}

// adapted from:
// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
static __device__ bool intersects(float3 V1, float3 V2, float3 V3, float3 dir,
                                  float3 pos) {
  // Find vectors for two edges sharing V1
  float3 e1 = V2 - V1;
  float3 e2 = V3 - V1;

  // //Begin calculating determinant - also used to calculate u parameter
  float3 P = cross(dir, e2);

  // if determinant is near zero, ray lies in plane of triangle
  float det = dot(e1, P);

  // NOT CULLING
  if (det > -KIRI_EPSILON && det < KIRI_EPSILON)
    return false;
  float inv_det = 1.f / det;

  // calculate distance from V1 to ray origin
  float3 T = pos - V1;
  // Calculate u parameter and test bound
  float u = dot(T, P) * inv_det;
  // The intersection lies outside of the triangle
  if (u < 0.f || u > 1.f)
    return false;

  // Prepare to test v parameter
  float3 Q = cross(T, e1);
  // Calculate V parameter and test bound
  float v = dot(dir, Q) * inv_det;
  // The intersection lies outside of the triangle
  if (v < 0.f || u + v > 1.f)
    return false;

  float t = dot(e2, Q) * inv_det;

  if (t > KIRI_EPSILON) { // ray intersection
    return true;
  }

  // No hit, no win
  return false;
}

} // namespace KIRI2D

#endif /* _CUDA_SHAPE_SAMPLER_COMMON_GPU_CUH_ */