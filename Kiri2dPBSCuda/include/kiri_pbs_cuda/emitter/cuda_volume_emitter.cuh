/*
 * @Author: Xu.WANG
 * @Date: 2021-02-04 12:36:10
 * @LastEditTime: 2021-11-11 03:23:25
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\emitter\cuda_volume_emitter.cuh
 */

#ifndef _CUDA_VOLUME_EMITTER_CUH_
#define _CUDA_VOLUME_EMITTER_CUH_

#pragma once

#include <kiri_pbs_cuda/data/cuda_ns_pack.h>
#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace KIRI {

struct SphVolumeData {
  Vec_Float2 pos;
  Vec_Float3 col;
};

struct DemVolumeData {
  Vec_Float2 pos;
  Vec_Float3 col;
  Vec_Float mass;
};

struct MRDemVolumeData {
  Vec_Float2 pos;
  Vec_Float3 col;
  Vec_Float mass;
  Vec_Float radius;
};

struct DemShapeVolumeData {
  float minRadius;
  float maxRadius;
  Vec_Float2 pos;
  Vec_Float3 col;
  Vec_Float mass;
  Vec_Float radius;
};

struct DemNSBoxVolumeData {
  std::vector<ns_sphere_data> sphere_data;
  std::vector<non_spherical_particles> ns_data;
};

class CudaVolumeEmitter {
public:
  explicit CudaVolumeEmitter(bool enable = true) : bEnable(enable) {}

  CudaVolumeEmitter(const CudaVolumeEmitter &) = delete;
  CudaVolumeEmitter &operator=(const CudaVolumeEmitter &) = delete;
  virtual ~CudaVolumeEmitter() noexcept {}

  void BuildSphVolume(SphVolumeData &data, float2 lowest, int2 vsize,
                      float particleRadius, float3 color);
  void BuildUniDemVolume(DemVolumeData &data, float2 lowest, int2 vsize,
                         float particleRadius, float3 color, float mass,
                         float jitter = 0.001f);
  void BuildDemUniShapeVolume(DemShapeVolumeData &data, Vec_Float3 shape,
                              float3 color, float mass,
                              float2 offset = make_float2(0.f));
  void BuildMRDemShapeVolume(DemShapeVolumeData &data, float density,
                             Vec_Float3 shape, float3 color,
                             float2 offset = make_float2(0.f));

  void BuildRndNSDemBoxVolume(DemNSBoxVolumeData &data, float2 lower,
                              float2 upper, float coef_mean, float coef_fuzz,
                              int max_number,
                              const std::vector<NSPackPtr> &ns_types);

  inline constexpr bool GetEmitterStatus() const { return bEnable; }

private:
  bool bEnable;
};

typedef SharedPtr<CudaVolumeEmitter> CudaVolumeEmitterPtr;
} // namespace KIRI

#endif