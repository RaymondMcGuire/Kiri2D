/*
 * @Author: Xu.WANG
 * @Date: 2021-02-04 12:36:10
 * @LastEditTime: 2021-11-11 19:33:57
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\particle\cuda_non_spherical_particles.cuh
 */

#ifndef _CUDA_NON_SPHERICAL_PARTICLES_CUH_
#define _CUDA_NON_SPHERICAL_PARTICLES_CUH_

#pragma once

#include <kiri_pbs_cuda/data/cuda_dem_params.h>
#include <kiri_pbs_cuda/particle/cuda_particles.cuh>

namespace KIRI {

class CudaNonSphericalParticles : public CudaParticles {
public:
  explicit CudaNonSphericalParticles::CudaNonSphericalParticles(
      const uint numOfMaxParticles, const uint numOfMaxNsParticles)
      : CudaParticles(numOfMaxParticles), mVel(numOfMaxParticles),
        mRadius(numOfMaxParticles), mCol(numOfMaxParticles),
        mNsMapping(numOfMaxParticles), mNsParticles(numOfMaxNsParticles) {}

  explicit CudaNonSphericalParticles::CudaNonSphericalParticles(
      const Vec_Float2 &p, const Vec_Float3 &col, const Vec_Float &rad,
      const std::vector<non_spherical_particles> &ns,
      const std::vector<ns_mapping> &map)
      : CudaParticles(p), mVel(p.size()), mCol(p.size()), mRadius(p.size()),
        mNsMapping(p.size()), mNsParticles(ns.size()) {

    KIRI_CUCALL(cudaMemcpy(mCol.Data(), &col[0], sizeof(float3) * col.size(),
                           cudaMemcpyHostToDevice));

    KIRI_CUCALL(cudaMemcpy(mRadius.Data(), &rad[0], sizeof(float) * rad.size(),
                           cudaMemcpyHostToDevice));

    KIRI_CUCALL(cudaMemcpy(mNsParticles.Data(), &ns[0],
                           sizeof(non_spherical_particles) * ns.size(),
                           cudaMemcpyHostToDevice));

    KIRI_CUCALL(cudaMemcpy(mNsMapping.Data(), &map[0],
                           sizeof(ns_mapping) * map.size(),
                           cudaMemcpyHostToDevice));
  }

  virtual ~CudaNonSphericalParticles() noexcept {}
  CudaNonSphericalParticles(const CudaNonSphericalParticles &) = delete;
  CudaNonSphericalParticles &
  operator=(const CudaNonSphericalParticles &) = delete;

  void Advect(const float dt, const float damping);

  inline float *GetRadiusPtr() const { return mRadius.Data(); }
  inline float2 *GetVelPtr() const { return mVel.Data(); }
  inline float3 *GetColPtr() const { return mCol.Data(); }

  inline ns_mapping *GetNSMappingPtr() const { return mNsMapping.Data(); }
  inline non_spherical_particles *GetNSParticlesPtr() const {
    return mNsParticles.Data();
  }

protected:
  CudaArray<float2> mVel;
  CudaArray<float3> mCol;
  CudaArray<float> mRadius;

  CudaArray<ns_mapping> mNsMapping;
  CudaArray<non_spherical_particles> mNsParticles;
};

typedef SharedPtr<CudaNonSphericalParticles> CudaNonSphericalParticlesPtr;
} // namespace KIRI

#endif