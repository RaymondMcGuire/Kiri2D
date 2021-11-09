/*
 * @Author: Xu.WANG
 * @Date: 2021-02-04 12:36:10
 * @LastEditTime: 2021-11-09 16:09:45
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
      const uint numOfMaxParticles)
      : CudaParticles(numOfMaxParticles), mId(numOfMaxParticles),
        mVel(numOfMaxParticles), mAcc(numOfMaxParticles),
        mCol(numOfMaxParticles), mMass(numOfMaxParticles),
        mNsMapping(numOfMaxParticles) {}

  explicit CudaNonSphericalParticles::CudaNonSphericalParticles(
      const Vec_Float2 &p, const Vec_Float3 &col, const Vec_Float &mass)
      : CudaParticles(p), mId(p.size()), mVel(p.size()), mAcc(p.size()),
        mCol(p.size()), mMass(p.size()), mNsMapping(p.size()) {
    KIRI_CUCALL(cudaMemcpy(mCol.Data(), &col[0], sizeof(float3) * col.size(),
                           cudaMemcpyHostToDevice));
    KIRI_CUCALL(cudaMemcpy(mMass.Data(), &mass[0], sizeof(float) * mass.size(),
                           cudaMemcpyHostToDevice));
  }

  virtual ~CudaNonSphericalParticles() noexcept {}
  CudaNonSphericalParticles(const CudaNonSphericalParticles &) = delete;
  CudaNonSphericalParticles &
  operator=(const CudaNonSphericalParticles &) = delete;

  void Advect(const float dt, const float damping);

  inline uint *GetIdPtr() const { return mId.Data(); }

  inline float2 *GetVelPtr() const { return mVel.Data(); }
  inline float2 *GetAccPtr() const { return mAcc.Data(); }
  inline float3 *GetColPtr() const { return mCol.Data(); }
  inline float *GetMassPtr() const { return mMass.Data(); }

  inline ns_mapping *GetNSMappingPtr() const { return mNsMapping.Data(); }

protected:
  CudaArray<uint> mId;

  CudaArray<float2> mVel;
  CudaArray<float2> mAcc;
  CudaArray<float3> mCol;
  CudaArray<float> mMass;

  CudaArray<ns_mapping> mNsMapping;
};

typedef SharedPtr<CudaNonSphericalParticles> CudaNonSphericalParticlesPtr;
} // namespace KIRI

#endif