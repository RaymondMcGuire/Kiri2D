
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-01 14:31:30
 * @LastEditTime: 2021-11-19 17:43:55
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_non_spherical_solver.cuh
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_non_spherical_solver.cuh
 */

#ifndef _CUDA_NON_SPHERICAL_SOLVER_CUH_
#define _CUDA_NON_SPHERICAL_SOLVER_CUH_

#pragma once

#include <kiri_pbs_cuda/solver/cuda_base_solver.cuh>

#include <kiri_pbs_cuda/data/cuda_boundary_params.h>
#include <kiri_pbs_cuda/data/cuda_dem_params.h>

#include <kiri_pbs_cuda/particle/cuda_non_spherical_particles.cuh>

namespace KIRI {
class CudaNonSphericalSolver : public CudaBaseSolver {
public:
  virtual void UpdateSolver(CudaNonSphericalParticlesPtr &sands,
                            const CudaArray<size_t> &cellStart,
                            float timeIntervalInSeconds,
                            const CudaDemNonSphericalParams params,
                            CudaBoundaryParams bparams);

  explicit CudaNonSphericalSolver(const size_t num, const size_t group_num)
      : CudaBaseSolver(num), mCudaGridGroupSize(CuCeilDiv(group_num, 8)) {}

  virtual ~CudaNonSphericalSolver() noexcept {}

protected:
  void ComputeNSDemLinearMomentum(
      CudaNonSphericalParticlesPtr &sands, const float young,
      const float poisson, const float tanFrictionAngle, const float dt,const float boundaryRadius,
      const CudaArray<size_t> &cellStart, const float2 lowestPoint,
      const float2 highestPoint, const float kernelRadius, const int2 gridSize);

  void ComputeNSMomentum(CudaNonSphericalParticlesPtr &sands,
                         const CudaDemNonSphericalParams params);

  void
  NonSphericalParticlesTimeIntegration(CudaNonSphericalParticlesPtr &sands,
                                       const CudaDemNonSphericalParams params,
                                       const float boundaryRadius,
                                       const float2 lowestPoint,
                                       const float2 highestPoint);

  size_t mCudaGridGroupSize;
};

typedef SharedPtr<CudaNonSphericalSolver> CudaNonSphericalSolverPtr;
} // namespace KIRI

#endif /* _CUDA_NON_SPHERICAL_SOLVER_CUH_ */