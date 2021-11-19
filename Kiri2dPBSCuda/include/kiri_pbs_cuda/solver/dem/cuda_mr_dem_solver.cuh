
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-01 14:31:30
 * @LastEditTime: 2021-11-19 16:43:24
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_mr_dem_solver.cuh
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_mr_dem_solver.cuh
 */

#ifndef _CUDA_MRDEM_SOLVER_CUH_
#define _CUDA_MRDEM_SOLVER_CUH_

#pragma once

#include <kiri_pbs_cuda/particle/cuda_mr_dem_particles.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver.cuh>

namespace KIRI {
class CudaMRDemSolver : public CudaDemSolver {
public:
  virtual void UpdateSolver(CudaDemParticlesPtr &sands,
                            const CudaArray<size_t> &cellStart,
                            float timeIntervalInSeconds, CudaDemParams params,
                            CudaBoundaryParams bparams) override;

  explicit CudaMRDemSolver(const size_t num) : CudaDemSolver(num) {}

  virtual ~CudaMRDemSolver() noexcept {}

protected:
  virtual void ComputeMRDemLinearMomentum(
      CudaMRDemParticlesPtr &sands, const float young, const float poisson,
      const float tanFrictionAngle, const float c0, const float dt,const float boundaryRadius,
      const CudaArray<size_t> &cellStart, const float2 lowestPoint,
      const float2 highestPoint, const float kernelRadius, const int2 gridSize);
};

typedef SharedPtr<CudaMRDemSolver> CudaMRDemSolverPtr;
} // namespace KIRI

#endif /* _CUDA_MRDEM_SOLVER_CUH_ */