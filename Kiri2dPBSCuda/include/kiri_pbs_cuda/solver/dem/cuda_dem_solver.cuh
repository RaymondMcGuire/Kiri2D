
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-01 14:31:30
 * @LastEditTime: 2021-11-19 16:43:18
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_dem_solver.cuh
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\dem\cuda_dem_solver.cuh
 */

#ifndef _CUDA_DEM_SOLVER_CUH_
#define _CUDA_DEM_SOLVER_CUH_

#pragma once

#include <kiri_pbs_cuda/solver/cuda_base_solver.cuh>

#include <kiri_pbs_cuda/data/cuda_boundary_params.h>
#include <kiri_pbs_cuda/data/cuda_dem_params.h>

#include <kiri_pbs_cuda/particle/cuda_dem_particles.cuh>

namespace KIRI {
class CudaDemSolver : public CudaBaseSolver {
public:
  virtual void UpdateSolver(CudaDemParticlesPtr &sands,
                            const CudaArray<size_t> &cellStart,
                            float timeIntervalInSeconds, CudaDemParams params,
                            CudaBoundaryParams bparams);

  explicit CudaDemSolver(const size_t num) : CudaBaseSolver(num) {}

  virtual ~CudaDemSolver()  {}

protected:
  virtual void ExtraForces(CudaDemParticlesPtr &sands, const float2 gravity);

  virtual void Advect(CudaDemParticlesPtr &sands, const float dt,
                      const float damping);

  virtual void
  ComputeDemLinearMomentum(CudaDemParticlesPtr &sands, const float radius,
                           const float young, const float poisson,
                           const float tanFrictionAngle, const float c0,
                           const float dt,const float boundaryRadius, const CudaArray<size_t> &cellStart,
                           const float2 lowestPoint, const float2 highestPoint,
                           const float kernelRadius, const int2 gridSize);
};

typedef SharedPtr<CudaDemSolver> CudaDemSolverPtr;
} // namespace KIRI

#endif /* _CUDA_DEM_SOLVER_CUH_ */