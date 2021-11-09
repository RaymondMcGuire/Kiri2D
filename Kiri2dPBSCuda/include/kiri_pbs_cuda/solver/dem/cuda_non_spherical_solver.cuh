
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-01 14:31:30
 * @LastEditTime: 2021-11-08 01:21:00
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
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
                            float timeIntervalInSeconds, CudaDemParams params,
                            CudaBoundaryParams bparams);

  explicit CudaNonSphericalSolver(const size_t num) : CudaBaseSolver(num) {}

  virtual ~CudaNonSphericalSolver() noexcept {}

protected:
  virtual void ExtraForces(CudaNonSphericalParticlesPtr &sands,
                           const float2 gravity);

  virtual void Advect(CudaNonSphericalParticlesPtr &sands, const float dt,
                      const float damping);
};

typedef SharedPtr<CudaNonSphericalSolver> CudaNonSphericalSolverPtr;
} // namespace KIRI

#endif /* _CUDA_NON_SPHERICAL_SOLVER_CUH_ */