/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 22:52:09
 * @LastEditTime: 2021-11-11 18:13:10
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\system\cuda_dem_system.cuh
 */
#ifndef _CUDA_DEM_NS_SYSTEM_CUH_
#define _CUDA_DEM_NS_SYSTEM_CUH_

#pragma once

#include <kiri_pbs_cuda/system/cuda_base_system.cuh>

#include <kiri_pbs_cuda/particle/cuda_boundary_particles.cuh>
#include <kiri_pbs_cuda/searcher/cuda_neighbor_searcher.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_non_spherical_solver.cuh>

namespace KIRI {
class CudaDemNSSystem : public CudaBaseSystem {
public:
  explicit CudaDemNSSystem(CudaNonSphericalParticlesPtr &sandParticles,
                           CudaBoundaryParticlesPtr &boundaryParticles,
                           CudaNonSphericalSolverPtr &solver,
                           CudaGNSearcherPtr &searcher,
                           CudaGNBoundarySearcherPtr &boundarySearcher);

  CudaDemNSSystem(const CudaDemNSSystem &) = delete;
  CudaDemNSSystem &operator=(const CudaDemNSSystem &) = delete;
  virtual ~CudaDemNSSystem() noexcept {}

  inline int GetNumofSandSize() const { return (*mSands).Size(); }
  inline CudaNonSphericalParticlesPtr GetParticles() const { return mSands; }

protected:
  virtual void OnUpdateSolver(float timeIntervalInSeconds) override;

private:
  const int mCudaGridSize;
  CudaNonSphericalParticlesPtr mSands;
  CudaGNSearcherPtr mSearcher;
};

typedef SharedPtr<CudaDemNSSystem> CudaDemNSSystemPtr;
} // namespace KIRI

#endif