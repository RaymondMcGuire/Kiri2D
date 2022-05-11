/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 22:59:48
 * @LastEditTime: 2021-11-11 18:14:17
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\system\cuda_dem_system.cu
 */

#include <kiri_pbs_cuda/system/cuda_base_system_gpu.cuh>
#include <kiri_pbs_cuda/system/cuda_dem_ns_system.cuh>
#include <kiri_pbs_cuda/thrust_helper/helper_thrust.cuh>

namespace KIRI2D {

CudaDemNSSystem::CudaDemNSSystem(CudaNonSphericalParticlesPtr &sandParticles,
                                 CudaBoundaryParticlesPtr &boundaryParticles,
                                 CudaNonSphericalSolverPtr &solver,
                                 CudaGNSearcherPtr &searcher,
                                 CudaGNBoundarySearcherPtr &boundarySearcher)
    : CudaBaseSystem(boundaryParticles,
                     std::static_pointer_cast<CudaBaseSolver>(solver),
                     boundarySearcher, sandParticles->Size(), false),
      mSands(std::move(sandParticles)), mSearcher(std::move(searcher)),
      mCudaGridSize(CuCeilDiv(sandParticles->Size(), KIRI_CUBLOCKSIZE)) {}

void CudaDemNSSystem::OnUpdateSolver(float renderInterval) {

  mSearcher->BuildGNSearcher(mSands);

  auto solver = std::dynamic_pointer_cast<CudaNonSphericalSolver>(mSolver);
  solver->UpdateSolver(mSands, mSearcher->GetCellStart(), renderInterval,
                       CUDA_DEM_NS_PARAMS, CUDA_BOUNDARY_PARAMS);
  cudaDeviceSynchronize();
  KIRI_CUKERNAL();
}
} // namespace KIRI2D
