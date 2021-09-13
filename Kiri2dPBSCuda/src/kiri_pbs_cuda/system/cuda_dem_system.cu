/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 22:59:48
 * @LastEditTime: 2021-09-03 09:24:29
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\system\cuda_dem_system.cu
 */

#include <kiri_pbs_cuda/thrust_helper/helper_thrust.cuh>
#include <kiri_pbs_cuda/system/cuda_dem_system.cuh>
#include <kiri_pbs_cuda/system/cuda_base_system_gpu.cuh>
namespace KIRI
{

    CudaDemSystem::CudaDemSystem(
        CudaDemParticlesPtr &sandParticles,
        CudaBoundaryParticlesPtr &boundaryParticles,
        CudaDemSolverPtr &solver,
        CudaGNSearcherPtr &searcher,
        CudaGNBoundarySearcherPtr &boundarySearcher)
        : CudaBaseSystem(
              boundaryParticles,
              std::static_pointer_cast<CudaBaseSolver>(solver),
              boundarySearcher,
              sandParticles->Size(),
              false),
          mSands(std::move(sandParticles)),
          mSearcher(std::move(searcher)),
          mCudaGridSize(CuCeilDiv(sandParticles->Size(), KIRI_CUBLOCKSIZE))
    {
    }

    void CudaDemSystem::OnUpdateSolver(float renderInterval)
    {

        mSearcher->BuildGNSearcher(mSands);
        
        auto solver = std::dynamic_pointer_cast<CudaDemSolver>(mSolver);
        solver->UpdateSolver(
            mSands,
            mSearcher->GetCellStart(),
            renderInterval,
            CUDA_DEM_PARAMS,
            CUDA_BOUNDARY_PARAMS);
        cudaDeviceSynchronize();
        KIRI_CUKERNAL();
    }
} // namespace KIRI
