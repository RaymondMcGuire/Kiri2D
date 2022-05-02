/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 22:52:09
 * @LastEditTime: 2021-09-13 19:32:09
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\system\cuda_dem_system.cuh
 */
#ifndef _CUDA_DEM_SYSTEM_CUH_
#define _CUDA_DEM_SYSTEM_CUH_

#pragma once

#include <kiri_pbs_cuda/system/cuda_base_system.cuh>

#include <kiri_pbs_cuda/solver/dem/cuda_mr_dem_solver.cuh>
#include <kiri_pbs_cuda/particle/cuda_boundary_particles.cuh>
#include <kiri_pbs_cuda/searcher/cuda_neighbor_searcher.cuh>

namespace KIRI
{
    class CudaDemSystem : public CudaBaseSystem
    {
    public:
        explicit CudaDemSystem(
            CudaDemParticlesPtr &sandParticles,
            CudaBoundaryParticlesPtr &boundaryParticles,
            CudaDemSolverPtr &solver,
            CudaGNSearcherPtr &searcher,
            CudaGNBoundarySearcherPtr &boundarySearcher);

        CudaDemSystem(const CudaDemSystem &) = delete;
        CudaDemSystem &operator=(const CudaDemSystem &) = delete;
        virtual ~CudaDemSystem()  {}

        inline int GetNumofSandSize() const { return (*mSands).Size(); }
        inline CudaDemParticlesPtr GetParticles() const { return mSands; }
        
    protected:
        virtual void OnUpdateSolver(float timeIntervalInSeconds) override;

    private:
        const int mCudaGridSize;
        CudaDemParticlesPtr mSands;
        CudaGNSearcherPtr mSearcher;
    };

    typedef SharedPtr<CudaDemSystem> CudaDemSystemPtr;
} // namespace KIRI

#endif