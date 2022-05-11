/***
 * @Author: Xu.WANG
 * @Date: 2021-11-15 12:33:59
 * @LastEditTime: 2021-11-19 16:45:17
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_mr_dem_solver.cpp
 */

#include <kiri_pbs_cuda/solver/dem/cuda_mr_dem_solver.cuh>

namespace KIRI2D
{
    void CudaMRDemSolver::UpdateSolver(
        CudaDemParticlesPtr &sands,
        const CudaArray<size_t> &cellStart,
        float renderInterval,
        CudaDemParams params,
        CudaBoundaryParams bparams)
    {

        mNumOfSubTimeSteps = static_cast<size_t>(renderInterval / params.dt);

        ExtraForces(
            sands,
            params.gravity);

        ComputeMRDemLinearMomentum(
            std::dynamic_pointer_cast<CudaMRDemParticles>(sands),
            params.young,
            params.poisson,
            params.tan_friction_angle,
            params.c0,
            params.dt,
            bparams.min_radius,
            cellStart,
            bparams.lowest_point,
            bparams.highest_point,
            bparams.kernel_radius,
            bparams.grid_size);

        Advect(
            sands,
            params.dt,
            params.damping);
    }

} // namespace KIRI2D