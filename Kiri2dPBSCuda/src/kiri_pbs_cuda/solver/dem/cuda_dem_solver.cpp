/*** 
 * @Author: Xu.WANG
 * @Date: 2021-03-15 22:37:26
 * @LastEditTime: 2021-09-11 21:53:45
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_dem_solver.cpp
 */

#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver.cuh>

namespace KIRI
{
    void CudaDemSolver::UpdateSolver(
        CudaDemParticlesPtr &sands,
        const CudaArray<size_t> &cellStart,
        float renderInterval,
        CudaDemParams params,
        CudaBoundaryParams bparams)
    {
        mNumOfSubTimeSteps = static_cast<int>(renderInterval / params.dt);

        ExtraForces(
            sands,
            params.gravity);

        ComputeDemLinearMomentum(
            sands,
            params.particle_radius,
            params.young,
            params.poisson,
            params.tan_friction_angle,
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

} // namespace KIRI