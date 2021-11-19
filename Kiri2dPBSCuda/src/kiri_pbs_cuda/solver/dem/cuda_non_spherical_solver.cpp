/***
 * @Author: Xu.WANG
 * @Date: 2021-11-17 11:15:51
 * @LastEditTime: 2021-11-19 16:45:09
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_non_spherical_solver.cpp
 */

#include <kiri_pbs_cuda/solver/dem/cuda_non_spherical_solver.cuh>

namespace KIRI
{
    void CudaNonSphericalSolver::UpdateSolver(
        CudaNonSphericalParticlesPtr &sands,
        const CudaArray<size_t> &cellStart,
        float renderInterval,
        const CudaDemNonSphericalParams params,
        CudaBoundaryParams bparams)
    {
        mNumOfSubTimeSteps = static_cast<int>(renderInterval / params.dt);

        ComputeNSDemLinearMomentum(
            sands,
            params.young,
            params.poisson,
            params.tan_friction_angle,
            params.dt,
            bparams.min_radius,
            cellStart,
            bparams.lowest_point,
            bparams.highest_point,
            bparams.kernel_radius,
            bparams.grid_size);

        ComputeNSMomentum(sands, params);
        NonSphericalParticlesTimeIntegration(sands, params,
                                             bparams.min_radius,
                                             bparams.lowest_point,
                                             bparams.highest_point);
    }

} // namespace KIRI