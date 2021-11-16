/***
 * @Author: Xu.WANG
 * @Date: 2021-11-11 17:08:00
 * @LastEditTime: 2021-11-16 16:22:08
 * @LastEditors: Xu.WANG
 * @Description:
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
            cellStart,
            bparams.lowest_point,
            bparams.highest_point,
            bparams.kernel_radius,
            bparams.grid_size);

        ComputeNSMomentum(sands, params);
        NonSphericalParticlesTimeIntegration(sands, params);
    }

} // namespace KIRI