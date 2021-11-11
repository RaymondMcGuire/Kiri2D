
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 17:49:11
 * @LastEditTime: 2021-11-11 19:28:01
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_non_spherical_solver.cu
 */

#include <kiri_pbs_cuda/solver/cuda_solver_common_gpu.cuh>
#include <kiri_pbs_cuda/solver/cuda_solver_utils.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_non_spherical_solver.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_non_spherical_solver_gpu.cuh>
#include <kiri_pbs_cuda/thrust_helper/helper_thrust.cuh>

namespace KIRI {

void CudaNonSphericalSolver::ComputeNSDemLinearMomentum(
    CudaNonSphericalParticlesPtr &sands, const float young, const float poisson,
    const float tanFrictionAngle, const float dt,
    const CudaArray<size_t> &cellStart, const float2 lowestPoint,
    const float2 highestPoint, const float kernelRadius, const int2 gridSize) {

  ComputeNSDemLinearMomentum_CUDA<<<mCudaGridSize, KIRI_CUBLOCKSIZE>>>(
      sands->GetNSParticlesPtr(), sands->GetNSMappingPtr(), sands->GetPosPtr(),
      sands->GetVelPtr(), sands->GetRadiusPtr(), young, poisson,
      tanFrictionAngle, sands->Size(), lowestPoint, highestPoint, dt,
      cellStart.Data(), gridSize,
      ThrustHelper::Pos2GridXY(lowestPoint, kernelRadius, gridSize),
      ThrustHelper::GridXY2GridHash(gridSize));
  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();
}

void CudaNonSphericalSolver::ComputeNSMomentum(
    CudaNonSphericalParticlesPtr &sands,
    const CudaDemNonSphericalParams params) {

  ComputeNSMomentum_CUDA<<<mCudaGridGroupSize, 8>>>(sands->GetNSParticlesPtr(),
                                                    params);

  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();
}

void CudaNonSphericalSolver::NonSphericalParticlesTimeIntegration(
    CudaNonSphericalParticlesPtr &sands,
    const CudaDemNonSphericalParams params) {

  NSTimeIntegration_CUDA<<<mCudaGridSize, KIRI_CUBLOCKSIZE>>>(
      sands->GetPosPtr(), sands->GetVelPtr(), sands->GetNSParticlesPtr(),
      sands->GetNSMappingPtr(), sands->Size(), params);

  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();
}

} // namespace KIRI
