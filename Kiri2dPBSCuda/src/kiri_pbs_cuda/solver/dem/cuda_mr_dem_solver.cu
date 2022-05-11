
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 17:49:11
 * @LastEditTime: 2021-11-19 16:42:53
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_mr_dem_solver.cu
 * \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_mr_dem_solver.cu
 */
#include <kiri_pbs_cuda/solver/cuda_solver_utils.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_mr_dem_solver.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_mr_dem_solver_gpu.cuh>
#include <kiri_pbs_cuda/thrust_helper/helper_thrust.cuh>

namespace KIRI2D {
void CudaMRDemSolver::ComputeMRDemLinearMomentum(
    CudaMRDemParticlesPtr &sands, const float young, const float poisson,
    const float tanFrictionAngle, const float c0, const float dt,const float boundaryRadius,
    const CudaArray<size_t> &cellStart, const float2 lowestPoint,
    const float2 highestPoint, const float kernelRadius, const int2 gridSize) {
  ComputeMRDemLinearMomentum_CUDA<<<mCudaGridSize, KIRI_CUBLOCKSIZE>>>(
      sands->GetPosPtr(), sands->GetVelPtr(), sands->GetAccPtr(),
      sands->GetMassPtr(), sands->GetRadiusPtr(), boundaryRadius, young, poisson,
      tanFrictionAngle, 0.f, sands->Size(), lowestPoint, highestPoint, dt,
      cellStart.Data(), gridSize,
      ThrustHelper::Pos2GridXY(lowestPoint, kernelRadius, gridSize),
      ThrustHelper::GridXY2GridHash(gridSize), LinearAttenCoeff(c0, 0.f));
  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();
}

} // namespace KIRI2D
