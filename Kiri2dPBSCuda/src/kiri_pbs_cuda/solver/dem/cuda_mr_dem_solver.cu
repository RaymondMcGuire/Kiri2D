
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 17:49:11
 * @LastEditTime: 2021-09-13 19:19:34
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_mr_dem_solver.cu
 */
#include <kiri_pbs_cuda/thrust_helper/helper_thrust.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_mr_dem_solver.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_mr_dem_solver_gpu.cuh>
#include <kiri_pbs_cuda/solver/cuda_solver_utils.cuh>
namespace KIRI
{
  void CudaMRDemSolver::ComputeMRDemLinearMomentum(
      CudaMRDemParticlesPtr &sands,
      const float young,
      const float poisson,
      const float tanFrictionAngle,
      const float c0,
      const CudaArray<size_t> &cellStart,
      const float2 lowestPoint,
      const float2 highestPoint,
      const float kernelRadius,
      const int2 gridSize)
  {
    ComputeMRDemLinearMomentum_CUDA<<<mCudaGridSize, KIRI_CUBLOCKSIZE>>>(
        sands->GetPosPtr(),
        sands->GetVelPtr(),
        sands->GetAccPtr(),
        sands->GetMassPtr(),
        sands->GetRadiusPtr(),
        young,
        poisson,
        tanFrictionAngle,
        0.f,
        sands->Size(),
        lowestPoint,
        highestPoint,
        cellStart.Data(),
        gridSize,
        ThrustHelper::Pos2GridXY(lowestPoint, kernelRadius, gridSize),
        ThrustHelper::GridXY2GridHash(gridSize),
        LinearAttenCoeff(c0, 0.f));
    KIRI_CUCALL(cudaDeviceSynchronize());
    KIRI_CUKERNAL();
  }

} // namespace KIRI
