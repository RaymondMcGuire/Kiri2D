
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 17:49:11
 * @LastEditTime: 2021-11-19 16:42:39
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_dem_solver.cu
 * \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_dem_solver.cu
 */

#include <kiri_pbs_cuda/solver/cuda_solver_common_gpu.cuh>
#include <kiri_pbs_cuda/solver/cuda_solver_utils.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver_gpu.cuh>
#include <kiri_pbs_cuda/thrust_helper/helper_thrust.cuh>

namespace KIRI2D {
void CudaDemSolver::Advect(CudaDemParticlesPtr &sands, const float dt,
                           const float damping) {
  size_t num = sands->Size();
  sands->Advect(dt, damping);

  thrust::fill(thrust::device, sands->GetAccPtr(), sands->GetAccPtr() + num,
               make_float2(0.f));
  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();
}

void CudaDemSolver::ExtraForces(CudaDemParticlesPtr &sands,
                                const float2 gravity) {

  thrust::transform(thrust::device, sands->GetAccPtr(),
                    sands->GetAccPtr() + sands->Size(), sands->GetAccPtr(),
                    ThrustHelper::Plus<float2>(gravity));

  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();
}

void CudaDemSolver::ComputeDemLinearMomentum(
    CudaDemParticlesPtr &sands, const float radius, const float young,
    const float poisson, const float tanFrictionAngle, const float c0,
    const float dt,const float boundaryRadius, const CudaArray<size_t> &cellStart,
    const float2 lowestPoint, const float2 highestPoint,
    const float kernelRadius, const int2 gridSize) {
  ComputeDemLinearMomentum_CUDA<<<mCudaGridSize, KIRI_CUBLOCKSIZE>>>(
      sands->GetPosPtr(), sands->GetVelPtr(), sands->GetAccPtr(),
      sands->GetMassPtr(), radius,boundaryRadius, young, poisson, tanFrictionAngle, 0.f,
      sands->Size(), lowestPoint, highestPoint, dt, cellStart.Data(), gridSize,
      ThrustHelper::Pos2GridXY(lowestPoint, kernelRadius, gridSize),
      ThrustHelper::GridXY2GridHash(gridSize), LinearAttenCoeff(c0, 0.f));
  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();
}

} // namespace KIRI2D
