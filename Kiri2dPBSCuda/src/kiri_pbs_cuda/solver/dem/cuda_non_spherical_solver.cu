
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 17:49:11
 * @LastEditTime: 2021-11-08 01:19:16
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_non_spherical_solver.cu
 */

#include <kiri_pbs_cuda/solver/cuda_solver_common_gpu.cuh>
#include <kiri_pbs_cuda/solver/cuda_solver_utils.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver_gpu.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_non_spherical_solver.cuh>
#include <kiri_pbs_cuda/thrust_helper/helper_thrust.cuh>

namespace KIRI {
void CudaNonSphericalSolver::Advect(CudaNonSphericalParticlesPtr &sands,
                                    const float dt, const float damping) {
  size_t num = sands->Size();
  sands->Advect(dt, damping);

  thrust::fill(thrust::device, sands->GetAccPtr(), sands->GetAccPtr() + num,
               make_float2(0.f));
  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();
}

void CudaNonSphericalSolver::ExtraForces(CudaNonSphericalParticlesPtr &sands,
                                         const float2 gravity) {

  thrust::transform(thrust::device, sands->GetAccPtr(),
                    sands->GetAccPtr() + sands->Size(), sands->GetAccPtr(),
                    ThrustHelper::Plus<float2>(gravity));

  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();
}

} // namespace KIRI
