/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 14:33:32
 * @LastEditTime: 2021-09-11 21:54:09
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\particle\cuda_dem_particles.cu
 */

#include <kiri_pbs_cuda/particle/cuda_dem_particles.cuh>
namespace KIRI2D
{
    void CudaDemParticles::Advect(const float dt, const float damping)
    {
        thrust::transform(thrust::device,
                          mAcc.Data(), mAcc.Data() + Size(),
                          mVel.Data(),
                          mAcc.Data(),
                          [dt, damping] __host__ __device__(const float2 &a, const float2 &lv) {
                              return a * (make_float2(1.f) - damping * sgn(a * (lv + 0.5f * dt * a)));
                          });

        thrust::transform(thrust::device,
                          mVel.Data(), mVel.Data() + Size(),
                          mAcc.Data(),
                          mVel.Data(),
                          [dt] __host__ __device__(const float2 &lv, const float2 &a) {
                              return lv + dt * a;
                          });

        thrust::transform(thrust::device,
                          mPos.Data(), mPos.Data() + Size(),
                          mVel.Data(),
                          mPos.Data(),
                          [dt] __host__ __device__(const float2 &lp, const float2 &v) {
                              return lp + dt * v;
                          });

        KIRI_CUCALL(cudaDeviceSynchronize());
        KIRI_CUKERNAL();
    }
  

} // namespace KIRI2D
