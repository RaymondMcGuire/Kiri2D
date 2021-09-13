
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-03 17:49:11
 * @LastEditTime: 2021-09-13 16:18:45
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\solver\dem\cuda_dem_solver.cu
 */

#include <kiri_pbs_cuda/thrust_helper/helper_thrust.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver.cuh>
#include <kiri_pbs_cuda/solver/dem/cuda_dem_solver_gpu.cuh>
#include <kiri_pbs_cuda/solver/cuda_solver_common_gpu.cuh>

namespace KIRI
{
  struct LinearAttenCoeff
    {
        float c0, csat;
        __host__ __device__ LinearAttenCoeff(
            const float c0,
            const float csat)
            : c0(c0),
              csat(csat)
        {
        }

        __device__ float operator()(const float sr)
        {
            return csat + (1.f - sr) * (c0 - csat);
        }
    };

    struct QuadraticBezierCoeff
    {
        float a, b, c, d, e;
        float py0, py1, px1, py2;
        __host__ __device__ QuadraticBezierCoeff(
            const float py0,
            const float py1,
            const float px1,
            const float py2)
            : py0(py0),
              py1(py1),
              px1(px1),
              py2(py2)
        {
            a = (px1 + 1.f) / 4.f;
            b = -2.f * a;
            c = b * b;
            d = -4.f * (1.f + b - px1);
            e = 2.f * (1.f + b - px1);
        }

        __device__ float rx2t(const float sr)
        {
            return (b + std::sqrt(c + d * (px1 - sr))) / e;
        }

        __device__ float operator()(const float sr)
        {
            if (sr < 0.f)
                return py0;

            if (sr >= 1.f)
                return py2;

            if (sr <= px1)
            {
                const float t = sr / px1;
                const float omt = 1.f - t;
                return omt * omt * py0 + 2 * t * omt * py1 + t * t * py1;
            }
            else
            {
                const float t = rx2t(sr);
                const float omt = 1.f - t;
                return omt * omt * py1 + 2 * t * omt * py1 + t * t * py2;
            }
        }
    };
    
  void CudaDemSolver::Advect(
      CudaDemParticlesPtr &sands,
      const float dt,
      const float damping)
  {
    size_t num = sands->Size();
    sands->Advect(dt, damping);

    thrust::fill(thrust::device, sands->GetAccPtr(), sands->GetAccPtr() + num, make_float2(0.f));
    KIRI_CUCALL(cudaDeviceSynchronize());
    KIRI_CUKERNAL();
  }

  void CudaDemSolver::ExtraForces(
      CudaDemParticlesPtr &sands,
      const float2 gravity)
  {

    thrust::transform(thrust::device,
                      sands->GetAccPtr(), sands->GetAccPtr() + sands->Size(),
                      sands->GetAccPtr(),
                      ThrustHelper::Plus<float2>(gravity));

    KIRI_CUCALL(cudaDeviceSynchronize());
    KIRI_CUKERNAL();
  }

  void CudaDemSolver::ComputeDemLinearMomentum(
      CudaDemParticlesPtr &sands,
      const float radius,
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
    ComputeDemLinearMomentum_CUDA<<<mCudaGridSize, KIRI_CUBLOCKSIZE>>>(
        sands->GetPosPtr(),
        sands->GetVelPtr(),
        sands->GetAccPtr(),
        sands->GetMassPtr(),
        radius,
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
