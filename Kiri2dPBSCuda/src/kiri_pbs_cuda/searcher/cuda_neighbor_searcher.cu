/*
 * @Author: Xu.WANG
 * @Date: 2021-02-05 12:33:37
 * @LastEditTime: 2021-11-09 16:10:06
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\searcher\cuda_neighbor_searcher.cu
 */

#include <kiri_pbs_cuda/searcher/cuda_neighbor_searcher.cuh>
#include <kiri_pbs_cuda/searcher/cuda_neighbor_searcher_gpu.cuh>
#include <kiri_pbs_cuda/thrust_helper/helper_thrust.cuh>

#include <kiri_pbs_cuda/particle/cuda_mr_dem_particles.cuh>
#include <kiri_pbs_cuda/particle/cuda_non_spherical_particles.cuh>

namespace KIRI {

CudaGNBaseSearcher::CudaGNBaseSearcher(const float2 lowestPoint,
                                       const float2 highestPoint,
                                       const size_t maxNumOfParticles,
                                       const float cellSize)
    : mLowestPoint(lowestPoint), mHighestPoint(highestPoint),
      mCellSize(cellSize),
      mGridSize(make_int2((highestPoint - lowestPoint) / cellSize)),
      mNumOfGridCells(mGridSize.x * mGridSize.y + 1),
      mCellStart(mNumOfGridCells), mMaxNumOfParticles(maxNumOfParticles),
      mGridIdxArray(max(mNumOfGridCells, maxNumOfParticles)),
      mCudaGridSize(CuCeilDiv(maxNumOfParticles, KIRI_CUBLOCKSIZE)) {
  // printf("cellsize=%.3f, gridsize=%d,%d; num grid=%d
  // \n",mCellSize,mGridSize.x,mGridSize.y,mNumOfGridCells);
}

void CudaGNBaseSearcher::BuildGNSearcher(const CudaParticlesPtr &particles) {
  thrust::transform(
      thrust::device, particles->GetPosPtr(),
      particles->GetPosPtr() + particles->Size(), mGridIdxArray.Data(),
      ThrustHelper::Pos2GridHash(mLowestPoint, mCellSize, mGridSize));

  this->SortData(particles);

  thrust::fill(thrust::device, mCellStart.Data(),
               mCellStart.Data() + mNumOfGridCells, 0);
  CountingInCell_CUDA<<<mCudaGridSize, KIRI_CUBLOCKSIZE>>>(
      mCellStart.Data(), mGridIdxArray.Data(), particles->Size());
  thrust::exclusive_scan(thrust::device, mCellStart.Data(),
                         mCellStart.Data() + mNumOfGridCells,
                         mCellStart.Data());

  cudaDeviceSynchronize();
  KIRI_CUKERNAL();
}

CudaGNSearcher::CudaGNSearcher(const float2 lp, const float2 hp,
                               const size_t num, const float cellSize,
                               const SearcherParticleType type)
    : CudaGNBaseSearcher(lp, hp, num, cellSize), mSearcherParticleType(type) {}

void CudaGNSearcher::SortData(const CudaParticlesPtr &particles) {

  auto particle_size = particles->Size();
  if (mSearcherParticleType == SearcherParticleType::DEM) {
    auto sands = std::dynamic_pointer_cast<CudaDemParticles>(particles);
    thrust::sort_by_key(
        thrust::device, mGridIdxArray.Data(),
        mGridIdxArray.Data() + particle_size,
        thrust::make_zip_iterator(thrust::make_tuple(
            sands->GetPosPtr(), sands->GetVelPtr(), sands->GetColPtr())));
  } else if (mSearcherParticleType == SearcherParticleType::MRDEM) {
    auto sands = std::dynamic_pointer_cast<CudaMRDemParticles>(particles);

    thrust::sort_by_key(thrust::device, mGridIdxArray.Data(),
                        mGridIdxArray.Data() + particle_size,
                        thrust::make_zip_iterator(thrust::make_tuple(
                            sands->GetPosPtr(), sands->GetVelPtr(),
                            sands->GetColPtr(), sands->GetRadiusPtr())));

  } else if (mSearcherParticleType == SearcherParticleType::NON_SPHERICAL) {
    auto sands =
        std::dynamic_pointer_cast<CudaNonSphericalParticles>(particles);
    thrust::sort_by_key(thrust::device, mGridIdxArray.Data(),
                        mGridIdxArray.Data() + particles->Size(),
                        thrust::make_zip_iterator(thrust::make_tuple(
                            sands->GetIdPtr(), sands->GetPosPtr(),
                            sands->GetVelPtr(), sands->GetColPtr(),
                            sands->GetMassPtr(), sands->GetNSMappingPtr())));
  }
  cudaDeviceSynchronize();
  KIRI_CUKERNAL();
}

CudaGNBoundarySearcher::CudaGNBoundarySearcher(const float2 lp, const float2 hp,
                                               const size_t num,
                                               const float cellSize)
    : CudaGNBaseSearcher(lp, hp, num, cellSize) {}

void CudaGNBoundarySearcher::SortData(const CudaParticlesPtr &particles) {
  auto boundaries = std::dynamic_pointer_cast<CudaBoundaryParticles>(particles);
  thrust::sort_by_key(thrust::device, mGridIdxArray.Data(),
                      mGridIdxArray.Data() + particles->Size(),
                      thrust::make_zip_iterator(thrust::make_tuple(
                          boundaries->GetPosPtr(), boundaries->GetLabelPtr())));

  cudaDeviceSynchronize();
  KIRI_CUKERNAL();
}

} // namespace KIRI
