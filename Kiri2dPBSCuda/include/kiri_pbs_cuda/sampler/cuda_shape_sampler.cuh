/*
 * @Author: Xu.WANG
 * @Date: 2021-02-25 01:18:29
 * @LastEditTime: 2022-01-05 23:24:45
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \Kiri\KiriPBSCuda\include\kiri_pbs_cuda\sampler\cuda_shape_sampler.cuh
 */

#ifndef _CUDA_SHAPE_SAMPLER_CUH_
#define _CUDA_SHAPE_SAMPLER_CUH_

#pragma once

#include <kiri_pbs_cuda/data/cuda_array.cuh>
#include <kiri_pbs_cuda/sampler/cuda_sampler_struct.cuh>

namespace KIRI {
class CudaShapeSampler {
public:
  explicit CudaShapeSampler(const LevelSetShapeInfo &info,
                            const Vec_Float3 &faceVertices,
                            const int dim3BlockSize = 8);

  virtual ~CudaShapeSampler() noexcept {}

  /**
   * @description: A safe shape sampling method which can deal with un-closed
   * mesh
   * @reference: https://github.com/kctess5/voxelizer
   * @param {int} attempt
   * @return {float} elapsed time
   */
  float ComputeSamplerTableMTSafe(int attempt);

  /**
   * @description: Compute sampled points positions inside model
   * @param {*}
   * @return {*}
   */
  Vec_Float3 GetInsidePoints(int num);

  inline const LevelSetShapeInfo &GetLevelSetInfo() { return mInfo; }
  size_t *GetSamplerTablePtr() const { return mSamplerTable.Data(); }

private:
  size_t mCudaGridSize;
  dim3 mCudaDim3BlockSize, mCudaDim3GridSize;

  LevelSetShapeInfo mInfo;
  CudaArray<float3> mFaceVertices;
  CudaArray<bool> mSampledGridInfo;
  CudaArray<size_t> mSamplerTable;
};

typedef SharedPtr<CudaShapeSampler> CudaShapeSamplerPtr;
} // namespace KIRI

#endif /* _CUDA_SHAPE_SAMPLER_CUH_ */