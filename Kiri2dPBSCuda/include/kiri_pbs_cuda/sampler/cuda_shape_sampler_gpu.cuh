/*
 * @Author: Xu.WANG
 * @Date: 2021-02-25 01:18:29
 * @LastEditTime: 2022-01-05 22:30:17
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath:
 * \KiriPBSCuda\include\kiri_pbs_cuda\sampler\cuda_shape_sampler_gpu.cuh
 */

#ifndef _CUDA_SHAPE_SAMPLER_GPU_CUH_
#define _CUDA_SHAPE_SAMPLER_GPU_CUH_

#pragma once

#include <kiri_pbs_cuda/sampler/cuda_shape_sampler_common_gpu.cuh>

namespace KIRI2D {
__global__ void ShapeSamplerMMT_CUDA(LevelSetShapeInfo info,
                                     float3 *faceVertices, size_t *samplerTable,
                                     int attempts, curandState *state) {
  const size_t x = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  const size_t y = __umul24(blockIdx.y, blockDim.y) + threadIdx.y;
  const size_t z = __umul24(blockIdx.z, blockDim.z) + threadIdx.z;

  if ((x < info.GridSize.x) && (y < info.GridSize.y) && (z < info.GridSize.z)) {
    size_t gridIdx =
        x + y * info.GridSize.y + z * info.GridSize.y * info.GridSize.z;

    float3 pos = info.BBox.Min + make_float3(x, y, z) * info.CellSize;
    float3 dir;

    int votes = 0;
    float theta;
    float z;

    for (int j = 0; j < attempts; ++j) {

      theta = RndFloat(state, gridIdx % KIRI_RANDOM_SEEDS) * 2.f * KIRI_PI;
      z = RndFloat(state, gridIdx % KIRI_RANDOM_SEEDS) * 2.f - 1.f;

      dir.x = sqrtf(1.f - z * z) * cosf(theta);
      dir.y = sqrtf(1.f - z * z) * sinf(theta);
      dir.z = sqrtf(1.f - z * z) * cosf(theta);

      size_t intersections = 0;
      for (int i = 0; i < info.NumOfFaces; ++i)
        if (intersects(faceVertices[i * 3], faceVertices[i * 3 + 1],
                       faceVertices[i * 3 + 2], dir, pos))
          intersections += 1;

      if (intersections % 2 == 1)
        votes++;
    }

    if (votes > attempts / 2)
      setBitXor(samplerTable, gridIdx);
  }
}
} // namespace KIRI2D

#endif /* _CUDA_SHAPE_SAMPLER_GPU_CUH_ */