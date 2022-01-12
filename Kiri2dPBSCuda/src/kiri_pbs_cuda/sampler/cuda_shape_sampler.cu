/*
 * @Author: Xu.WANG
 * @Date: 2021-02-25 01:18:58
 * @LastEditTime: 2022-01-12 15:10:46
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri\KiriPBSCuda\src\kiri_pbs_cuda\sampler\cuda_shape_sampler.cu
 */

#include <kiri_pbs_cuda/sampler/cuda_shape_sampler.cuh>
#include <kiri_pbs_cuda/sampler/cuda_shape_sampler_gpu.cuh>
#include <random>

namespace KIRI {
CudaShapeSampler::CudaShapeSampler(const LevelSetShapeInfo &info,
                                   const Vec_Float3 &faceVertices,
                                   const int attemptNum,
                                   const int dim3BlockSize)
    : mInfo(info), mFaceVertices(faceVertices.size()),
      mSamplerTable(info.GridSize.x * info.GridSize.y * info.GridSize.z, true),
      mSampledGridInfo(info.GridSize.x * info.GridSize.y * info.GridSize.z),
      mCudaGridSize(CuCeilDiv(mInfo.NumOfFaces, KIRI_CUBLOCKSIZE)),
      mCudaDim3BlockSize(dim3BlockSize, dim3BlockSize, dim3BlockSize),
      mCudaDim3GridSize(CuCeilDiv(info.GridSize.x, dim3BlockSize),
                        CuCeilDiv(info.GridSize.y, dim3BlockSize),
                        CuCeilDiv(info.GridSize.z, dim3BlockSize))

{
  if (mFaceVertices.Length() != mInfo.NumOfFaces * 3) {
    printf("Loaded mesh data is not correct! \n");
    return;
  }

  // Transfer poly cpu data to gpu
  KIRI_CUCALL(cudaMemcpy(mFaceVertices.Data(), &faceVertices[0],
                         sizeof(float3) * faceVertices.size(),
                         cudaMemcpyHostToDevice));

  float cudaSamplingTime = this->ComputeSamplerTableMTSafe(attemptNum);
  printf("CUDA Shape Sampler MMT Construct Finished! Time=%.3f \n",
         cudaSamplingTime);
}

float CudaShapeSampler::ComputeSamplerTableMTSafe(int attempt) {

  curandState *devStates;
  dim3 d3Rnd(KIRI_RANDOM_SEEDS, 1, 1);
  KIRI_CUCALL(cudaMalloc(&devStates, KIRI_RANDOM_SEEDS * sizeof(curandState)));
  SetUpRndGen_CUDA<<<1, d3Rnd>>>(devStates, time(NULL));

  float elapsedTime;

  cudaEvent_t cudaTimerStart, cudaTimerEnd;
  KIRI_CUCALL(cudaEventCreate(&cudaTimerStart));
  KIRI_CUCALL(cudaEventCreate(&cudaTimerEnd));

  KIRI_CUCALL(cudaEventRecord(cudaTimerStart, 0));
  ShapeSamplerMMT_CUDA<<<mCudaDim3GridSize, mCudaDim3BlockSize>>>(
      mInfo, mFaceVertices.Data(), mSamplerTable.Data(), attempt, devStates);
  KIRI_CUCALL(cudaDeviceSynchronize());
  KIRI_CUKERNAL();

  KIRI_CUCALL(cudaEventRecord(cudaTimerEnd, 0));
  KIRI_CUCALL(cudaEventSynchronize(cudaTimerEnd));
  KIRI_CUCALL(cudaEventElapsedTime(&elapsedTime, cudaTimerStart, cudaTimerEnd));

  KIRI_CUCALL(cudaEventDestroy(cudaTimerStart));
  KIRI_CUCALL(cudaEventDestroy(cudaTimerEnd));

  KIRI_CUCALL(cudaFree(devStates));

  return elapsedTime;
}

bool CudaShapeSampler::CheckPointsInside(float3 p) {
  float3 relPos = p - mInfo.BBox.Min;
  int3 pos2SamplerTable =
      make_int3(static_cast<int>(relPos.x / mInfo.CellSize),
                static_cast<int>(relPos.y / mInfo.CellSize),
                static_cast<int>(relPos.z / mInfo.CellSize));

  if (pos2SamplerTable.x < 0 || pos2SamplerTable.y < 0 ||
      pos2SamplerTable.z < 0 || pos2SamplerTable.x >= mInfo.GridSize.x ||
      pos2SamplerTable.y >= mInfo.GridSize.y ||
      pos2SamplerTable.z >= mInfo.GridSize.z)
    return false;

  if (CheckSamplerTable(pos2SamplerTable.x, pos2SamplerTable.y,
                        pos2SamplerTable.z, mInfo.GridSize,
                        mSamplerTable.Data()))
    return true;

  return false;
}

Vec_Float3 CudaShapeSampler::GetInsidePoints(int num) {
  std::random_device seedGen;
  std::default_random_engine rndEngine(seedGen());
  std::uniform_real_distribution<float> dist(0.f, 1.f);

  int counter = 0;
  Vec_Float3 posArray;
  while (counter < num) {
    float3 pos = mInfo.BBox.Min + make_float3(dist(rndEngine), dist(rndEngine),
                                              dist(rndEngine)) *
                                      (mInfo.BBox.Max - mInfo.BBox.Min);

    if (this->CheckPointsInside(pos)) {
      posArray.emplace_back(pos);
      counter++;
    }
  }
  return posArray;
}
} // namespace KIRI