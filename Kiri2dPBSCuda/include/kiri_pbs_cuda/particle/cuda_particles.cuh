/*
 * @Author: Xu.WANG
 * @Date: 2021-02-04 12:36:10
 * @LastEditTime: 2021-09-03 07:26:46
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\KiriPBSCuda2D\include\kiri_pbs_cuda\particle\cuda_particles.cuh
 */

#ifndef _CUDA_PARTICLES_CUH_
#define _CUDA_PARTICLES_CUH_

#pragma once

#include <kiri_pbs_cuda/data/cuda_array.cuh>

namespace KIRI
{
    class CudaParticles
    {
    public:
        explicit CudaParticles(const size_t numOfMaxParticles)
            : mPos(numOfMaxParticles),
              mNumOfParticles(numOfMaxParticles),
              mNumOfMaxParticles(numOfMaxParticles)
        {
        }

        explicit CudaParticles(const Vec_Float2 &p)
            : mPos(p.size()),
              mNumOfParticles(p.size()),
              mNumOfMaxParticles(p.size())
        {
            KIRI_CUCALL(cudaMemcpy(mPos.Data(), &p[0], sizeof(float2) * p.size(), cudaMemcpyHostToDevice));
        }

        explicit CudaParticles(
            const size_t numOfMaxParticles,
            const Vec_Float2 &p)
            : mPos(numOfMaxParticles),
              mNumOfParticles(p.size()),
              mNumOfMaxParticles(numOfMaxParticles)
        {
            KIRI_CUCALL(cudaMemcpy(mPos.Data(), &p[0], sizeof(float2) * p.size(), cudaMemcpyHostToDevice));
        }

        CudaParticles(const CudaParticles &) = delete;
        CudaParticles &operator=(const CudaParticles &) = delete;

        virtual ~CudaParticles() noexcept {}

        inline size_t Size() const { return mNumOfParticles; }
        inline size_t MaxSize() const { return mNumOfMaxParticles; }
        inline float2 *GetPosPtr() const { return mPos.Data(); }

    protected:
        size_t mNumOfParticles;
        size_t mNumOfMaxParticles;
        CudaArray<float2> mPos;
    };

    typedef SharedPtr<CudaParticles> CudaParticlesPtr;
} // namespace KIRI

#endif