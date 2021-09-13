/*
 * @Author: Xu.WANG
 * @Date: 2021-02-04 12:36:10
 * @LastEditTime: 2021-09-13 16:12:42
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\emitter\cuda_volume_emitter.cuh
 */

#ifndef _CUDA_VOLUME_EMITTER_CUH_
#define _CUDA_VOLUME_EMITTER_CUH_

#pragma once

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace KIRI
{

    struct SphVolumeData
    {
        Vec_Float2 pos;
        Vec_Float3 col;
    };

    struct DemVolumeData
    {
        Vec_Float2 pos;
        Vec_Float3 col;
        Vec_Float mass;
    };

    struct DemShapeVolumeData
    {
        float minRadius;
        Vec_Float2 pos;
        Vec_Float3 col;
        Vec_Float mass;
        Vec_Float radius;
    };

    class CudaVolumeEmitter
    {
    public:
        explicit CudaVolumeEmitter(
            bool enable = true)
            : bEnable(enable)
        {
        }

        CudaVolumeEmitter(const CudaVolumeEmitter &) = delete;
        CudaVolumeEmitter &operator=(const CudaVolumeEmitter &) = delete;
        virtual ~CudaVolumeEmitter() noexcept {}

        void BuildSphVolume(SphVolumeData &data, float2 lowest, int2 vsize, float particleRadius, float3 color);
        void BuildUniDemVolume(DemVolumeData &data, float2 lowest, int2 vsize, float particleRadius, float3 color, float mass, float jitter = 0.001f);
        void BuildDemUniShapeVolume(DemShapeVolumeData &data, Vec_Float3 shape, float3 color, float mass, float2 offset=make_float2(0.f));

        inline constexpr bool GetEmitterStatus() const { return bEnable; }

    private:
        bool bEnable;
    };

    typedef SharedPtr<CudaVolumeEmitter> CudaVolumeEmitterPtr;
} // namespace KIRI

#endif