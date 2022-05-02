/*
 * @Author: Xu.WANG
 * @Date: 2021-02-04 12:36:10
 * @LastEditTime: 2022-05-02 13:33:20
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\emitter\cuda_boundary_emitter.cuh
 */

#ifndef _CUDA_BOUNDARY_EMITTER_CUH_
#define _CUDA_BOUNDARY_EMITTER_CUH_

#pragma once

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace KIRI
{
    struct BoundaryData
    {
        Vec_Float2 pos;
        Vec_SizeT label;
    };

    class CudaBoundaryEmitter
    {
    public:
        explicit CudaBoundaryEmitter(
            bool enable = true)
            : bEnable(enable)
        {
        }

        CudaBoundaryEmitter(const CudaBoundaryEmitter &) = delete;
        CudaBoundaryEmitter &operator=(const CudaBoundaryEmitter &) = delete;
        virtual ~CudaBoundaryEmitter()  {}

        void BuildWorldBoundary(BoundaryData &data, const float2 &lowest, const float2 &highest, const float particleRadius);
        void BuildBoundaryShapeVolume(BoundaryData &data, Vec_Float2 shape);

    private:
        bool bEnable;
    };

    typedef SharedPtr<CudaBoundaryEmitter> CudaBoundaryEmitterPtr;
} // namespace KIRI

#endif