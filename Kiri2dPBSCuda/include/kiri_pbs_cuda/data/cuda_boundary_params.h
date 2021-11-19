/***
 * @Author: Xu.WANG
 * @Date: 2021-11-15 12:33:58
 * @LastEditTime: 2021-11-19 16:38:31
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\data\cuda_boundary_params.h
 */

#ifndef _CUDA_BOUNDARY_PARAMS_CUH_
#define _CUDA_BOUNDARY_PARAMS_CUH_

#pragma once

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace KIRI
{
    struct CudaBoundaryParams
    {
        float min_radius;
        float kernel_radius;
        float2 lowest_point;
        float2 highest_point;
        float2 world_size;
        float2 world_center;
        int2 grid_size;
    };

    extern CudaBoundaryParams CUDA_BOUNDARY_PARAMS;

} // namespace KIRI

#endif