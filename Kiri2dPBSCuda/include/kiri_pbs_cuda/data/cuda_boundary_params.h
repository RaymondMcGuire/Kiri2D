/*** 
 * @Author: Xu.WANG
 * @Date: 2021-09-03 07:00:28
 * @LastEditTime: 2021-09-03 09:17:55
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\data\cuda_boundary_params.h
 */
/*
 * @Author: Xu.WANG
 * @Date: 2021-02-11 01:26:00
 * @LastEditTime: 2021-08-18 16:36:52
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri\KiriPBSCuda\include\kiri_pbs_cuda\data\cuda_boundary_params.cuh
 */

#ifndef _CUDA_BOUNDARY_PARAMS_CUH_
#define _CUDA_BOUNDARY_PARAMS_CUH_

#pragma once

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace KIRI
{
    struct CudaBoundaryParams
    {
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