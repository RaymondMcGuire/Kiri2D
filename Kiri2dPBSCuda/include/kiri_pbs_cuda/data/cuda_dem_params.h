/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-10 15:29:35
 * @LastEditTime: 2021-09-13 16:18:08
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\data\cuda_dem_params.h
 */

#ifndef _CUDA_DEM_PARAMS_H_
#define _CUDA_DEM_PARAMS_H_

#pragma once

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace KIRI
{
    struct CudaDemParams
    {
        float rest_mass;
        float rest_density;
        float particle_radius;
        float kernel_radius;

        float young;
        float poisson;
        float tan_friction_angle;

        float c0;

        float2 gravity;
        float dt;
        float damping;
    };

    struct CudaDemAppParams
    {
        size_t max_num = 100000;

        bool run = false;
        bool run_offline = false;

        int scene_data_idx = 0;
        char bgeo_export_folder[320] = "default";
        bool bgeo_export = true;
    };

    extern CudaDemParams CUDA_DEM_PARAMS;
    extern CudaDemAppParams CUDA_DEM_APP_PARAMS;

} // namespace KIRI

#endif