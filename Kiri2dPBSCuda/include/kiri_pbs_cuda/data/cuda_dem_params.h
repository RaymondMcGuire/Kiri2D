/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-10 15:29:35
 * @LastEditTime: 2021-11-10 01:06:31
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
    #define MAX_SUB_SPHERICAL_NUM 10

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

    struct non_spherical_params
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

        uint group_num;
    };


    struct non_spherical_particles {

        float2 vel;
        float mass;
        float inertia;
        float2 centroid;
        float angle_vel;
        float angle_acc;
        rotation2 rot;

        int sub_num;
        float2 force_list[MAX_SUB_SPHERICAL_NUM];
        float torque_list[MAX_SUB_SPHERICAL_NUM];
        float radius_list[MAX_SUB_SPHERICAL_NUM];
        float2 pos_list[MAX_SUB_SPHERICAL_NUM];
        rotation2 ori_list[MAX_SUB_SPHERICAL_NUM];
    };

    struct ns_mapping {
        uint ns_id;
        int sub_id;
        float2 rel_pos;
        rotation2 rel_rot;
    };

    extern CudaDemParams CUDA_DEM_PARAMS;
    extern CudaDemAppParams CUDA_DEM_APP_PARAMS;

} // namespace KIRI

#endif