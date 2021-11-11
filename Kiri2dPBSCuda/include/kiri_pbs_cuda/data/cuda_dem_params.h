/***
 * @Author: Xu.WANG
 * @Date: 2021-02-10 15:29:35
 * @LastEditTime: 2021-11-10 22:15:13
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

    enum ns_pack_type
    {
        MSM_S1 = 1,
        MSM_L2 = 2,
        MSM_L3 = 3
    };

    struct ns_sphere_gen
    {
        int ns_id, min_id, max_id;
        float radius;
        float2 center;
    };

    struct ns_sphere_data
    {
        int ns_id;
        float radius;
        float2 center;
        float3 color;
        rotation2 rot;
        ns_sphere_data(const float2 &_c, float _r, float3 _color = make_float3(0.f), int _id = -1)
            : center(_c), radius(_r), color(_color), ns_id(_id){};
    };

    struct CudaDemNonSphericalParams
    {
        float rest_mass;
        float rest_density;
        float particle_radius;
        float kernel_radius;

        float young;
        float poisson;
        float tan_friction_angle;

        float2 gravity;
        float dt;
        float damping;

        uint group_num;
    };

    struct non_spherical_particles
    {
        uint ns_id;
        float2 vel;
        float mass;
        float inertia;
        float2 centroid;
        float angle_vel;
        float angle_acc;
        matrix2x2 rot;

        int sub_num;
        float2 force_list[MAX_SUB_SPHERICAL_NUM];
        float torque_list[MAX_SUB_SPHERICAL_NUM];

        float3 sub_color_list[MAX_SUB_SPHERICAL_NUM];
        float sub_radius_list[MAX_SUB_SPHERICAL_NUM];
        float2 sub_pos_list[MAX_SUB_SPHERICAL_NUM];
        matrix2x2 sub_rot_list[MAX_SUB_SPHERICAL_NUM];
    };

    struct ns_mapping
    {
        uint ns_id;
        int sub_id;
        float2 rel_pos;
        matrix2x2 rel_rot;
    };

    extern CudaDemParams CUDA_DEM_PARAMS;
    extern CudaDemAppParams CUDA_DEM_APP_PARAMS;

    extern CudaDemNonSphericalParams CUDA_DEM_NS_PARAMS;

} // namespace KIRI

#endif