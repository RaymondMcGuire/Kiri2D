/***
 * @Author: Xu.WANG
 * @Date: 2021-11-10 21:59:44
 * @LastEditTime: 2021-11-11 17:53:25
 * @LastEditors: Xu.WANG
 * @Description:
 */

#include <kiri_pbs_cuda/data/cuda_ns_pack.h>
namespace KIRI
{

    NSPack::NSPack() {}
    NSPack::NSPack(ns_pack_type type, float radius)
    {
        mType = type;
        GenDefaultTypeUniRadius(radius);
    }

    void NSPack::AppendSubParticles(const float2 center, float radius)
    {
        mPack.push_back(ns_sphere_data(center, radius));
    }

    void NSPack::UpdateDefaultTypeRadius(float radius)
    {
        GenDefaultTypeUniRadius(radius);
    }

    void NSPack::SetColor(float3 color)
    {
        for (auto &s : mPack)
        {
            s.color = color;
        }
    }

    void NSPack::Translate(const float2 shift)
    {
        for (auto &s : mPack)
        {
            s.center += shift;
        }
    }

    void NSPack::RotateAroundOrigin(const float angle)
    {
        for (auto &s : mPack)
        {
            auto rot = make_rotation2(angle);
            s.rot = rot;
            s.center = rot.mat * s.center;
        }
    }

    void NSPack::AABB(float2 &min, float2 &max) const
    {
        float inf = std::numeric_limits<float>::infinity();
        min = make_float2(inf, inf);
        max = make_float2(-inf, -inf);
        for (const auto &s : mPack)
        {
            float2 r = make_float2(s.radius);
            min = fminf(s.center - r, min);
            max = fmaxf(s.center + r, max);
        }
    }

    void NSPack::GenDefaultTypeUniRadius(float radius)
    {
        mPack.clear();
        switch (mType)
        {
        case MSM_S1:
            AppendSubParticles(make_float2(0.f), radius);
            break;
        case MSM_L2:
            AppendSubParticles(make_float2(0.f, radius / 2.f), radius);
            AppendSubParticles(make_float2(0.f, -radius / 2.f), radius);
            break;
        case MSM_L3:
            AppendSubParticles(make_float2(0.f), radius);
            AppendSubParticles(make_float2(0.f, radius), radius);
            AppendSubParticles(make_float2(0.f, -radius), radius);
            break;
        default:
            break;
        }
    }
}