/***
 * @Author: Xu.WANG
 * @Date: 2021-11-10 22:00:18
 * @LastEditTime: 2021-11-11 03:17:23
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _CUDA_NS_PACK_H_
#define _CUDA_NS_PACK_H_

#include <kiri_pbs_cuda/data/cuda_dem_params.h>
namespace KIRI
{
    class NSPack
    {
    public:
        NSPack();
        NSPack(ns_pack_type type, float radius);

        void AppendSubParticles(const float2 center, float radius);

        void UpdateDefaultTypeRadius(float radius);

        void SetColor(float3 color);

        void Translate(const float2 shift);

        void RotateAroundOrigin(const float rot);

        void AABB(float2 &min, float2 &max) const;

        float2 LowerPoint() const
        {
            float2 min, max;
            AABB(min, max);
            return min;
        }

        float2 UpperPoint() const
        {
            float2 min, max;
            AABB(min, max);
            return max;
        }

        float2 MidPoint() const
        {
            float2 min, max;
            AABB(min, max);
            return .5f * (min + max);
        }

        int Size() const
        {
            return mPack.size();
        }

        thrust::host_vector<ns_sphere_data> GetPack() { return mPack; }

    private:
        ns_pack_type mType;
        thrust::host_vector<ns_sphere_data> mPack;

        void GenDefaultTypeUniRadius(float radius);
    };

    typedef std::shared_ptr<NSPack> NSPackPtr;
} // namespace KIRI

#endif /* _CUDA_NS_PACK_H_ */