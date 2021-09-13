/*** 
 * @Author: Xu.WANG
 * @Date: 2021-03-19 22:04:26
 * @LastEditTime: 2021-09-13 21:46:28
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\emitter\cuda_volume_emitter.cpp
 */
#include <random>
#include <kiri_pbs_cuda/emitter/cuda_volume_emitter.cuh>
namespace KIRI
{

    void CudaVolumeEmitter::BuildSphVolume(SphVolumeData &data, float2 lowest, int2 vsize, float particleRadius, float3 color)
    {
        if (!bEnable)
            return;

        float offset = 2.f * particleRadius;
        for (size_t i = 0; i < static_cast<size_t>(vsize.x); ++i)
        {
            for (size_t j = 0; j < static_cast<size_t>(vsize.y); ++j)
            {
                float2 p = make_float2(lowest.x + i * offset, lowest.y + j * offset);

                data.pos.emplace_back(p);
                data.col.emplace_back(color);
            }
        }
    }

    void CudaVolumeEmitter::BuildUniDemVolume(DemVolumeData &data, float2 lowest, int2 vsize, float particleRadius, float3 color, float mass, float jitter)
    {
        if (!bEnable)
            return;

        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<> dist(-1.f, 1.f);

        float offset = 2.f * particleRadius;
        for (size_t i = 0; i < static_cast<size_t>(vsize.x); ++i)
        {
            for (size_t j = 0; j < static_cast<size_t>(vsize.y); ++j)
            {
                float2 p = make_float2(lowest.x + i * offset, lowest.y + j * offset);

                data.pos.emplace_back(p + jitter * normalize(make_float2(dist(rndEngine), dist(rndEngine))));
                //data.pos.emplace_back(p);
                data.col.emplace_back(color);
                data.mass.emplace_back(mass);
            }
        }
    }

    void CudaVolumeEmitter::BuildDemUniShapeVolume(
        DemShapeVolumeData &data,
        Vec_Float3 shape,
        float3 color,
        float mass,
        float2 offset)
    {
        if (!bEnable)
            return;

        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<> dist(-1.f, 1.f);

        data.minRadius = Huge<size_t>();
        for (size_t i = 0; i < shape.size(); i++)
        {
            float radius = shape[i].z;
            auto jitter = 1e-6f * normalize(make_float2(dist(rndEngine), dist(rndEngine)));
            data.pos.emplace_back(make_float2(shape[i].x, shape[i].y) + offset + jitter);
            data.col.emplace_back(color);
            data.radius.emplace_back(radius);
            data.mass.emplace_back(mass);
            data.minRadius = std::min(radius, data.minRadius);
        }
    }

    void CudaVolumeEmitter::BuildMRDemShapeVolume(
        DemShapeVolumeData &data,
        float density,
        Vec_Float3 shape,
        float3 color,
        float2 offset)
    {
        if (!bEnable)
            return;

        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<> dist(-1.f, 1.f);

        data.minRadius = Huge<float>();
        data.maxRadius = Tiny<float>();
        for (size_t i = 0; i < shape.size(); i++)
        {
            auto radius = shape[i].z / 30.f;
            auto mass = density * std::powf(radius * 2.f, 2.f);

            data.pos.emplace_back(make_float2(shape[i].x, shape[i].y) / 30.f + offset);
            data.col.emplace_back(color);
            data.radius.emplace_back(radius);
            data.mass.emplace_back(mass);
            data.minRadius = std::min(radius, data.minRadius);
            data.maxRadius = std::max(radius, data.maxRadius);
        }
    }

} // namespace KIRI
