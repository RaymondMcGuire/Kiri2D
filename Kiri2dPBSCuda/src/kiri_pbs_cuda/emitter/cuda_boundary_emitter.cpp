/***
 * @Author: Xu.WANG
 * @Date: 2021-03-19 22:04:26
 * @LastEditTime: 2021-09-03 13:23:32
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\emitter\cuda_boundary_emitter.cpp
 */
#include <kiri_pbs_cuda/emitter/cuda_boundary_emitter.cuh>
namespace KIRI2D
{
    void CudaBoundaryEmitter::BuildWorldBoundary(BoundaryData &data, const float2 &lowest, const float2 &highest, const float particleRadius)
    {
        if (!bEnable)
            return;

        size_t epsilon = 0;
        float spacing = particleRadius * 2.f;
        float2 sides = (highest - lowest) / spacing;

        for (size_t i = -epsilon; i <= sides.x + epsilon; ++i)
        {
            data.pos.emplace_back(make_float2(lowest.x + i * spacing, lowest.y));
            data.label.emplace_back(0);
        }

        for (size_t i = -epsilon; i <= sides.y + epsilon; ++i)
        {
            data.pos.emplace_back(make_float2(lowest.x, lowest.y + i * spacing));
            data.label.emplace_back(0);
        }

        for (size_t i = -epsilon; i <= sides.x + epsilon; ++i)
        {
            data.pos.emplace_back(make_float2(lowest.x + i * spacing, highest.y));
            data.label.emplace_back(0);
        }

        for (size_t i = -epsilon; i <= sides.y + epsilon; ++i)
        {
            data.pos.emplace_back(make_float2(highest.x, lowest.y + i * spacing));
            data.label.emplace_back(0);
        }
    }

    void CudaBoundaryEmitter::BuildBoundaryShapeVolume(BoundaryData &data, Vec_Float2 shape)
    {
        if (!bEnable)
            return;

        for (size_t i = 0; i < shape.size(); i++)
        {
            data.pos.emplace_back(make_float2(shape[i].x, shape[i].y));
            data.label.emplace_back(1);
        }
    }

} // namespace KIRI2D
