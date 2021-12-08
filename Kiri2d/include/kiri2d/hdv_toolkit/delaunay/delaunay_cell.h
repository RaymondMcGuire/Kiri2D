/***
 * @Author: Xu.WANG
 * @Date: 2021-12-08 14:43:08
 * @LastEditTime: 2021-12-08 14:44:13
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_DELAUNAY_CELL_H_
#define _HDV_DELAUNAY_CELL_H_

#pragma once

#include <kiri2d/hdv_toolkit/primitives/simplex.h>

namespace HDV::Delaunay
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class DelaunayCell
    {
    public:
        explicit DelaunayCell() {}
        explicit DelaunayCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEX>> &simplex, std::vector<float> circumCenter, float radius)
        {
            mSimplex = simplex;

            CircumCenter = circumCenter;

            Radius = radius;
        }
        virtual ~DelaunayCell() noexcept {}

        std::shared_ptr<HDV::Primitives::Simplex<VERTEX>> mSimplex;

        std::vector<float> CircumCenter;

        float Radius;
    };

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_CELL_H_ */