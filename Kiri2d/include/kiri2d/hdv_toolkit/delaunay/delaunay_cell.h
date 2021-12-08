/***
 * @Author: Xu.WANG
 * @Date: 2021-12-08 23:18:30
 * @LastEditTime: 2021-12-08 23:21:55
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_DELAUNAY_CELL_H_
#define _HDV_DELAUNAY_CELL_H_

#pragma once

#include <kiri2d/hdv_toolkit/primitives/simplex.h>

namespace HDV::Delaunay
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class DelaunayCell
    {
    public:
        explicit DelaunayCell() {}
        explicit DelaunayCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> &simplex, std::vector<float> circumCenter, float radius)
        {
            mSimplex = simplex;

            CircumCenter = std::make_shared<VERTEX>();
            CircumCenter->mPosition = circumCenter;

            Radius = radius;
        }
        virtual ~DelaunayCell() noexcept {}

        std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> mSimplex;

        VERTEXPTR CircumCenter;

        float Radius;
    };

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_CELL_H_ */