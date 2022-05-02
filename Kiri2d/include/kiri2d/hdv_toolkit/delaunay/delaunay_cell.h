/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2021-12-29 14:58:41
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
        explicit DelaunayCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> &simplex, std::vector<double> circumCenter, double radius)
        {
            mSimplex = simplex;

            CircumCenter = std::make_shared<VERTEX>();
            CircumCenter->mPosition = circumCenter;

            Radius = radius;
        }
        virtual ~DelaunayCell() {}

        void Clear()
        {
            // CircumCenter = nullptr;
            // mSimplex = nullptr;
        }

        std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> mSimplex;

        VERTEXPTR CircumCenter;

        double Radius;
    };

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_CELL_H_ */