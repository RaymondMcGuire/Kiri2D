/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2022-05-03 18:04:02
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
        explicit DelaunayCell()
        {
        }

        explicit DelaunayCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> &simplex, std::vector<double> circumCenter, double radius)
        {
            mSimplex = simplex;

            CircumCenter = std::make_shared<VERTEX>();
            CircumCenter->positions() = circumCenter;

            Radius = radius;
        }

        virtual ~DelaunayCell()
        {
        }

        void clear()
        {
        }

        double Radius;
        VERTEXPTR CircumCenter;
        std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> mSimplex;
    };

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_CELL_H_ */