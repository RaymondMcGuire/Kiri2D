/***
 * @Author: Xu.WANG
 * @Date: 2021-12-08 19:55:11
 * @LastEditTime: 2021-12-08 19:58:38
 * @LastEditors: Xu.WANG
 * @Description:
 */
#ifndef _HDV_VORONOI_EDGE_H_
#define _HDV_VORONOI_EDGE_H_

#pragma once

#include <kiri2d/hdv_toolkit/delaunay/delaunay_cell.h>

namespace HDV::Voronoi
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiEdge
    {
    public:
        explicit VoronoiEdge() {}
        explicit VoronoiEdge(const std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> &from, const std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> &to)
        {
            From = from;
            To = to;
        }
        virtual ~VoronoiEdge() noexcept {}

        std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> From;
        std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> To;
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_EDGE_H_ */