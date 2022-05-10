/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2022-05-10 11:15:40
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
        explicit VoronoiEdge()
        {
        }

        explicit VoronoiEdge(
            const std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> &from,
            const std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> &to)
        {
            mFrom = from;
            mTo = to;
        }

        virtual ~VoronoiEdge()
        {
        }

        void clear()
        {
            mFrom->clear();
            mTo->clear();
            mFrom = nullptr;
            mTo = nullptr;
        }

        std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> &from()
        {
            return mFrom;
        }

        std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> &to()
        {
            return mTo;
        }

    private:
        std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> mFrom;
        std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>> mTo;
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_EDGE_H_ */