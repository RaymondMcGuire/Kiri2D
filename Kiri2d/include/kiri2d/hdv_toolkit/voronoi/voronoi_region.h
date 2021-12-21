/***
 * @Author: Xu.WANG
 * @Date: 2021-12-21 12:34:39
 * @LastEditTime: 2021-12-21 13:05:51
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\hdv_toolkit\voronoi\voronoi_region.h
 */

#ifndef _HDV_VORONOI_REGION_H_
#define _HDV_VORONOI_REGION_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_edge.h>

namespace HDV::Voronoi
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiRegion
    {
    public:
        explicit VoronoiRegion() {}
        virtual ~VoronoiRegion() noexcept {}

        int Id;
        std::vector<std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>> Cells;
        std::vector<std::shared_ptr<VoronoiEdge<VERTEXPTR, VERTEX>>> Edges;
        VERTEXPTR site;

        BoundingBox2F GetBBox()
        {
            BoundingBox2F bbox;
            for (size_t i = 0; i < Edges.size(); i++)
            {
                auto from = Edges[i]->From->CircumCenter;
                auto to = Edges[i]->To->CircumCenter;
                bbox.merge(Vector2F(from->X(), from->Y()));
                bbox.merge(Vector2F(to->X(), to->Y()));
            }
            return bbox;
        }
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_REGION_H_ */