/***
 * @Author: Xu.WANG
 * @Date: 2021-12-08 19:59:56
 * @LastEditTime: 2021-12-08 20:01:35
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_VORONOI_REGION_H_
#define _HDV_VORONOI_REGION_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_edge.h>

namespace HDV::Voronoi
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr>
    class VoronoiRegion
    {
    public:
        explicit VoronoiRegion() {}
        virtual ~VoronoiRegion() noexcept {}

        int Id;
        std::vector<std::shared_ptr<HDV::Delaunay::DelaunayCell>> Cells;
        std::vector<std::shared_ptr<VoronoiEdge>> Edges;
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_REGION_H_ */