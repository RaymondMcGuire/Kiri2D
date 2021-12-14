/***
 * @Author: Xu.WANG
 * @Date: 2021-12-14 10:22:02
 * @LastEditTime: 2021-12-14 10:22:21
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_VORONOI_CELL_POLYGON_H_
#define _HDV_VORONOI_CELL_POLYGON_H_

#pragma once

#include <kiri2d/hdv_toolkit/delaunay/delaunay_cell.h>

namespace HDV::Voronoi
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiCellPolygon
    {
    public:
        explicit VoronoiCellPolygon() {}
        virtual ~VoronoiCellPolygon() noexcept {}

        void AddVert2(Vector2F vert)
        {
            Verts.emplace_back(vert);
            BBox.merge(vert);
        }

        BoundingBox2F BBox;
        std::vector<Vector2F> Verts;
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_CELL_POLYGON_H_ */