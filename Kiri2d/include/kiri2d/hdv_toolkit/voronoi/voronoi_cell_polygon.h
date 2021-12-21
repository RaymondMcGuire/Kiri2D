/***
 * @Author: Xu.WANG
 * @Date: 2021-12-21 21:10:15
 * @LastEditTime: 2021-12-21 21:18:39
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

        Vector2F GetCentroid()
        {
            std::vector<Vector2F> tmpPolygonVertices(Verts);

            auto first = tmpPolygonVertices[0], last = tmpPolygonVertices[tmpPolygonVertices.size() - 1];
            if (first.x != last.x || first.y != last.y)
                tmpPolygonVertices.emplace_back(first);

            auto twicearea = 0.f,
                 x = 0.f, y = 0.f, f = 0.f;
            auto nPts = tmpPolygonVertices.size();
            Vector2F p1, p2;

            for (size_t i = 0, j = nPts - 1; i < nPts; j = i++)
            {
                p1 = tmpPolygonVertices[i];
                p2 = tmpPolygonVertices[j];
                f = p1.x * p2.y - p2.x * p1.y;
                twicearea += f;
                x += (p1.x + p2.x) * f;
                y += (p1.y + p2.y) * f;
            }
            f = twicearea * 3.f;

            // KIRI_LOG_DEBUG("GetCentroid -------");
            // for (size_t i = 0; i < tmpPolygonVertices.size(); i++)
            // {
            //     KIRI_LOG_DEBUG("vert={0},{1}", tmpPolygonVertices[i].x, tmpPolygonVertices[i].y);
            // }
            // KIRI_LOG_DEBUG("GetCentroid -------,f={0}", f);

            return Vector2F(x / f, y / f);
        }

        BoundingBox2F BBox;
        std::vector<Vector2F> Verts;
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_CELL_POLYGON_H_ */