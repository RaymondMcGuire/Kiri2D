/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2021-12-29 15:01:19
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

        void AddVert2(Vector2D vert)
        {
            Verts.emplace_back(vert);
            BBox.merge(vert);
        }

        Vector2D GetCentroid()
        {
            std::vector<Vector2D> tmpPolygonVertices(Verts);

            auto first = tmpPolygonVertices[0], last = tmpPolygonVertices[tmpPolygonVertices.size() - 1];
            if (first.x != last.x || first.y != last.y)
                tmpPolygonVertices.emplace_back(first);

            auto twicearea = 0.0,
                 x = 0.0, y = 0.0, f = 0.0;
            auto nPts = tmpPolygonVertices.size();
            Vector2D p1, p2;

            for (size_t i = 0, j = nPts - 1; i < nPts; j = i++)
            {
                p1 = tmpPolygonVertices[i];
                p2 = tmpPolygonVertices[j];
                f = p1.x * p2.y - p2.x * p1.y;
                twicearea += f;
                x += (p1.x + p2.x) * f;
                y += (p1.y + p2.y) * f;
            }
            f = twicearea * 3.0;

            // KIRI_LOG_DEBUG("GetCentroid -------");
            // for (size_t i = 0; i < tmpPolygonVertices.size(); i++)
            // {
            //     KIRI_LOG_DEBUG("vert={0},{1}", tmpPolygonVertices[i].x, tmpPolygonVertices[i].y);
            // }
            // KIRI_LOG_DEBUG("GetCentroid -------,f={0}", f);

            return Vector2D(x / f, y / f);
        }

        BoundingBox2D BBox;
        std::vector<Vector2D> Verts;
    };

    class VoronoiPolygon3
    {
    public:
        explicit VoronoiPolygon3() {}
        virtual ~VoronoiPolygon3() noexcept {}

        void UpdateBBox()
        {
            BBox.reset();
            for (auto i = 0; i < Positions.size(); i++)
                BBox.merge(Positions[i]);
        }

        BoundingBox3D BBox;
        std::vector<Vector3D> Positions;
        std::vector<Vector3D> Normals;
        std::vector<int> Indices;
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_CELL_POLYGON_H_ */