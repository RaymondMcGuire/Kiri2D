/***
 * @Author: Xu.WANG
 * @Date: 2021-12-21 21:10:15
 * @LastEditTime: 2021-12-22 19:03:19
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_DELAUNAY_TRIANGULATION_H_
#define _HDV_DELAUNAY_TRIANGULATION_H_

#pragma once

#include <kiri2d/hdv_toolkit/delaunay/delaunay_cell.h>

namespace HDV::Delaunay
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class DelaunayTriangulation
    {
    public:
        explicit DelaunayTriangulation() {}
        explicit DelaunayTriangulation(int dimension)
        {
            Dimension = dimension;
            Centroid = std::make_shared<VERTEX>();
        }
        virtual ~DelaunayTriangulation() noexcept {}

        int Dimension;

        std::vector<VERTEXPTR> Vertices;

        std::vector<std::shared_ptr<DelaunayCell<VERTEXPTR, VERTEX>>> Cells;

        VERTEXPTR Centroid;

        std::shared_ptr<HDV::Hull::ConvexHull<VERTEXPTR>> Hull;

        virtual void Clear()
        {
            for (auto i = 0; i < Cells.size(); i++)
            {
                Cells[i]->Clear();
            }

            Cells.clear();
            Vertices.clear();
            Centroid = std::make_shared<VERTEX>();

            if (Hull != nullptr)
                Hull->Clear();
        }

        virtual void Generate(const std::vector<VERTEXPTR> &input, bool assignIds = true, bool checkInput = false) = 0;
    };

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_TRIANGULATION_H_ */