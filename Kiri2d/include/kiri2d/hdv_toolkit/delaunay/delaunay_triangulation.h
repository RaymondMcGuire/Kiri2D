/***
 * @Author: Xu.WANG
 * @Date: 2021-12-08 14:55:27
 * @LastEditTime: 2021-12-08 14:58:25
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

        virtual void Clear()
        {
            Cells.clear();
            Vertices.clear();
            Centroid = std::make_shared<VERTEX>();
        }

        virtual void Generate(const std::vector<VERTEXPTR> &input, bool assignIds = true, bool checkInput = false) = 0;
    };

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_TRIANGULATION_H_ */