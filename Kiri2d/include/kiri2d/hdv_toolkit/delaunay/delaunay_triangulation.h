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
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class DelaunayTriangulation
    {
    public:
        explicit DelaunayTriangulation() {}
        explicit DelaunayTriangulation(int dimension)
        {
            Dimension = dimension;
            Centroid.assign(dimension, 0.f);
        }
        virtual ~DelaunayTriangulation() noexcept {}

        int Dimension;

        std::vector<VERTEX> Vertices;

        std::vector<std::shared_ptr<DelaunayCell<VERTEX>>> Cells;

        std::vector<float> Centroid;

        virtual void Clear()
        {
            Cells.clear();
            Vertices.clear();
            Centroid.assign(Dimension, 0.f);
        }

        virtual void Generate(const std::vector<VERTEX> &input, bool assignIds = true, bool checkInput = false) = 0;
    };

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_TRIANGULATION_H_ */