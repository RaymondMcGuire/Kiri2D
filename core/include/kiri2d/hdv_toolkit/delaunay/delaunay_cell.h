/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-12-23 17:57:21
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-05-24 10:01:15
 * @FilePath: \Kiri2D\core\include\kiri2d\hdv_toolkit\delaunay\delaunay_cell.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _HDV_DELAUNAY_CELL_H_
#define _HDV_DELAUNAY_CELL_H_

#pragma once

#include <kiri2d/hdv_toolkit/primitives/simplex.h>

namespace HDV::Delaunay
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class DelaunayCell
    {
    public:
        explicit DelaunayCell()
        {
        }

        explicit DelaunayCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> &simplex, std::vector<double> circumCenter, double radius)
        {
            mSimplex = simplex;

            mCircumCenter = std::make_shared<VERTEX>();
            mCircumCenter->positions() = circumCenter;

            mRadius = radius;
        }

        virtual ~DelaunayCell()
        {
        }

        constexpr double radius() const
        {
            return mRadius;
        }

        VERTEXPTR &circumCenter()
        {
            return mCircumCenter;
        }

        std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> &simplex()
        {
            return mSimplex;
        }

        void clear()
        {
        }

    private:
        double mRadius;
        VERTEXPTR mCircumCenter;
        std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> mSimplex;
    };

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_CELL_H_ */