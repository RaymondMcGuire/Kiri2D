/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2022-05-09 11:39:05
 * @LastEditors: Xu.WANG
 * @Description:
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