/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2022-05-10 09:35:42
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_DELAUNAY_TRIANGULATION_H_
#define _HDV_DELAUNAY_TRIANGULATION_H_

#pragma once

#include <kiri2d/hdv_toolkit/delaunay/delaunay_cell.h>
#include <kiri2d/hdv_toolkit/hull/convex_hull.h>

namespace HDV::Delaunay
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class DelaunayTriangulation
    {
    public:
        explicit DelaunayTriangulation()
        {
        }

        explicit DelaunayTriangulation(int dimension)
        {
            mDimension = dimension;
            mCentroid = std::make_shared<VERTEX>();
        }

        virtual ~DelaunayTriangulation()
        {
        }

        const int dimension() const
        {
            return mDimension;
        }

        VERTEXPTR &centroid()
        {
            return mCentroid;
        }

        std::vector<VERTEXPTR> &vertices()
        {
            return mVertices;
        }

        std::vector<std::shared_ptr<DelaunayCell<VERTEXPTR, VERTEX>>> &cells()
        {
            return mCells;
        }

        std::shared_ptr<HDV::Hull::ConvexHull<VERTEXPTR>> &hull()
        {
            return mHull;
        }

        virtual void generate(const std::vector<VERTEXPTR> &input, bool assignIds = true, bool checkInput = false) = 0;

        virtual void clear()
        {
            for (auto i = 0; i < mCells.size(); i++)
            {
                mCells[i]->clear();
            }

            mCells.clear();
            mVertices.clear();
            mCentroid = std::make_shared<VERTEX>();

            if (mHull != nullptr)
                mHull->clear();
        }

    protected:
        int mDimension;

        std::vector<VERTEXPTR> mVertices;

        std::vector<std::shared_ptr<DelaunayCell<VERTEXPTR, VERTEX>>> mCells;

        VERTEXPTR mCentroid;

        std::shared_ptr<HDV::Hull::ConvexHull<VERTEXPTR>> mHull;
    };

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_TRIANGULATION_H_ */