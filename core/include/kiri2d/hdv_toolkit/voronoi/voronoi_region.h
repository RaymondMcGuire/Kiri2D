/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2022-05-10 11:14:17
 * @LastEditors: Xu.WANG
 * @Description:
 */
#ifndef _HDV_VORONOI_REGION_H_
#define _HDV_VORONOI_REGION_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_edge.h>

namespace HDV::Voronoi
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiRegion
    {
    public:
        explicit VoronoiRegion()
        {
        }

        virtual ~VoronoiRegion()
        {
        }

        void clear()
        {
            for (auto i = 0; i < mCells.size(); i++)
                mCells[i]->clear();

            for (auto i = 0; i < mEdges.size(); i++)
                mEdges[i]->clear();

            mCells.clear();
            mEdges.clear();
            mSite = nullptr;
        }

        const int id() const
        {
            return mId;
        }

        void setId(int id)
        {
            mId = id;
        }

        VERTEXPTR &site()
        {
            return mSite;
        }

        std::vector<std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>> &cells()
        {
            return mCells;
        }

        std::vector<std::shared_ptr<VoronoiEdge<VERTEXPTR, VERTEX>>> &edges()
        {
            return mEdges;
        }

        BoundingBox2F bbox()
        {
            BoundingBox2F bounding_box;
            for (size_t i = 0; i < Edges.size(); i++)
            {
                auto from = Edges[i]->from()->circumCenter();
                auto to = Edges[i]->to()->circumCenter();
                bounding_box.merge(Vector2D(from->X(), from->Y()));
                bounding_box.merge(Vector2D(to->X(), to->Y()));
            }
            return bounding_box;
        }

    private:
        int mId;
        std::vector<std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>> mCells;
        std::vector<std::shared_ptr<VoronoiEdge<VERTEXPTR, VERTEX>>> mEdges;
        VERTEXPTR mSite;
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_REGION_H_ */