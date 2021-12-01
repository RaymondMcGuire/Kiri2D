/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 00:36:23
 * @LastEditTime: 2021-12-02 00:36:23
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_SIMPLEX_WARP_H_
#define _HDV_SIMPLEX_WARP_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/vertex_buffer.h>
namespace HDV::Hull
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class SimplexWarp
    {
    public:
        explicit SimplexWarp() : Next{nullptr}, Prev{} {}
        explicit SimplexWarp(
            int dimension,
            const VertexBuffer<VERTEX> &beyondList)
            : Next{nullptr}, Prev{}
        {
            mNormals.assign(dimension, 0.f);
            mVertices.assign(dimension, VERTEX());
            mAdjacentFaces.assign(dimension, SimplexWarp<VERTEX>());
            mBeyondList = beyondList;
        }
        virtual ~SimplexWarp() noexcept {}

        std::shared_ptr<SimplexWarp<VERTEX>> Next;
        std::weak_ptr<SimplexWarp<VERTEX>> Prev;

    private:
        bool mIsNormalFlipped;
        float mOffset;
        int mTag;
        bool mInList;
        VERTEX mFurthestVertex;
        VertexBuffer<VERTEX> mBeyondList;

        std::vector<float> mNormals;
        std::vector<VERTEX> mVertices;
        VertexBuffer<VERTEX> mVerticesBeyond;
        std::vector<SimplexWarp<VERTEX>> mAdjacentFaces;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_WARP_H_ */