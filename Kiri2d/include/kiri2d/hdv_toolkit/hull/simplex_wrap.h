/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 00:36:23
 * @LastEditTime: 2021-12-02 17:50:29
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_SIMPLEX_WRAP_H_
#define _HDV_SIMPLEX_WRAP_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/vertex_buffer.h>
namespace HDV::Hull
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class SimplexWrap
    {
    public:
        explicit SimplexWrap() : Next{nullptr}, Prev{} {}
        explicit SimplexWrap(
            int dimension,
            const VertexBuffer<VERTEX> &beyondList)
            : Next{nullptr}, Prev{}
        {
            mNormals.assign(dimension, 0.f);
            mVertices.assign(dimension, VERTEX());
            mAdjacentFaces.assign(dimension, SimplexWrap<VERTEX>());
            mBeyondList = beyondList;
        }
        virtual ~SimplexWrap() noexcept {}

        std::shared_ptr<SimplexWrap<VERTEX>> Next;
        std::weak_ptr<SimplexWrap<VERTEX>> Prev;

        bool GetInList() const { return mInList; }
        void SetInList(bool il) { mInList = il; }

        int GetTag() const { return mTag; }
        void SetTag(int tag) { mTag = tag; }

        const VertexBuffer<VERTEX> &GetBeyondList() { return mBeyondList; }
        const std::vector<float> &GetNormals() { return mNormals; }
        const std::vector<VERTEX> &GetVertices() { return mVertices; }
        const VertexBuffer<VERTEX> &GetVerticesBeyond() { return mVerticesBeyond; }
        const std::vector<SimplexWrap<VERTEX>> &GetAdjacentFaces() { return mAdjacentFaces; }

        void ToString()
        {
            KIRI_LOG_DEBUG("Tag = {0}, InList = {1}, Prev tag = {2}, Next tag = {3}",
                           mTag,
                           mInList,
                           Prev.lock() == nullptr ? "null" : std::to_string(Prev.lock()->GetTag()),
                           Next == nullptr ? "null" : std::to_string(Next->GetTag()));
        }

    private:
        bool mIsNormalFlipped = false;
        float mOffset = 0.f;
        int mTag = -1;
        bool mInList = false;
        VERTEX mFurthestVertex;
        VertexBuffer<VERTEX> mBeyondList;

        std::vector<float> mNormals;
        std::vector<VERTEX> mVertices;
        VertexBuffer<VERTEX> mVerticesBeyond;
        std::vector<SimplexWrap<VERTEX>> mAdjacentFaces;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_WRAP_H_ */