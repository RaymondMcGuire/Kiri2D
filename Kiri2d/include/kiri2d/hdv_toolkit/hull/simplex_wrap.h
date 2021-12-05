/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 00:36:23
 * @LastEditTime: 2021-12-05 20:31:59
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
            const std::shared_ptr<VertexBuffer<VERTEX>> &beyondList)
            : Next{nullptr}, Prev{}
        {
            mNormals.assign(dimension, 0.f);
            mVertices.assign(dimension, VERTEX());
            mAdjacentFaces.assign(dimension, std::make_shared<SimplexWrap<VERTEX>>());
            mBeyondList = beyondList;
        }
        virtual ~SimplexWrap() noexcept {}

        std::shared_ptr<std::shared_ptr<SimplexWrap<VERTEX>>> Next;
        std::weak_ptr<std::shared_ptr<SimplexWrap<VERTEX>>> Prev;

        std::vector<float> Normals;
        std::vector<VERTEX> Vertices;
        float Offset = 0.f;
        bool IsNormalFlipped = false;

        bool GetInList() const { return mInList; }
        void SetInList(bool il) { mInList = il; }

        int GetTag() const { return mTag; }
        void SetTag(int tag) { mTag = tag; }

        const std::shared_ptr<VertexBuffer<VERTEX>> &GetBeyondList() { return mBeyondList; }
        const std::shared_ptr<VertexBuffer<VERTEX>> &GetVerticesBeyond() { return mVerticesBeyond; }
        const std::vector<std::shared_ptr<SimplexWrap<VERTEX>>> &GetAdjacentFaces() { return mAdjacentFaces; }

        void ToString()
        {
            KIRI_LOG_DEBUG("Tag = {0}, InList = {1}, Prev tag = {2}, Next tag = {3}",
                           mTag,
                           mInList,
                           Prev.lock() == nullptr ? "null" : std::to_string(Prev.lock()->GetTag()),
                           Next == nullptr ? "null" : std::to_string(Next->GetTag()));
        }

    private:
        int mTag = -1;
        bool mInList = false;
        VERTEX mFurthestVertex;
        std::shared_ptr<VertexBuffer<VERTEX>> mBeyondList;
        std::shared_ptr<VertexBuffer<VERTEX>> mVerticesBeyond;
        std::vector<std::shared_ptr<SimplexWrap<VERTEX>>> mAdjacentFaces;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_WRAP_H_ */