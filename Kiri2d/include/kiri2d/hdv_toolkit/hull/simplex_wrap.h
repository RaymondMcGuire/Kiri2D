/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 00:36:23
 * @LastEditTime: 2021-12-06 12:49:19
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
            Normals.assign(dimension, 0.f);
            Vertices.assign(dimension, VERTEX());
            AdjacentFaces.assign(dimension, std::make_shared<SimplexWrap<VERTEX>>());
            BeyondList = beyondList;
            VerticesBeyond = std::make_shared<VertexBuffer<VERTEX>>();
        }
        virtual ~SimplexWrap() noexcept {}

        std::shared_ptr<SimplexWrap<VERTEX>> Next;
        std::weak_ptr<SimplexWrap<VERTEX>> Prev;

        float Offset = 0.f;
        bool IsNormalFlipped = false;

        bool GetInList() const { return mInList; }
        void SetInList(bool il) { mInList = il; }

        int GetTag() const { return mTag; }
        void SetTag(int tag) { mTag = tag; }

        std::vector<float> Normals;
        std::vector<VERTEX> Vertices;
        VERTEX FurthestVertex;
        std::shared_ptr<VertexBuffer<VERTEX>> BeyondList;
        std::shared_ptr<VertexBuffer<VERTEX>> VerticesBeyond;
        std::vector<std::shared_ptr<SimplexWrap<VERTEX>>> AdjacentFaces;

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
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_WRAP_H_ */