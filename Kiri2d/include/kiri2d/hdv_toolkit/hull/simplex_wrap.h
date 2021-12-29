/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2021-12-29 14:59:24
 * @LastEditors: Xu.WANG
 * @Description:
 */
#ifndef _HDV_SIMPLEX_WRAP_H_
#define _HDV_SIMPLEX_WRAP_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/vertex_buffer.h>
namespace HDV::Hull
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr>
    class SimplexWrap
    {
    public:
        explicit SimplexWrap() : Next{nullptr}, Prev{} {}
        explicit SimplexWrap(
            int dimension,
            const std::shared_ptr<VertexBuffer<VERTEXPTR>> &beyondList)
            : Next{nullptr}, Prev{}
        {
            Normals.assign(dimension, 0.0);
            Vertices.assign(dimension, VERTEXPTR());
            AdjacentFaces.assign(dimension, std::make_shared<SimplexWrap<VERTEXPTR>>());
            BeyondList = beyondList;
            VerticesBeyond = std::make_shared<VertexBuffer<VERTEXPTR>>();
        }
        virtual ~SimplexWrap() noexcept
        {
        }

        void Clear()
        {
            Next = nullptr;
            Prev.reset();
            FurthestVertex = nullptr;
            BeyondList = nullptr;
            VerticesBeyond = nullptr;
            Vertices.clear();
            AdjacentFaces.clear();
        }

        std::shared_ptr<SimplexWrap<VERTEXPTR>> Next;
        std::weak_ptr<SimplexWrap<VERTEXPTR>> Prev;

        double Offset = 0.0;
        bool IsNormalFlipped = false;

        bool GetInList() const { return mInList; }
        void SetInList(bool il) { mInList = il; }

        int GetTag() const { return mTag; }
        void SetTag(int tag) { mTag = tag; }

        std::vector<double> Normals;
        std::vector<VERTEXPTR> Vertices;
        VERTEXPTR FurthestVertex;
        std::shared_ptr<VertexBuffer<VERTEXPTR>> BeyondList;
        std::shared_ptr<VertexBuffer<VERTEXPTR>> VerticesBeyond;
        std::vector<std::shared_ptr<SimplexWrap<VERTEXPTR>>> AdjacentFaces;

        void ToString()
        {
            std::string vert_data = "";
            for (size_t i = 0; i < Vertices.size(); i++)
            {
                vert_data += Vertices[i]->GetString();
                // vert_data += std::to_string(Vertices[i]->GetId()) + ":(" + std::to_string(Vertices[i]->GetPosition()[0]) + "," + std::to_string(Vertices[i]->GetPosition()[1]) + "); ";
            }

            KIRI_LOG_DEBUG("Tag = {0}, InList = {1}, Prev tag = {2}, Next tag = {3}; Verts={4}; Normal={5},{6}; Offset={7}; IsNormalFlipped={8}, FurthestVertex={9}",
                           mTag,
                           mInList,
                           Prev.lock() == nullptr ? "null" : std::to_string(Prev.lock()->GetTag()),
                           Next == nullptr ? "null" : std::to_string(Next->GetTag()),
                           vert_data,
                           Normals[0], Normals[1], Offset, IsNormalFlipped,
                           FurthestVertex == nullptr ? "null" : FurthestVertex->GetString());
        }

    private:
        int mTag = 0;
        bool mInList = false;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_WRAP_H_ */