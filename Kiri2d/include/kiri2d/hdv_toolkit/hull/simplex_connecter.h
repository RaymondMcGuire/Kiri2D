/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 16:48:58
 * @LastEditTime: 2021-12-02 16:49:48
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_SIMPLEX_CONNECTOR_H_
#define _HDV_SIMPLEX_CONNECTOR_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/simplex_wrap.h>

namespace HDV::Hull
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr>
    class SimplexConnector
    {
    public:
        explicit SimplexConnector() : Next{nullptr}, Prev{} {}
        explicit SimplexConnector(int dimension) : Next{nullptr}, Prev{}
        {
            VerticeIndexes.assign(dimension - 1, 0);
        }
        virtual ~SimplexConnector() noexcept {}

        std::shared_ptr<SimplexConnector<VERTEXPTR>> Next;
        std::weak_ptr<SimplexConnector<VERTEXPTR>> Prev;

        std::shared_ptr<SimplexWrap<VERTEXPTR>> Face;
        std::vector<int> VerticeIndexes;

        void Clear()
        {
            Next = nullptr;
            Prev.reset();
            Face = nullptr;
        }

        void Update(const std::shared_ptr<SimplexWrap<VERTEXPTR>> &face, int edgeIndex, int dim)
        {
            Face = face;
            mEdgeIndex = edgeIndex;

            uint hashCode = 31;

            auto vs = face->Vertices;
            for (auto i = 0, c = 0; i < dim; i++)
            {
                if (i != edgeIndex)
                {
                    // int v = vs[i]->GetId();
                    int v = vs[i];

                    VerticeIndexes[c++] = v;
                    hashCode += 23 * hashCode + (uint)v;
                }
            }

            mHashCode = hashCode;
        }

        static bool AreConnectable(
            const std::shared_ptr<SimplexConnector<VERTEXPTR>> &a,
            const std::shared_ptr<SimplexConnector<VERTEXPTR>> &b,
            int dim)
        {
            if (a->GetHashCode() != b->GetHashCode())
                return false;

            auto n = dim - 1;
            for (auto i = 0; i < n; i++)
            {
                if (a->VerticeIndexes[i] != b->VerticeIndexes[i])
                    return false;
            }

            return true;
        }

        static void Connect(
            const std::shared_ptr<SimplexConnector<VERTEXPTR>> &a,
            const std::shared_ptr<SimplexConnector<VERTEXPTR>> &b)
        {
            // KIRI_LOG_DEBUG("a edge index={0}; b edge index={1}", a->GetEdgeIndex(), b->GetEdgeIndex());
            a->Face->AdjacentFaces[a->GetEdgeIndex()] = b->Face;
            b->Face->AdjacentFaces[b->GetEdgeIndex()] = a->Face;
        }

        void ToString()
        {
            KIRI_LOG_DEBUG("HashCode = {0}, EdgeIndex = {1}, Prev HashCode = {2}, Next HashCode = {3}",
                           mHashCode,
                           mEdgeIndex,
                           Prev.lock() == nullptr ? "null" : std::to_string(Prev.lock()->GetHashCode()),
                           Next == nullptr ? "null" : std::to_string(Next->GetHashCode()));

            Face->ToString();
        }

        uint GetHashCode() const { return mHashCode; }
        int GetEdgeIndex() const { return mEdgeIndex; }

    private:
        uint mHashCode = -1;
        int mEdgeIndex = -1;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_CONNECTOR_H_ */