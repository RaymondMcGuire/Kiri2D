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
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class SimplexConnector
    {
    public:
        explicit SimplexConnector() : Next{nullptr}, Prev{} {}
        explicit SimplexConnector(int dimension) : Next{nullptr}, Prev{}
        {
            mVerticeIndexes.assign(dimension - 1, 0);
        }
        virtual ~SimplexConnector() noexcept {}

        std::shared_ptr<SimplexConnector<VERTEX>> Next;
        std::weak_ptr<SimplexConnector<VERTEX>> Prev;

        void Update(const std::shared_ptr<SimplexWrap<VERTEX>> &face, int edgeIndex, int dim)
        {
            mFace = face;
            mEdgeIndex = edgeIndex;

            uint hashCode = 31;

            auto vs = face->Vertices;
            for (auto i = 0, c = 0; i < dim; i++)
            {
                if (i != edgeIndex)
                {
                    int v = vs[i]->GetId();
                    mVerticeIndexes[c++] = v;
                    hashCode += 23 * hashCode + (uint)v;
                }
            }

            mHashCode = hashCode;
        }

        static bool AreConnectable(
            const std::shared_ptr<SimplexConnector<VERTEX>> &a,
            const std::shared_ptr<SimplexConnector<VERTEX>> &b,
            int dim)
        {
            if (a->GetHashCode() != b->GetHashCode())
                return false;

            auto n = dim - 1;
            auto av = a->GetVerticeIndexes();
            auto bv = b->GetVerticeIndexes();
            for (auto i = 0; i < n; i++)
            {
                if (av[i] != bv[i])
                    return false;
            }

            return true;
        }

        static void Connect(
            const std::shared_ptr<SimplexConnector<VERTEX>> &a,
            const std::shared_ptr<SimplexConnector<VERTEX>> &b)
        {
            a->GetFace()->AdjacentFaces[a->GetEdgeIndex()] = b->GetFace();
            b->GetFace()->AdjacentFaces[b->GetEdgeIndex()] = a->GetFace();
        }

        void ToString()
        {
            KIRI_LOG_DEBUG("HashCode = {0}, EdgeIndex = {1}, Prev HashCode = {2}, Next HashCode = {3}",
                           mHashCode,
                           mEdgeIndex,
                           Prev.lock() == nullptr ? "null" : std::to_string(Prev.lock()->GetHashCode()),
                           Next == nullptr ? "null" : std::to_string(Next->GetHashCode()));
        }

        uint GetHashCode() const { return mHashCode; }
        int GetEdgeIndex() const { return mEdgeIndex; }
        const std::shared_ptr<SimplexWrap<VERTEX>> &GetFace() { return mFace; }
        const std::vector<int> &GetVerticeIndexes() { return mVerticeIndexes; }

    private:
        uint mHashCode = -1;
        int mEdgeIndex = -1;
        std::shared_ptr<SimplexWrap<VERTEX>> mFace;
        std::vector<int> mVerticeIndexes;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_CONNECTOR_H_ */