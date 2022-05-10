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

#include <kiri2d/hdv_toolkit/hull/simplex_node.h>

namespace HDV::Hull
{
    class SimplexConnector
    {
    public:
        explicit SimplexConnector()
            : Next{nullptr}, Prev{}
        {
        }

        explicit SimplexConnector(int dimension)
            : Next{nullptr}, Prev{}
        {
            mVerticeIndexes.assign(dimension - 1, 0);
        }

        virtual ~SimplexConnector()
        {
        }

        std::shared_ptr<SimplexConnector> Next;
        std::weak_ptr<SimplexConnector> Prev;
        std::shared_ptr<SimplexNode> Simplex;

        UInt hashCode() const { return mHashCode; }
        int edgeIndex() const { return mEdgeIndex; }

        void clear()
        {
            Next = nullptr;
            Prev.reset();
            Simplex = nullptr;
        }

        void update(const std::shared_ptr<SimplexNode> &simplex, int edgeIndex, int dim)
        {
            Simplex = simplex;
            mEdgeIndex = edgeIndex;

            UInt hashCode = 31;

            auto vertices = simplex->vertices();
            for (auto i = 0, c = 0; i < dim; i++)
            {
                if (i != edgeIndex)
                {
                    mVerticeIndexes[c++] = vertices[i];
                    hashCode += 23 * hashCode + (UInt)vertices[i];
                }
            }

            mHashCode = hashCode;
        }

        static bool connectable(
            const std::shared_ptr<SimplexConnector> &a,
            const std::shared_ptr<SimplexConnector> &b,
            int dim)
        {
            if (a->hashCode() != b->hashCode())
                return false;

            auto n = dim - 1;
            for (auto i = 0; i < n; i++)
            {
                if (a->mVerticeIndexes[i] != b->mVerticeIndexes[i])
                    return false;
            }

            return true;
        }

        static void connect(
            const std::shared_ptr<SimplexConnector> &a,
            const std::shared_ptr<SimplexConnector> &b)
        {
            // KIRI_LOG_DEBUG("a edge index={0}; b edge index={1}", a->edgeIndex(), b->edgeIndex());
            a->Simplex->adjacentFaces()[a->edgeIndex()] = b->Simplex;
            b->Simplex->adjacentFaces()[b->edgeIndex()] = a->Simplex;
        }

        void toString()
        {
            KIRI_LOG_DEBUG("HashCode = {0}, EdgeIndex = {1}, Prev HashCode = {2}, Next HashCode = {3}",
                           mHashCode,
                           mEdgeIndex,
                           Prev.lock() == nullptr ? "null" : std::to_string(Prev.lock()->hashCode()),
                           Next == nullptr ? "null" : std::to_string(Next->hashCode()));

            Simplex->toString();
        }

    private:
        UInt mHashCode = -1;
        int mEdgeIndex = -1;
        std::vector<int> mVerticeIndexes;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_CONNECTOR_H_ */