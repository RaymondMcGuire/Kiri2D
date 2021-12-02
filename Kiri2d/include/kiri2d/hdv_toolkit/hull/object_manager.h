/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 17:34:05
 * @LastEditTime: 2021-12-02 17:48:29
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_OBJECT_MANAGER_H_
#define _HDV_OBJECT_MANAGER_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/simplex_connecter.h>
#include <kiri2d/hdv_toolkit/hull/deferred_simplex.h>
namespace HDV::Hull
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class ObjectManager
    {
    public:
        explicit ObjectManager() {}
        explicit ObjectManager(int dimension) { mDimension = dimension; }

        virtual ~ObjectManager() noexcept {}

        void Clear()
        {
            mRecycledFaceStack.clear();
            mConnectorStack.clear();
            mEmptyBufferStack.clear();
            mDeferredSimplexStack.clear();
        }

        void DepositFace(const std::shared_ptr<SimplexWrap<VERTEX>> &face)
        {

            face->Prev = nullptr;
            face->Next = nullptr;

            for (auto i = 0; i < mDimension; i++)
            {
                face->GetAdjacentFaces()[i] = nullptr;
            }
            mRecycledFaceStack.emplace_back(face);
        }

        const std::shared_ptr<SimplexWrap<VERTEX>> &GetFace()
        {
            return mRecycledFaceStack.size() != 0
                       ? mRecycledFaceStack.pop_back()
                       : std::make_shared<SimplexWrap<VERTEX>>(mDimension, GetVertexBuffer());
        }

        void DepositConnector(const std::shared_ptr<SimplexConnector<VERTEX>> &connector)
        {
            connector->GetFace() = nullptr;
            connector->Prev = nullptr;
            connector->Next = nullptr;
            mConnectorStack.emplace_back(connector);
        }

        const std::shared_ptr<SimplexConnector<VERTEX>> &GetConnector()
        {
            return mConnectorStack.size() != 0
                       ? mConnectorStack.pop_back()
                       : std::make_shared<SimplexConnector<VERTEX>>(mDimension);
        }

        void DepositVertexBuffer(const std::shared_ptr<VertexBuffer<VERTEX>> &buffer)
        {
            buffer.Clear();
            mEmptyBufferStack.emplace_back(buffer);
        }

        const std::shared_ptr<VertexBuffer<VERTEX>> &GetVertexBuffer()
        {
            return mEmptyBufferStack.size() != 0 ? mEmptyBufferStack.pop_back() : std::make_shared<VertexBuffer<VERTEX>>();
        }

        void DepositDeferredSimplex(const std::shared_ptr<DeferredSimplex<VERTEX>> &face)
        {
            face->Face = nullptr;
            face->Pivot = nullptr;
            face->OldFace = nullptr;
            mDeferredSimplexStack.emplace_back(face);
        }

        const std::shared_ptr<DeferredSimplex<VERTEX>> &GetDeferredSimplex()
        {
            return mDeferredSimplexStack.size() != 0 ? mDeferredSimplexStack.pop_back() : std::make_shared<DeferredSimplex<VERTEX>>();
        }

    private:
        int mDimension = -1;

        std::vector<std::shared_ptr<SimplexWrap<VERTEX>>> mRecycledFaceStack;
        std::vector<std::shared_ptr<SimplexConnector<VERTEX>>> mConnectorStack;
        std::vector<std::shared_ptr<VertexBuffer<VERTEX>>> mEmptyBufferStack;
        std::vector<std::shared_ptr<DeferredSimplex<VERTEX>>> mDeferredSimplexStack;
    };
} // namespace HDV::Hull

#endif /* _HDV_OBJECT_MANAGER_H_ */