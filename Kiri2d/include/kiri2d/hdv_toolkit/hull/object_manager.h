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
#include <stack>
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
            mRecycledFaceStack = std::stack<std::shared_ptr<SimplexWrap<VERTEX>>>();
            mConnectorStack = std::stack<std::shared_ptr<SimplexConnector<VERTEX>>>();
            mEmptyBufferStack = std::stack<std::shared_ptr<VertexBuffer<VERTEX>>>();
            mDeferredSimplexStack = std::stack<std::shared_ptr<DeferredSimplex<VERTEX>>>();
        }

        void DepositFace(const std::shared_ptr<SimplexWrap<VERTEX>> &face)
        {
            face->Prev.reset();
            face->Next = nullptr;

            for (auto i = 0; i < mDimension; i++)
            {
                face->AdjacentFaces[i] = nullptr;
            }
            mRecycledFaceStack.push(face);
        }

        std::shared_ptr<SimplexWrap<VERTEX>> GetFace()
        {
            if (mRecycledFaceStack.size() != 0)
            {
                auto val = mRecycledFaceStack.top();
                mRecycledFaceStack.pop();
                return val;
            }
            else
                return std::make_shared<SimplexWrap<VERTEX>>(mDimension, GetVertexBuffer());
        }

        void DepositConnector(const std::shared_ptr<SimplexConnector<VERTEX>> &connector)
        {
            auto faces = connector->GetFace();
            faces = nullptr;
            connector->Prev.reset();
            connector->Next = nullptr;
            mConnectorStack.push(connector);
        }

        std::shared_ptr<SimplexConnector<VERTEX>> GetConnector()
        {

            if (mConnectorStack.size() != 0)
            {
                auto val = mConnectorStack.top();
                mConnectorStack.pop();
                return val;
            }
            else
                return std::make_shared<SimplexConnector<VERTEX>>(mDimension);
        }

        void DepositVertexBuffer(const std::shared_ptr<VertexBuffer<VERTEX>> &buffer)
        {
            buffer->Clear();
            mEmptyBufferStack.push(buffer);
        }

        std::shared_ptr<VertexBuffer<VERTEX>> GetVertexBuffer()
        {
            if (mEmptyBufferStack.size() != 0)
            {
                auto val = mEmptyBufferStack.top();
                mEmptyBufferStack.pop();
                return val;
            }
            else
                return std::make_shared<VertexBuffer<VERTEX>>();
        }

        void DepositDeferredSimplex(const std::shared_ptr<DeferredSimplex<VERTEX>> &face)
        {
            face->Face = nullptr;
            face->Pivot = nullptr;
            face->OldFace = nullptr;
            mDeferredSimplexStack.push(face);
        }

        std::shared_ptr<DeferredSimplex<VERTEX>> GetDeferredSimplex()
        {
            if (mDeferredSimplexStack.size() != 0)
            {
                auto val = mDeferredSimplexStack.top();
                mDeferredSimplexStack.pop();
                return val;
            }
            else
                return std::make_shared<DeferredSimplex<VERTEX>>();
        }

    private:
        int mDimension = -1;

        std::stack<std::shared_ptr<SimplexWrap<VERTEX>>> mRecycledFaceStack;
        std::stack<std::shared_ptr<SimplexConnector<VERTEX>>> mConnectorStack;
        std::stack<std::shared_ptr<VertexBuffer<VERTEX>>> mEmptyBufferStack;
        std::stack<std::shared_ptr<DeferredSimplex<VERTEX>>> mDeferredSimplexStack;
    };
} // namespace HDV::Hull

#endif /* _HDV_OBJECT_MANAGER_H_ */