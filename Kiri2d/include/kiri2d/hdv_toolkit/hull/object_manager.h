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
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr>
    class ObjectManager
    {
    public:
        explicit ObjectManager() {}
        explicit ObjectManager(int dimension) { mDimension = dimension; }

        virtual ~ObjectManager() noexcept {}

        void Clear()
        {
            mRecycledFaceStack = std::stack<std::shared_ptr<SimplexWrap<VERTEXPTR>>>();
            mConnectorStack = std::stack<std::shared_ptr<SimplexConnector<VERTEXPTR>>>();
            mEmptyBufferStack = std::stack<std::shared_ptr<VertexBuffer<VERTEXPTR>>>();
            mDeferredSimplexStack = std::stack<std::shared_ptr<DeferredSimplex<VERTEXPTR>>>();
        }

        void DepositFace(const std::shared_ptr<SimplexWrap<VERTEXPTR>> &face)
        {
            face->Prev.reset();
            face->Next = nullptr;

            for (auto i = 0; i < mDimension; i++)
            {
                face->AdjacentFaces[i] = nullptr;
            }
            mRecycledFaceStack.push(face);
        }

        std::shared_ptr<SimplexWrap<VERTEXPTR>> GetFace()
        {
            if (mRecycledFaceStack.size() != 0)
            {
                auto val = mRecycledFaceStack.top();
                mRecycledFaceStack.pop();
                return val;
            }
            else
                return std::make_shared<SimplexWrap<VERTEXPTR>>(mDimension, GetVertexBuffer());
        }

        void DepositConnector(const std::shared_ptr<SimplexConnector<VERTEXPTR>> &connector)
        {
            connector->Face = nullptr;
            connector->Prev.reset();
            connector->Next = nullptr;
            mConnectorStack.push(connector);
        }

        std::shared_ptr<SimplexConnector<VERTEXPTR>> GetConnector()
        {

            if (mConnectorStack.size() != 0)
            {
                auto val = mConnectorStack.top();
                mConnectorStack.pop();
                return val;
            }
            else
                return std::make_shared<SimplexConnector<VERTEXPTR>>(mDimension);
        }

        void DepositVertexBuffer(const std::shared_ptr<VertexBuffer<VERTEXPTR>> &buffer)
        {
            buffer->Clear();
            mEmptyBufferStack.push(buffer);
        }

        std::shared_ptr<VertexBuffer<VERTEXPTR>> GetVertexBuffer()
        {
            if (mEmptyBufferStack.size() != 0)
            {
                auto val = mEmptyBufferStack.top();
                mEmptyBufferStack.pop();
                return val;
            }
            else
                return std::make_shared<VertexBuffer<VERTEXPTR>>();
        }

        void DepositDeferredSimplex(const std::shared_ptr<DeferredSimplex<VERTEXPTR>> &face)
        {
            face->Face = nullptr;
            face->Pivot = nullptr;
            face->OldFace = nullptr;
            mDeferredSimplexStack.push(face);
        }

        std::shared_ptr<DeferredSimplex<VERTEXPTR>> GetDeferredSimplex()
        {
            if (mDeferredSimplexStack.size() != 0)
            {
                auto val = mDeferredSimplexStack.top();
                mDeferredSimplexStack.pop();
                return val;
            }
            else
                return std::make_shared<DeferredSimplex<VERTEXPTR>>();
        }

    private:
        int mDimension = -1;

        std::stack<std::shared_ptr<SimplexWrap<VERTEXPTR>>> mRecycledFaceStack;
        std::stack<std::shared_ptr<SimplexConnector<VERTEXPTR>>> mConnectorStack;
        std::stack<std::shared_ptr<VertexBuffer<VERTEXPTR>>> mEmptyBufferStack;
        std::stack<std::shared_ptr<DeferredSimplex<VERTEXPTR>>> mDeferredSimplexStack;
    };
} // namespace HDV::Hull

#endif /* _HDV_OBJECT_MANAGER_H_ */