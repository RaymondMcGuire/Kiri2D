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

    class ObjectManager
    {
    public:
        explicit ObjectManager()
        {
        }

        explicit ObjectManager(int dimension)
        {
            mDimension = dimension;
        }

        virtual ~ObjectManager()
        {
        }

        void clear()
        {
            mRecycledFaceStack = std::stack<std::shared_ptr<SimplexNode>>();
            mConnectorStack = std::stack<std::shared_ptr<SimplexConnector>>();
            mEmptyBufferStack = std::stack<std::vector<int>>();
            mDeferredSimplexStack = std::stack<std::shared_ptr<DeferredSimplex>>();
        }

        void depositSimplex(const std::shared_ptr<SimplexNode> &simplex)
        {
            simplex->Prev.reset();
            simplex->Next = nullptr;

            for (auto i = 0; i < mDimension; i++)
            {
                simplex->adjacentFaces()[i] = nullptr;
            }
            mRecycledFaceStack.push(simplex);
        }

        std::shared_ptr<SimplexNode> simplex()
        {
            if (mRecycledFaceStack.size() != 0)
            {
                auto val = mRecycledFaceStack.top();
                mRecycledFaceStack.pop();
                return val;
            }
            else
                return std::make_shared<SimplexNode>(mDimension, vertexBuffer());
        }

        void depositConnector(const std::shared_ptr<SimplexConnector> &connector)
        {
            connector->Simplex = nullptr;
            connector->Prev.reset();
            connector->Next = nullptr;
            mConnectorStack.push(connector);
        }

        std::shared_ptr<SimplexConnector> connector()
        {

            if (mConnectorStack.size() != 0)
            {
                auto val = mConnectorStack.top();
                mConnectorStack.pop();
                return val;
            }
            else
                return std::make_shared<SimplexConnector>(mDimension);
        }

        void depositVertexBuffer(std::vector<int> buffer)
        {
            buffer.clear();
            mEmptyBufferStack.push(buffer);
        }

        std::vector<int> vertexBuffer()
        {
            if (mEmptyBufferStack.size() != 0)
            {
                auto val = mEmptyBufferStack.top();
                mEmptyBufferStack.pop();
                return val;
            }
            else
                return std::vector<int>();
        }

        void depositDeferredSimplex(const std::shared_ptr<DeferredSimplex> &simplex)
        {
            simplex->Face = nullptr;
            simplex->Pivot = nullptr;
            simplex->OldFace = nullptr;
            mDeferredSimplexStack.push(simplex);
        }

        std::shared_ptr<DeferredSimplex> deferredSimplex()
        {
            if (mDeferredSimplexStack.size() != 0)
            {
                auto val = mDeferredSimplexStack.top();
                mDeferredSimplexStack.pop();
                return val;
            }
            else
                return std::make_shared<DeferredSimplex>();
        }

    private:
        int mDimension = -1;

        std::stack<std::shared_ptr<SimplexNode>> mRecycledFaceStack;
        std::stack<std::shared_ptr<SimplexConnector>> mConnectorStack;
        std::stack<std::vector<int>> mEmptyBufferStack;
        std::stack<std::shared_ptr<DeferredSimplex>> mDeferredSimplexStack;
    };
} // namespace HDV::Hull

#endif /* _HDV_OBJECT_MANAGER_H_ */