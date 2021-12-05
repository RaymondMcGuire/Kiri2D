/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 00:36:23
 * @LastEditTime: 2021-12-02 00:36:23
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_CONNECTOR_LIST_H_
#define _HDV_CONNECTOR_LIST_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/simplex_connecter.h>
namespace HDV::Hull
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class ConnectorList
    {
    public:
        explicit ConnectorList() : mFirst{nullptr}, mLast{nullptr} {}

        virtual ~ConnectorList() noexcept {}

        void Clear()
        {
            mFirst = nullptr;
            mLast = nullptr;
        }

        void AddFirst(const std::shared_ptr<SimplexConnector<VERTEX>> &connector)
        {
            connector->Next = mFirst;
            if (mFirst == nullptr)
                mLast = connector;
            else
                mFirst->Prev = connector;

            mFirst = connector;
        }

        void Remove(const std::shared_ptr<SimplexConnector<VERTEX>> &connector)
        {

            if (connector->Prev.lock() != nullptr)
            {
                std::shared_ptr<SimplexConnector<VERTEX>> prev{connector->Prev};
                prev->Next = connector->Next;
            }
            else if (connector->Prev.lock() == nullptr)
            {
                mFirst = connector->Next;
            }

            if (connector->Next != nullptr)
            {
                std::shared_ptr<SimplexConnector<VERTEX>> next{connector->Next};
                next->Prev = connector->Prev;
            }
            else if (connector->Next == nullptr)
            {
                std::shared_ptr<SimplexConnector<VERTEX>> prev{connector->Prev};
                mLast = prev;
            }

            connector->Next = nullptr;
            connector->Prev.reset();
        }

        void Add(const std::shared_ptr<SimplexConnector<VERTEX>> &connector)
        {

            if (mLast != nullptr)
            {
                mLast->Next = connector;
            }

            connector->Prev = mLast;
            mLast = connector;

            if (mFirst == nullptr)
            {
                mFirst = connector;
            }
        }

        void ToString()
        {
            KIRI_LOG_DEBUG("---Connector List Start---");
            std::shared_ptr<SimplexConnector<VERTEX>> current{mFirst};
            while (current != nullptr)
            {
                current->ToString();
                current = current->Next;
            }
            KIRI_LOG_DEBUG("---Connector List End---");
        }

    private:
        std::shared_ptr<SimplexConnector<VERTEX>> mFirst;
        std::shared_ptr<SimplexConnector<VERTEX>> mLast;
    };

} // namespace HDV::Hull

#endif /* _HDV_CONNECTOR_LIST_H_ */