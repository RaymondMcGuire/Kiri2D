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
        explicit ConnectorList() : First{nullptr}, Last{nullptr} {}

        virtual ~ConnectorList() noexcept {}

        std::shared_ptr<SimplexConnector<VERTEX>> First;
        std::shared_ptr<SimplexConnector<VERTEX>> Last;

        void Clear()
        {
            First = nullptr;
            Last = nullptr;
        }

        void AddFirst(const std::shared_ptr<SimplexConnector<VERTEX>> &connector)
        {
            connector->Next = First;
            if (First == nullptr)
                Last = connector;
            else
                First->Prev = connector;

            First = connector;
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
                First = connector->Next;
            }

            if (connector->Next != nullptr)
            {
                std::shared_ptr<SimplexConnector<VERTEX>> next{connector->Next};
                next->Prev = connector->Prev;
            }
            else if (connector->Next == nullptr)
            {
                std::shared_ptr<SimplexConnector<VERTEX>> prev{connector->Prev};
                Last = prev;
            }

            connector->Next = nullptr;
            connector->Prev.reset();
        }

        void Add(const std::shared_ptr<SimplexConnector<VERTEX>> &connector)
        {

            if (Last != nullptr)
            {
                Last->Next = connector;
            }

            connector->Prev = Last;
            Last = connector;

            if (First == nullptr)
            {
                First = connector;
            }
        }

        void ToString()
        {
            KIRI_LOG_DEBUG("---Connector List Start---");
            std::shared_ptr<SimplexConnector<VERTEX>> current{First};
            while (current != nullptr)
            {
                current->ToString();
                current = current->Next;
            }
            KIRI_LOG_DEBUG("---Connector List End---");
        }
    };

} // namespace HDV::Hull

#endif /* _HDV_CONNECTOR_LIST_H_ */