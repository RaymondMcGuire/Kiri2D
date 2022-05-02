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
    class ConnectorList
    {
    public:
        explicit ConnectorList() : First{nullptr}, Last{nullptr} {}

        virtual ~ConnectorList() {}

        std::shared_ptr<SimplexConnector> First;
        std::shared_ptr<SimplexConnector> Last;

        void clear()
        {

            removeAll();
            First = nullptr;
            Last = nullptr;
        }

        void removeAll()
        {
            std::shared_ptr<SimplexConnector> tmp;
            while (First != nullptr)
            {
                tmp = First;
                First = First->Next;
                tmp->clear();
                tmp->Next = nullptr;
                tmp->Prev.reset();
                tmp = nullptr;
            }
        }

        void AddFirst(const std::shared_ptr<SimplexConnector> &connector)
        {
            connector->Next = First;
            if (First == nullptr)
                Last = connector;
            else
                First->Prev = connector;

            First = connector;
        }

        void remove(const std::shared_ptr<SimplexConnector> &connector)
        {

            // KIRI_LOG_DEBUG("Connector list prev={0}; next={1}", connector->Prev.lock() == nullptr ? "null" : "has value", connector->Next == nullptr ? "null" : "has value");

            if (connector->Prev.lock() != nullptr)
            {
                std::shared_ptr<SimplexConnector> prev{connector->Prev};
                prev->Next = connector->Next;
            }
            else if (connector->Prev.lock() == nullptr)
            {
                First = connector->Next;
            }

            if (connector->Next != nullptr)
            {
                std::shared_ptr<SimplexConnector> next{connector->Next};
                next->Prev = connector->Prev;
            }
            else if (connector->Next == nullptr)
            {

                if (connector->Prev.lock() == nullptr)
                    Last = nullptr;
                else
                {
                    std::shared_ptr<SimplexConnector> prev{connector->Prev};
                    Last = prev;
                }
            }

            connector->Next = nullptr;
            connector->Prev.reset();
        }

        void add(const std::shared_ptr<SimplexConnector> &connector)
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

        void toString()
        {
            KIRI_LOG_DEBUG("---Connector List Start---");
            std::shared_ptr<SimplexConnector> current{First};
            while (current != nullptr)
            {
                current->toString();
                current = current->Next;
            }
            KIRI_LOG_DEBUG("---Connector List End---");
        }
    };

} // namespace HDV::Hull

#endif /* _HDV_CONNECTOR_LIST_H_ */