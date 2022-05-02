/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 00:36:23
 * @LastEditTime: 2021-12-02 00:36:23
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_SIMPLEX_LIST_H_
#define _HDV_SIMPLEX_LIST_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/simplex_node.h>
namespace HDV::Hull
{
    class SimplexList
    {
    public:
        explicit SimplexList()
            : First{nullptr}, Last{nullptr}
        {
        }

        virtual ~SimplexList()
        {
        }

        std::shared_ptr<SimplexNode> First;
        std::shared_ptr<SimplexNode> Last;

        void clear()
        {
            removeAll();

            First = nullptr;
            Last = nullptr;
        }

        void AddFirst(const std::shared_ptr<SimplexNode> &simplex)
        {
            simplex->setInLinkList(true);
            simplex->Next = First;
            if (First == nullptr)
                Last = simplex;
            else
                First->Prev = simplex;

            First = simplex;
        }

        void removeAll()
        {
            std::shared_ptr<SimplexNode> tmp;
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

        void remove(const std::shared_ptr<SimplexNode> &simplex)
        {
            if (!simplex->inLinkList())
                return;

            simplex->setInLinkList(false);

            if (simplex->Prev.lock() != nullptr)
            {
                std::shared_ptr<SimplexNode> prev{simplex->Prev};
                prev->Next = simplex->Next;
            }
            else if (simplex->Prev.lock() == nullptr)
            {
                First = simplex->Next;
            }

            if (simplex->Next != nullptr)
            {
                std::shared_ptr<SimplexNode> next{simplex->Next};
                next->Prev = simplex->Prev;
            }
            else if (simplex->Next == nullptr)
            {
                if (simplex->Prev.lock() == nullptr)
                    Last = nullptr;
                else
                {
                    std::shared_ptr<SimplexNode> prev{simplex->Prev};
                    Last = prev;
                }
            }

            simplex->Next = nullptr;
            simplex->Prev.reset();
        }

        void add(const std::shared_ptr<SimplexNode> &simplex)
        {
            if (simplex->inLinkList())
            {
                if (First->verticesBeyond().size() < simplex->verticesBeyond().size())
                {
                    remove(simplex);
                    AddFirst(simplex);
                }
                return;
            }

            simplex->setInLinkList(true);

            if (First != nullptr && (First->verticesBeyond().size() < simplex->verticesBeyond().size()))
            {

                First->Prev = simplex;
                simplex->Next = First;
                First = simplex;
            }
            else
            {
                if (Last != nullptr)
                {
                    Last->Next = simplex;
                }

                simplex->Prev = Last;
                Last = simplex;

                if (First == nullptr)
                {
                    First = simplex;
                }
            }
        }

        void toString()
        {
            KIRI_LOG_DEBUG("---Simplex List Start---");
            std::shared_ptr<SimplexNode> current{First};
            while (current != nullptr)
            {
                current->toString();
                current = current->Next;
            }
            KIRI_LOG_DEBUG("---Simplex List End---");
        }
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_LIST_H_ */