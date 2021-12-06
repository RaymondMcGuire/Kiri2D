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

#include <kiri2d/hdv_toolkit/hull/simplex_wrap.h>
namespace HDV::Hull
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class SimplexList
    {
    public:
        explicit SimplexList() : First{nullptr}, Last{nullptr} {}

        virtual ~SimplexList() noexcept {}

        std::shared_ptr<SimplexWrap<VERTEX>> First;
        std::shared_ptr<SimplexWrap<VERTEX>> Last;

        void Clear()
        {
            First = nullptr;
            Last = nullptr;
        }

        void AddFirst(const std::shared_ptr<SimplexWrap<VERTEX>> &face)
        {
            face->SetInList(true);
            face->Next = First;
            if (First == nullptr)
                Last = face;
            else
                First->Prev = face;

            First = face;
        }

        void Remove(const std::shared_ptr<SimplexWrap<VERTEX>> &face)
        {
            if (!face->GetInList())
                return;

            face->SetInList(false);

            if (face->Prev.lock() != nullptr)
            {
                std::shared_ptr<SimplexWrap<VERTEX>> prev{face->Prev};
                prev->Next = face->Next;
            }
            else if (face->Prev.lock() == nullptr)
            {
                First = face->Next;
            }

            if (face->Next != nullptr)
            {
                std::shared_ptr<SimplexWrap<VERTEX>> next{face->Next};
                next->Prev = face->Prev;
            }
            else if (face->Next == nullptr)
            {
                std::shared_ptr<SimplexWrap<VERTEX>> prev{face->Prev};
                Last = prev;
            }

            face->Next = nullptr;
            face->Prev.reset();
        }

        void Add(const std::shared_ptr<SimplexWrap<VERTEX>> &face)
        {
            if (face->GetInList())
            {
                if (First->VerticesBeyond->GetCount() < face->VerticesBeyond->GetCount())
                {
                    Remove(face);
                    AddFirst(face);
                }
                return;
            }

            face->SetInList(true);

            if (First != nullptr && First->VerticesBeyond->GetCount() < face->VerticesBeyond->GetCount())
            {
                First->Prev = face;
                face->Next = First;
                First = face;
            }
            else
            {
                if (Last != nullptr)
                {
                    Last->Next = face;
                }

                face->Prev = Last;
                Last = face;

                if (First == nullptr)
                {
                    First = face;
                }
            }
        }

        void ToString()
        {
            KIRI_LOG_DEBUG("---Simplex List Start---");
            std::shared_ptr<SimplexWrap<VERTEX>> current{First};
            while (current != nullptr)
            {
                current->ToString();
                current = current->Next;
            }
            KIRI_LOG_DEBUG("---Simplex List End---");
        }
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_LIST_H_ */