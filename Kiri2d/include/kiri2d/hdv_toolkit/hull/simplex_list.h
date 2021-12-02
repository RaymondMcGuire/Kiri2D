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
        explicit SimplexList() : mFirst{nullptr}, mLast{nullptr} {}

        virtual ~SimplexList() noexcept {}

        void Clear()
        {
            mFirst = nullptr;
            mLast = nullptr;
        }

        void AddFirst(const std::shared_ptr<SimplexWrap<VERTEX>> &face)
        {
            face->SetInList(true);
            face->Next = mFirst;
            if (mFirst == nullptr)
                mLast = face;
            else
                mFirst->Prev = face;

            mFirst = face;
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
                mFirst = face->Next;
            }

            if (face->Next != nullptr)
            {
                std::shared_ptr<SimplexWrap<VERTEX>> next{face->Next};
                next->Prev = face->Prev;
            }
            else if (face->Next == nullptr)
            {
                std::shared_ptr<SimplexWrap<VERTEX>> prev{face->Prev};
                mLast = prev;
            }

            face->Next = nullptr;
            face->Prev.reset();
        }

        void Add(const std::shared_ptr<SimplexWrap<VERTEX>> &face)
        {
            if (face->GetInList())
            {
                if (mFirst->GetVerticesBeyond().GetCount() < face->GetVerticesBeyond().GetCount())
                {
                    Remove(face);
                    AddFirst(face);
                }
                return;
            }

            face->SetInList(true);

            if (mFirst != nullptr && mFirst->GetVerticesBeyond().GetCount() < face->GetVerticesBeyond().GetCount())
            {
                mFirst->Prev = face;
                face->Next = mFirst;
                mFirst = face;
            }
            else
            {
                if (mLast != nullptr)
                {
                    mLast->Next = face;
                }

                face->Prev = mLast;
                mLast = face;

                if (mFirst == nullptr)
                {
                    mFirst = face;
                }
            }
        }

        void ToString()
        {
            KIRI_LOG_DEBUG("---Simplex List Start---");
            std::shared_ptr<SimplexWrap<VERTEX>> current{mFirst};
            while (current != nullptr)
            {
                current->ToString();
                current = current->Next;
            }
            KIRI_LOG_DEBUG("---Simplex List End---");
        }

    private:
        std::shared_ptr<SimplexWrap<VERTEX>> mFirst;
        std::shared_ptr<SimplexWrap<VERTEX>> mLast;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_LIST_H_ */