/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2022-01-02 21:14:10
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_VERTEX_BUFFER_H_
#define _HDV_VERTEX_BUFFER_H_

#pragma once

#include <kiri2d/hdv_toolkit/primitives/vertex.h>
namespace HDV::Hull
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr>
    class VertexBuffer
    {
    public:
        explicit VertexBuffer() {}
        explicit VertexBuffer(int capacity)
        {
            // mItems.assign(capacity, VERTEXPTR());
            mItems.assign(capacity, -1);
            mCapacity = capacity;
        }
        virtual ~VertexBuffer() noexcept {}

        int GetCount() const { return mCount; }
        int GetCapacity() const { return mCapacity; }
        // const VERTEXPTR &GetItem(int idx) { return mItems[idx]; }
        int GetItem(int idx) { return mItems[idx]; }

        void EnsureCapacity()
        {
            if (mCount + 1 > mCapacity)
            {
                if (mCapacity == 0)
                    mCapacity = 4;
                else
                    mCapacity = 2 * mCapacity;
                // mItems.resize(mCapacity, VERTEXPTR());
                mItems.resize(mCapacity, -1);
            };
        }

        // void Add(const VERTEXPTR &item)
        // {
        //     EnsureCapacity();
        //     mItems[mCount++] = item;
        // }

        void Add(int item)
        {
            EnsureCapacity();
            mItems[mCount++] = item;
        }

        void Clear()
        {
            mCount = 0;
        }

        void ToString()
        {
            // KIRI_LOG_DEBUG("Vertex Buffer: Capacity={0},Count={1}", mCapacity, mCount);
            // std::string indexs = "";
            // for (auto i = 0; i < mCount; i++)
            //     mItems[i]->ToString();

            // KIRI_LOG_DEBUG("Vertex Buffer------------------------");
        }

    private:
        int mCount = 0;
        int mCapacity = 0;
        // std::vector<VERTEXPTR> mItems;
        std::vector<int> mItems;
    };
} // namespace HDV::Hull

#endif /* _HDV_VERTEX_BUFFER_H_ */