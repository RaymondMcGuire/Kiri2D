/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-12-17 14:27:34
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 17:20:10
 * @FilePath: \core\include\kiri2d\straight_skeleton\sskel_lav.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _KIRI2D_SSKEL_LAV_H_
#define _KIRI2D_SSKEL_LAV_H_

#pragma once

#include <kiri2d/straight_skeleton/sskel_event.h>
#include <queue>
#include <tuple>

namespace KIRI2D::SSKEL
{
    class SSkelLAV
    {
    public:
        explicit SSkelLAV(UInt id) : mId(id) {}

        explicit SSkelLAV(
            UInt id,
            const SSkelVertexPtr &head)
            : mId(id)
        {
            mHead = head;
            if (mHead != nullptr)
            {
                // FIXME
                auto x = mHead;
                auto head_point = mHead->GetPoint();
                do
                {
                    mCounter++;
                    x->SetLAVId(id);
                    // KIRI_LOG_DEBUG("vertex={0},{1};head={0},{1}", x->GetPoint().x, x->GetPoint().y, mHead->GetPoint().x, mHead->GetPoint().y);
                    x = x->next.lock();
                } while ((x->GetPoint() - head_point).length() > std::numeric_limits<float>::epsilon());
            }
        }

        SSkelLAV() = default;
        SSkelLAV(const SSkelLAV &) = delete;
        SSkelLAV &operator=(const SSkelLAV &) = delete;

        ~SSkelLAV() {}

        UInt id() { return mId; }
        UInt length() { return mCounter; }

        const SSkelVertexPtr &head() { return mHead; }
        void setHead(const SSkelVertexPtr &head) { mHead = head; }

        void push(const SSkelVertexPtr &x);

        void print();

        Vector<std::tuple<Vector4F, Vector4F, Vector4F>> computeEdgesData();
        Vector<SSkelEventPtr> computeEvents(Vector<std::tuple<Vector4F, Vector4F, Vector4F>> originalEdges);
        SSkelEventPtr computeEventByVertex(const SSkelVertexPtr &vertex, Vector<std::tuple<Vector4F, Vector4F, Vector4F>> originalEdges);
        SSkelVertexPtr unify(SSkelVertexPtr va, SSkelVertexPtr vb, Vector2F mid);

    private:
        UInt mId;
        UInt mCounter = 0;
        SSkelVertexPtr mHead = nullptr;

        Vector<SSkelEventPtr> computeSplitEventByVertex(const SSkelVertexPtr &vertex, Vector<std::tuple<Vector4F, Vector4F, Vector4F>> originalEdges);
    };
    typedef SharedPtr<SSkelLAV> SSkelLAVPtr;
}

#endif