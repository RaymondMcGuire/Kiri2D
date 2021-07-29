/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 10:58:21
 * @LastEditTime: 2021-07-25 19:02:22
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\straight_skeleton\sskel_lav.h
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
            if (mHead != NULL)
            {
                auto x = mHead;
                do
                {
                    mCounter++;
                    x->SetLAVId(id);
                    //KIRI_LOG_DEBUG("vertex={0},{1}", x->GetPoint().x, x->GetPoint().y);
                    x = x->next.lock();
                } while (x != mHead);
            }
        }

        SSkelLAV() = default;
        SSkelLAV(const SSkelLAV &) = delete;
        SSkelLAV &operator=(const SSkelLAV &) = delete;

        ~SSkelLAV() {}

        UInt GetId() { return mId; }
        UInt Length() { return mCounter; }

        const SSkelVertexPtr &GetHead() { return mHead; }
        void SetHead(const SSkelVertexPtr &head) { mHead = head; }

        void Push(const SSkelVertexPtr &x);

        void PrintSSkelLAV();

        Vector<std::tuple<Vector4F, Vector4F, Vector4F>> GenEdgesData();
        Vector<SSkelEventPtr> GenEvents(Vector<std::tuple<Vector4F, Vector4F, Vector4F>> originalEdges);
        SSkelEventPtr GenEventByVertex(const SSkelVertexPtr &vertex, Vector<std::tuple<Vector4F, Vector4F, Vector4F>> originalEdges);
        SSkelVertexPtr Unify(const SSkelVertexPtr &va, const SSkelVertexPtr &vb, Vector2F mid);

    private:
        UInt mId;
        UInt mCounter = 0;
        SSkelVertexPtr mHead = NULL;

        Vector<SSkelEventPtr> GenSplitEventByVertex(const SSkelVertexPtr &vertex, Vector<std::tuple<Vector4F, Vector4F, Vector4F>> originalEdges);
    };
    typedef SharedPtr<SSkelLAV> SSkelLAVPtr;
}

#endif