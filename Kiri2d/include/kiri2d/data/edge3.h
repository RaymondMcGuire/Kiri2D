/*** 
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:49:01
 * @LastEditTime: 2021-05-18 01:23:05
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\data\edge3.h
 */

#ifndef _KIRI_EDGE3_H_
#define _KIRI_EDGE3_H_

#pragma once

#include <kiri2d/data/vertex3.h>

namespace KIRI2D
{
    class KiriEdge3
    {
    public:
        explicit KiriEdge3::KiriEdge3(UInt id, KiriVertex3 ori, KiriVertex3 dst)
        {
            mNext = NULL;
            mPrev = NULL;
            mTwin = NULL;

            mId = id;
            mOrigin = ori;
            mDest = dst;
        }

        ~KiriEdge3() noexcept {}

        constexpr UInt GetId() const { return mId; }
        void SetNextEdge(const SharedPtr<KiriEdge3> &nxt) { mNext = nxt; }
        void SetPrevEdge(const SharedPtr<KiriEdge3> &pre) { mPrev = pre; }
        void SetTwinEdge(const SharedPtr<KiriEdge3> &twn) { mTwin = twn; }

        const SharedPtr<KiriEdge3> &GetNextEdge() const { return mNext; }
        const SharedPtr<KiriEdge3> &GetPrevEdge() const { return mPrev; }
        const SharedPtr<KiriEdge3> &GetTwinEdge() const { return mTwin; }

        KiriVertex3 GetOriginVertex() const { return mOrigin; }
        KiriVertex3 GetDestVertex() const { return mDest; }

        const bool IsEqual(KiriVertex3 a, KiriVertex3 b)
        {
            //KIRI_LOG_DEBUG("edge equal ={0}", ((mOrigin.IsEqual(a) && mDest.IsEqual(b)) || (mOrigin.IsEqual(b) && mDest.IsEqual(a))));
            return ((mOrigin.IsEqual(a) && mDest.IsEqual(b)) || (mOrigin.IsEqual(b) && mDest.IsEqual(a)));
        }

        void PrintEdgeInfo()
        {
            KIRI_LOG_DEBUG("----------EDGE INFO----------");
            KIRI_LOG_DEBUG("edge info: id={0}, origin=({1},{2},{3}), dest=({4},{5},{6})",
                           mId,
                           mOrigin.GetValue().x, mOrigin.GetValue().y, mOrigin.GetValue().z,
                           mDest.GetValue().x, mDest.GetValue().y, mDest.GetValue().z);

            KIRI_LOG_DEBUG("prev edge id={0}, next edge id={1}, twin edge id={2},",
                           (mPrev != NULL) ? mPrev->GetId() : -1,
                           (mNext != NULL) ? mNext->GetId() : -1,
                           (mTwin != NULL) ? mTwin->GetId() : -1);
            KIRI_LOG_DEBUG("------------------------------");
        }

    private:
        UInt mId;
        KiriVertex3 mOrigin, mDest;
        SharedPtr<KiriEdge3> mNext;
        SharedPtr<KiriEdge3> mPrev;
        SharedPtr<KiriEdge3> mTwin;
    };
    typedef SharedPtr<KiriEdge3> KiriEdge3Ptr;
} // namespace KIRI

#endif /* _KIRI_EDGE3_H_ */