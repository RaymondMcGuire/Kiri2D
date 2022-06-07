/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-10-05 00:08:58
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 17:22:40
 * @FilePath: \core\include\kiri2d\straight_skeleton\sskel_slav.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _KIRI2D_SSKEL_SLAV_H_
#define _KIRI2D_SSKEL_SLAV_H_

#pragma once

#include <kiri2d/straight_skeleton/sskel_lav.h>
#include <unordered_map>

namespace KIRI2D::SSKEL
{
    class SSkelSLAV
    {
    public:
        explicit SSkelSLAV(Vec_Vec2F poly)
        {

            auto lav_poly = std::make_shared<KIRI2D::SSKEL::SSkelLAV>(mLAVCounter++);

            mLAVMaps[lav_poly->id()] = lav_poly;

            auto size = poly.size();
            for (size_t i = size; i < size + size; i++)
            {
                auto prev = poly[(i - 1) % size];
                auto curr = poly[i % size];
                auto next = poly[(i + 1) % size];
                lav_poly->push(std::make_shared<SSkelVertex>(
                    lav_poly->id(),
                    Vector4F(prev.x, prev.y, curr.x, curr.y),
                    curr,
                    Vector4F(curr.x, curr.y, next.x, next.y)));
            }

            mEdges = lav_poly->computeEdgesData();
            auto events = lav_poly->computeEvents(mEdges);

            // reset priority queue
            mPriorityQueue =
                std::priority_queue<
                    SSkelEventPtr,
                    Vector<SSkelEventPtr>,
                    SSkelEventCmpDistance>();

            for (size_t i = 0; i < events.size(); i++)
                mPriorityQueue.push(events[i]);

            // debug prior queue
            //  while (!mPriorityQueue.empty())
            //  {
            //      auto p = mPriorityQueue.top();
            //      mPriorityQueue.pop();
            //      p->print();
            //  }

            // KIRI_LOG_DEBUG("-----handle start-----");
            this->HandleEvents();
        }

        SSkelSLAV() = default;
        SSkelSLAV(const SSkelSLAV &) = delete;
        SSkelSLAV &operator=(const SSkelSLAV &) = delete;

        ~SSkelSLAV() {}

        void PrintSSkelEdges();

        Vector<std::tuple<Vector2F, Vec_Vec2F>> GetSkeletons() const { return mSkeletons; }

        void HandleEvents();

    private:
        UInt mLAVCounter = 0;
        std::unordered_map<UInt, SSkelLAVPtr> mLAVMaps;
        Vector<std::tuple<Vector4F, Vector4F, Vector4F>> mEdges;

        Vector<
            std::tuple<Vector2F, Vec_Vec2F>>
            mSkeletons;

        std::priority_queue<
            SSkelEventPtr,
            Vector<SSkelEventPtr>,
            SSkelEventCmpDistance>
            mPriorityQueue;

        Vector<SSkelEventPtr> HandleEdgeEvent(const SSkelEdgeEventPtr &edgeEvent);
        Vector<SSkelEventPtr> HandleSplitEvent(const SSkelSplitEventPtr &splitEvent);
    };
    typedef SharedPtr<SSkelSLAV> SSkelSLAVPtr;
}

#endif