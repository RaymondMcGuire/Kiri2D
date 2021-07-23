/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 10:58:21
 * @LastEditTime: 2021-07-23 14:39:23
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
        explicit SSkelLAV(Vec_Vec2F poly)
        {
            auto size = poly.size();
            for (size_t i = size; i < size + size; i++)
            {
                auto prev = poly[(i - 1) % size];
                auto curr = poly[i % size];
                auto next = poly[(i + 1) % size];
                this->Push(std::make_shared<SSkelVertex>(prev, curr, next));
            }
        }

        SSkelLAV() = default;
        SSkelLAV(const SSkelLAV &) = delete;
        SSkelLAV &operator=(const SSkelLAV &) = delete;

        ~SSkelLAV() {}

        UInt Length() { return mCounter; }
        Vector<std::tuple<Vector2F, Vec_Vec2F>> GetSkeletons() const { return mSkeletons; }

        void PrintSSkelLAV();

        void GenInitEvents();
        void HandleEvents();

    private:
        UInt mCounter = 0;
        SSkelVertexPtr mHead = NULL;
        std::priority_queue<
            SSkelEdgeEventPtr,
            Vector<SSkelEdgeEventPtr>,
            SSkelEdgeEventCmpDistance>
            mPriorityQueue;

        Vector<std::tuple<Vector2F, Vec_Vec2F>> mSkeletons;

        void Push(const SSkelVertexPtr &x);
        SSkelVertexPtr Unify(const SSkelVertexPtr &va, const SSkelVertexPtr &vb, Vector2F mid);
        SSkelEdgeEventPtr GenEdgeEventByVertex(const SSkelVertexPtr &vertex);
        Vector<SSkelEdgeEventPtr> HandleEdgeEvent(const SSkelEdgeEventPtr &edgeEvent);
    };
    typedef SharedPtr<SSkelLAV> SSkelLAVPtr;
}

#endif