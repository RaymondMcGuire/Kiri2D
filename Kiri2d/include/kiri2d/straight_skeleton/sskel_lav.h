/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 10:58:21
 * @LastEditTime: 2021-07-24 22:22:50
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
                this->Push(std::make_shared<SSkelVertex>(
                    Vector4F(prev.x, prev.y, curr.x, curr.y),
                    curr,
                    Vector4F(curr.x, curr.y, next.x, next.y)));
            }

            InitEdgesData();
        }

        SSkelLAV() = default;
        SSkelLAV(const SSkelLAV &) = delete;
        SSkelLAV &operator=(const SSkelLAV &) = delete;

        ~SSkelLAV() {}

        UInt Length() { return mCounter; }
        Vector<std::tuple<Vector2F, Vec_Vec2F>> GetSkeletons() const { return mSkeletons; }

        void PrintSSkelLAV();
        void PrintSSkelEdges();

        void InitEdgesData();
        void GenInitEvents();
        void HandleEvents();

    private:
        UInt mCounter = 0;
        SSkelVertexPtr mHead = NULL;
        std::priority_queue<
            SSkelEventPtr,
            Vector<SSkelEventPtr>,
            SSkelEventCmpDistance>
            mPriorityQueue;

        Vector<std::tuple<Vector2F, Vec_Vec2F>> mSkeletons;

        // edge/ bisector left/ right
        Vector<std::tuple<Vector4F, Vector4F, Vector4F>> mEdges;

        void Push(const SSkelVertexPtr &x);
        SSkelVertexPtr Unify(const SSkelVertexPtr &va, const SSkelVertexPtr &vb, Vector2F mid);

        Vector<SSkelEventPtr> GenSplitEventByVertex(const SSkelVertexPtr &vertex);
        SSkelEventPtr GenEventByVertex(const SSkelVertexPtr &vertex);
        Vector<SSkelEventPtr> HandleEdgeEvent(const SSkelEdgeEventPtr &edgeEvent);
    };
    typedef SharedPtr<SSkelLAV> SSkelLAVPtr;
}

#endif