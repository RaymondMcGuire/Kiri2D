/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 10:58:21
 * @LastEditTime: 2021-07-23 14:15:56
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\straight_skeleton\sskel_event.h
 */

#ifndef _KIRI2D_SSKEL_EVENT_H_
#define _KIRI2D_SSKEL_EVENT_H_

#pragma once

#include <kiri2d/straight_skeleton/sskel_vertex.h>
namespace KIRI2D::SSKEL
{
    class SSkelEdgeEvent
    {
    public:
        explicit SSkelEdgeEvent(
            float distance,
            Vector2F intersect,
            const SSkelVertexPtr &va,
            const SSkelVertexPtr &vb)
            : mDistance(distance),
              mIntersect(intersect),
              mVertA(va),
              mVertB(vb)
        {
        }

        SSkelEdgeEvent() = default;
        ~SSkelEdgeEvent() {}

        float GetDistance() { return mDistance; }
        Vector2F GetIntersectPoint() { return mIntersect; }

        const SSkelVertexPtr &GetVertA() const { return mVertA; }
        const SSkelVertexPtr &GetVertB() const { return mVertB; }

        void Print()
        {
            KIRI_LOG_DEBUG("Edge Event: distance={0}, intersect=({1},{2}), between VertA={3},{4} and VertB={5},{6}",
                           mDistance, mIntersect.x, mIntersect.y,
                           mVertA->GetPoint().x, mVertA->GetPoint().y,
                           mVertB->GetPoint().x, mVertB->GetPoint().y);
        }

    private:
        float mDistance;
        Vector2F mIntersect;
        SSkelVertexPtr mVertA, mVertB;
    };
    typedef SharedPtr<SSkelEdgeEvent> SSkelEdgeEventPtr;

    struct SSkelEdgeEventCmpDistance
    {
        bool operator()(const SSkelEdgeEventPtr &e1, const SSkelEdgeEventPtr &e2)
        {
            return e1->GetDistance() < e2->GetDistance();
        }
    };
}

#endif