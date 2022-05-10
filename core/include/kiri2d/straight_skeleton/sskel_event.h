/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 10:58:21
 * @LastEditTime: 2021-07-24 16:50:46
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
    class SSkelEvent
    {
    public:
        explicit SSkelEvent(
            float distance,
            Vector2F intersect)
            : mDistance(distance),
              mIntersect(intersect)
        {
        }

        SSkelEvent() = default;
        ~SSkelEvent() {}

        float GetDistance() { return mDistance; }
        Vector2F GetIntersectPoint() { return mIntersect; }

        virtual void Print() = 0;

    protected:
        float mDistance;
        Vector2F mIntersect;
    };

    class SSkelEdgeEvent : public SSkelEvent
    {
    public:
        explicit SSkelEdgeEvent(
            float distance,
            Vector2F intersect,
            const SSkelVertexPtr &va,
            const SSkelVertexPtr &vb)
            : SSkelEvent(distance, intersect),
              mVertA(va),
              mVertB(vb)
        {
        }

        SSkelEdgeEvent() = default;
        ~SSkelEdgeEvent() {}

        const SSkelVertexPtr &GetVertA() const { return mVertA; }
        const SSkelVertexPtr &GetVertB() const { return mVertB; }

        virtual void Print() override;

    private:
        SSkelVertexPtr mVertA, mVertB;
    };

    class SSkelSplitEvent : public SSkelEvent
    {
    public:
        explicit SSkelSplitEvent(
            float distance,
            Vector2F intersect,
            const SSkelVertexPtr &v,
            Vector4F oedge)
            : SSkelEvent(distance, intersect),
              mVert(v),
              mOppositeEdge(oedge)
        {
        }

        SSkelSplitEvent() = default;
        ~SSkelSplitEvent() {}

        const SSkelVertexPtr &GetVert() const { return mVert; }
        Vector4F GetOppositeEdge() { return mOppositeEdge; }

        virtual void Print() override;

    private:
        SSkelVertexPtr mVert;
        Vector4F mOppositeEdge;
    };

    typedef SharedPtr<SSkelEvent> SSkelEventPtr;
    typedef SharedPtr<SSkelEdgeEvent> SSkelEdgeEventPtr;
    typedef SharedPtr<SSkelSplitEvent> SSkelSplitEventPtr;

    struct SSkelEventCmpDistance
    {
        bool operator()(const SSkelEventPtr &e1, const SSkelEventPtr &e2)
        {
            return e1->GetDistance() > e2->GetDistance();
        }
    };
}

#endif