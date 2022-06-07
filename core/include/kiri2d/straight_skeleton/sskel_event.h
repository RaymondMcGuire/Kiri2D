/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-07-27 19:42:45
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 17:18:46
 * @FilePath: \core\include\kiri2d\straight_skeleton\sskel_event.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
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

        float dist() { return mDistance; }
        Vector2F intersectPoint() { return mIntersect; }

        virtual void print() = 0;

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

        const SSkelVertexPtr &vertA() const { return mVertA; }
        const SSkelVertexPtr &vertB() const { return mVertB; }

        virtual void print() override;

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

        const SSkelVertexPtr &vert() const { return mVert; }
        Vector4F oppositeEdge() { return mOppositeEdge; }

        virtual void print() override;

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
            return e1->dist() > e2->dist();
        }
    };
}

#endif