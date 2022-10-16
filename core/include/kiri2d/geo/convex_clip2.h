/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-10-16 15:11:21
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-16 15:19:38
 * @FilePath: \Kiri2D\core\include\kiri2d\geo\convex_clip2.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _KIRI_CONVEX_CLIP2_H_
#define _KIRI_CONVEX_CLIP2_H_

#pragma once

#include <kiri2d/geo/vector2_list.h>

namespace KIRI
{
    enum InsideFlag
    {
        IF_PINSIDE,
        IF_QINSIDE,
        IF_UNKNOWN
    };

    struct LineSegment
    {
        Vector2F start;
        Vector2F end;
        char code;

        LineSegment(Vector2F s, Vector2F e, char c)
            : start(s), end(e), code(c) {}
    };

    class KiriConvexClip2
    {
    public:
        KiriConvexClip2()
        {
            mIntersectionList = std::make_shared<KiriVector2List>();
        }

        ~KiriConvexClip2() {}

        void reset() { mIntersectionList->removeAll(); }

        const KiriVector2ListPtr &intersectionList() const { return mIntersectionList; }

        bool computeConvexPolygonIntersection(const KiriVector2ListPtr &a, const KiriVector2ListPtr &b);

    private:
        KiriVector2ListPtr mIntersectionList;

        bool isBetween(const Vector2F &v1, const Vector2F &v2, const Vector2F &v3);
        bool isConvex(const KiriVector2ListPtr &list);
        Int signedAreasOfTriangle2(const Vector2F &v1, const Vector2F &v2, const Vector2F &v3);
        LineSegment computeLineIntersection(const Vector2F &a1, const Vector2F &a2, const Vector2F &b1, const Vector2F &b2);
        void convexIntersection(const KiriVector2ListPtr &p, const KiriVector2ListPtr &q);
    };
    typedef SharedPtr<KiriConvexClip2> KiriConvexClip2Ptr;
} // namespace KIRI

#endif /* _KIRI_CONVEX_CLIP2_H_ */