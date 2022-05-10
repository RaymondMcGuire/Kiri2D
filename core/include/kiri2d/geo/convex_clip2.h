/***
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:49:01
 * @LastEditTime: 2021-05-26 17:45:40
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\geo\convex_clip2.h
 */

#ifndef _KIRI_CONVEX_CLIP2_H_
#define _KIRI_CONVEX_CLIP2_H_

#pragma once

#include <kiri2d/geo/vector2_list.h>

namespace KIRI
{
    /***
     * @description: Flag whether p is inside q or q is inside p
     * @param {*}
     * @return {*}
     */
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

        const KiriVector2ListPtr &GetIntersectionList() const { return mIntersectionList; }

        bool ComputeConvexPolygonIntersection(const KiriVector2ListPtr &a, const KiriVector2ListPtr &b);

    private:
        KiriVector2ListPtr mIntersectionList;

        /***
         * @description: Checks if vector2 v3 is between v1 and v2 or not
         * @param {const Vector2F} &v1
         * @param {const Vector2F} &v2
         * @param {const Vector2F} &v3
         * @return {*}
         */
        bool IsBetween(const Vector2F &v1, const Vector2F &v2, const Vector2F &v3);

        /***
         * @description: Checks whether list is convex and counterclockwise oriented
         * @param {const KiriVector2ListPtr} &list
         * @return {*}
         */
        bool IsConvex(const KiriVector2ListPtr &list);

        /***
         * @description: Signed area of triangle2
         * @param {const Vector2F} &v1
         * @param {const Vector2F} &v2
         * @param {const Vector2F} &v3
         * @return {*}
         */
        Int GetSignedAreasOfTriangle2(const Vector2F &v1, const Vector2F &v2, const Vector2F &v3);

        /***
         * @description: Computes the intersection of the lines between a1a2 and b1b2
         * @param {const Vector2F} &a1
         * @param {const Vector2F} &a2
         * @param {const Vector2F} &b1
         * @param {const Vector2F} &b2
         * @return {*}
         */
        LineSegment ComputeLineIntersection(const Vector2F &a1, const Vector2F &a2, const Vector2F &b1, const Vector2F &b2);

        /***
         * @description: Computes the intersection of convex hull 2d
         * @param {const KiriVector2ListPtr} p counterclockwise oriented
         * @param {KiriVector2ListPtr} q counterclockwise oriented
         * @return {*}
         */
        void ConvexIntersection(const KiriVector2ListPtr &p, const KiriVector2ListPtr &q);
    };
    typedef SharedPtr<KiriConvexClip2> KiriConvexClip2Ptr;
} // namespace KIRI

#endif /* _KIRI_CONVEX_CLIP2_H_ */