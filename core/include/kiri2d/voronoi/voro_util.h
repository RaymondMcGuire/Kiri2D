/***
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-06-25 01:30:34
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_util.h
 */

#ifndef _KIRI_VORO_UTIL_H_
#define _KIRI_VORO_UTIL_H_

#pragma once

#include <kiri_pch.h>
namespace KIRI
{

    /***
     * @description:
     * @param {Vector2F} v
     * @param {Vector2F} w
     * @param {Vector2F} p
     * @return {float} minimum distance between line segment vw and point p
     * @reference: https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     */
    static float minDis2LineSegment2(Vector2F v, Vector2F w, Vector2F p)
    {
        const float l2 = v.distanceSquaredTo(w);
        if (l2 == 0.f)
            return p.distanceTo(v);

        const float t = std::clamp(((p - v).dot(w - v)) / l2, 0.f, 1.f);
        const Vector2F projection = v + t * (w - v);
        return p.distanceTo(projection);
    }

    /***
     * @description:
     * @param {Vector2F} p1: start
     * @param {Vector2F} p2: mid
     * @param {Vector2F} p3: end
     * @param {bool} cw: clockwise
     * @return {float} angle
     */
    static float AngleBetween2Edges2(Vector2F p1, Vector2F p2, Vector2F p3, bool cw)
    {
        auto x1 = p1.x;
        auto y1 = p1.y;
        auto x2 = p2.x;
        auto y2 = p2.y;
        auto x3 = p3.x;
        auto y3 = p3.y;
        auto x12 = x2 - x1;
        auto y12 = y2 - y1;
        auto x23 = x3 - x2;
        auto y23 = y3 - y2;
        auto d12 = x12 * x12 + y12 * y12;
        auto d23 = x23 * x23 + y23 * y23;
        auto d13 = (x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1);
        auto a = std::acos((d12 + d23 - d13) / (2 * std::sqrt(d12) * std::sqrt(d23))); // angle between 0 and pi radians
        auto c = x12 * y23 - x23 * y12;                                                // cross product of two edges: e1 x e2
        // cross product will be negative if interior angle is greater than pi when
        // the edges are ordered clockwise; positive when ordered counterclockwise
        if ((c > 0 && cw) || (c < 0 && !cw))
            a = 2 * KIRI_PI<float>() - a; // ensuring angle is that of the interior angle

        // auto e1 = p2 - p1;
        // auto e2 = p3 - p2;
        // auto c = e1.cross(e2);

        // //KIRI_LOG_DEBUG(" e1={0},{1}, e2={2},{3}, c={4}", e1.x, e1.y, e2.x, e2.y, c);
        // auto e3 = p1 - p2;
        // auto a = Vector2F::angle(e2, e3);

        // if ((c > 0.f && cw) || (c < 0.f && !cw))
        //     a = 2 * KIRI_PI<float>() - a; // ensuring angle is that of the interior angle

        return a;
    }

    /***
     * @description: shrinking direction for the vertex connecting the two edges
     * @idea: compute angle between the two lines and pick a vector that splits the angle evenly (bisector).
     * @reference: https://github.com/savannaos/straight-skeletons/blob/master/polygon.js
     * @param {Vector2F} p1
     * @param {Vector2F} p2
     * @param {Vector2F} p3
     * @param {bool} cw
     * @return {*}
     */
    static Vector2F DirectionBetween2Edges2(Vector2F p1, Vector2F p2, Vector2F p3, bool cw)
    {
        auto a = AngleBetween2Edges2(p1, p2, p3, cw) / 2.f;

        auto x1 = p1.x;
        auto y1 = p1.y;
        if (!cw)
        { // determine new angle from second edge if edges are ordered counterclockwise
            x1 = p3.x;
            y1 = p3.y;
        }
        auto x2 = p2.x;
        auto y2 = p2.y;
        auto dx = x1 - x2;
        auto dy = y1 - y2;
        auto sy = KIRI_SGN<float>(dy);
        if (dx == 0.f)
            a = a + sy * (KIRI_PI<float>() / 2.f);
        else if (dx > 0.f)
            a = a + sy * std::atanf(std::abs(dy) / std::abs(dx));
        else
            a = a + KIRI_PI<float>() - sy * std::atanf(std::abs(dy) / std::abs(dx));
        auto x = std::cos(a);
        auto y = std::sin(a);
        auto d = std::sqrt(x * x + y * y);
        return Vector2F(x / d * -1.f, y / d * -1.f);
    }

    /***
     * @description: check clockwise
     * @param {Vector2F} p1
     * @param {Vector2F} p2
     * @param {Vector2F} p3
     * @return {Int} 0=co-linear, 1=counter clockwise, -1=clockwise
     */
    static Int CheckClockwise2(Vector2F p1, Vector2F p2, Vector2F p3)
    {
        auto res = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
        if (res > 0.f)
            return 1;
        else if (res == 0.f)
            return 0;
        else
            return -1;
    }

    /***
     * @description: compute the intersection point for line p and q
     * @param {Vector2F} p1
     * @param {Vector2F} p2
     * @param {Vector2F} q1
     * @param {Vector2F} q2
     * @return {Vector3F} Vector2F intersection point 2d, float is intersection(0/1)
     */
    static Vector3F IntersectionPoint2(Vector2F p1, Vector2F p2, Vector2F q1, Vector2F q2)
    {

        if ((CheckClockwise2(p1, p2, q1) == CheckClockwise2(p1, p2, q2)) &&
            (CheckClockwise2(q1, q2, p1) == CheckClockwise2(q1, q2, p2)))
            return Vector3F(0.f);

        // compute the intersection
        auto m1 = (p1.y - p2.y) / (p1.x - p2.x);
        auto m2 = (q1.y - q2.y) / (q1.x - q2.x);
        auto b1 = p1.y - m1 * p1.x;
        auto b2 = q1.y - m2 * q1.x;
        auto x = (b2 - b1) / (m1 - m2);
        auto y = m1 * x + b1;
        return Vector3F(x, y, 1.f);
    }

    static bool IsApproxFloat(float a, float b, float eps)
    {
        if (std::abs(a - b) < eps)
            return true;
        return false;
    }

    /***
     * @description:
     * @param {Vector2F} p1
     * @param {Vector2F} p2
     * @param {float} eps
     * @return {Vector3F} Vector2F point, float is IsApproxFloat or not(0/1)
     */
    static Vector3F IsApproxVec2(Vector2F p1, Vector2F p2, float eps)
    {
        if (IsApproxFloat(p1.x, p2.x, eps) && IsApproxFloat(p1.y, p2.y, eps))
            return Vector3F((p1 + p2) / 2.f, 1.f);

        return Vector3F(0.f);
    }

    static bool IsApproxVec4(Vector4F e1, Vector4F e2, float eps)
    {
        if ((IsApproxFloat(e1.x, e2.x, eps) && IsApproxFloat(e1.y, e2.y, eps) &&
             IsApproxFloat(e1.z, e2.z, eps) && IsApproxFloat(e1.w, e2.w, eps)) ||
            (IsApproxFloat(e1.x, e2.z, eps) && IsApproxFloat(e1.y, e2.w, eps) &&
             IsApproxFloat(e1.z, e2.x, eps) && IsApproxFloat(e1.w, e2.y, eps)))
            return true;
        return false;
    }
}
#endif