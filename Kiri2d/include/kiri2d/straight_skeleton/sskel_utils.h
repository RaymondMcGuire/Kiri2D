/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 10:58:21
 * @LastEditTime: 2021-07-22 21:56:37
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\straight_skeleton\sskel_utils.h
 */

#ifndef _KIRI2D_SSKEL_UTILS_H_
#define _KIRI2D_SSKEL_UTILS_H_

#pragma once

#include <kiri_pch.h>
namespace KIRI2D::SSKEL
{

    /*** 
     * @description: 
     * @param {Vector2F} p1
     * @param {Vector2F} p2
     * @param {Vector2F} q
     * @return {float} minimum distance between line segment p and point q
     * @reference: https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     */
    static float _distance_point2_line2(Vector2F p1, Vector2F p2, Vector2F q)
    {
        const float l2 = p1.distanceSquaredTo(p2);
        if (l2 == 0.f)
            return q.distanceTo(p1);

        const float t = std::clamp(((q - p1).dot(p2 - p1)) / l2, 0.f, 1.f);
        const Vector2F projection = p1 + t * (p2 - p1);
        return q.distanceTo(projection);
    }

    static Vector3F _intersect_line2_line2(Vector2F p, Vector2F p_dir, Vector2F q, Vector2F q_dir)
    {
        auto d = (p_dir).cross(q_dir);
        if (d == 0.f)
            return Vector3F(0.f);

        auto dx = p.x - q.x;
        auto dy = p.y - q.y;

        auto ua = (q_dir.x * dy - q_dir.y * dx) / d;
        auto ub = (p_dir.x * dy - p_dir.y * dx) / d;

        return Vector3F(p.x + ua * p_dir.x,
                        p.y + ua * p_dir.y,
                        1.f);
    }
}

#endif