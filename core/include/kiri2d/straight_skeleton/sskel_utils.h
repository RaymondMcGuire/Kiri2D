/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-07-22 17:36:32
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 17:22:23
 * @FilePath: \core\include\kiri2d\straight_skeleton\sskel_utils.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
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
    static float _distance_point2_lineseg2(Vector2F p1, Vector2F p2, Vector2F q)
    {
        const float l2 = p1.distanceSquaredTo(p2);
        if (l2 == 0.f)
            return q.distanceTo(p1);

        const float t = std::clamp(((q - p1).dot(p2 - p1)) / l2, 0.f, 1.f);

        const Vector2F projection = p1 + t * (p2 - p1);
        return q.distanceTo(projection);
    }

    // TODO line lineseg ray
    static float _distance_point2_line2(Vector2F p1, Vector2F p2, Vector2F q)
    {
        const float l2 = p1.distanceSquaredTo(p2);
        if (l2 == 0.f)
            return q.distanceTo(p1);

        const float t = ((q - p1).dot(p2 - p1)) / l2;

        const Vector2F projection = p1 + t * (p2 - p1);
        return q.distanceTo(projection);
    }

    static Vector3F _intersect_line2_line2(Vector2F p, Vector2F p_dir, Vector2F q, Vector2F q_dir)
    {
        auto d = (q_dir).cross(p_dir);

        if (d == 0.f)
            return Vector3F(0.f);

        auto dx = q.x - p.x;
        auto dy = q.y - p.y;

        auto ua = (p_dir.x * dy - p_dir.y * dx) / d;
        auto ub = (q_dir.x * dy - q_dir.y * dx) / d;

        return Vector3F(q.x + ua * q_dir.x,
                        q.y + ua * q_dir.y,
                        1.f);
    }

    static Vector3F _intersect_line2_ray2(Vector2F p, Vector2F p_dir, Vector2F q, Vector2F q_dir)
    {
        auto d = (q_dir).cross(p_dir);

        if (d == 0.f)
            return Vector3F(0.f);

        auto dx = q.x - p.x;
        auto dy = q.y - p.y;

        auto ua = (p_dir.x * dy - p_dir.y * dx) / d;
        if (ua < 0.f)
            return Vector3F(0.f);

        auto ub = (q_dir.x * dy - q_dir.y * dx) / d;

        return Vector3F(q.x + ua * q_dir.x,
                        q.y + ua * q_dir.y,
                        1.f);
    }

    static Vector3F _intersect_ray2_ray2(Vector2F p, Vector2F p_dir, Vector2F q, Vector2F q_dir)
    {
        auto d = (q_dir).cross(p_dir);

        if (d == 0.f)
            return Vector3F(0.f);

        auto dx = q.x - p.x;
        auto dy = q.y - p.y;

        auto ua = (p_dir.x * dy - p_dir.y * dx) / d;
        if (ua < 0.f)
            return Vector3F(0.f);

        auto ub = (q_dir.x * dy - q_dir.y * dx) / d;

        if (ub < 0.f)
            return Vector3F(0.f);

        return Vector3F(q.x + ua * q_dir.x,
                        q.y + ua * q_dir.y,
                        1.f);
    }

    static bool _approximately_equals_point2(Vector2F p1, Vector2F p2)
    {
        return (p1 == p2) || ((p1 - p2).length() <= std::max(p1.length(), p2.length()) * 0.001f);
    }

}

#endif