/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-06-08 16:11:40
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
    float MinDis2LineSegment2(Vector2F v, Vector2F w, Vector2F p)
    {
        const float l2 = v.distanceSquaredTo(w);
        if (l2 == 0.f)
            return p.distanceTo(v);

        const float t = std::clamp(((p - v).dot(w - v)) / l2, 0.f, 1.f);
        const Vector2F projection = v + t * (w - v);
        return p.distanceTo(projection);
    }
}
#endif