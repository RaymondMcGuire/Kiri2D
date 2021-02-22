/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 16:35:26
 * @LastEditTime: 2021-02-22 16:56:29
 * @LastEditors: Xu.WANG
 * @Description: Reference-> Computing Distance, Erin Catto, Blizzard Entertainment, GDC2010
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\sdf\kiri_sdf_func.h
 */

#ifndef _KIRI2D_SDF_FUNC_H_
#define _KIRI2D_SDF_FUNC_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D
{

    /*** 
     * @description: Closest distence(2D) between point p to line segment (ls,le)
     * @param {*}
     * @return {*}
     */
    inline static float P2LS2D(Vector2F p, Vector2F ls, Vector2F le)
    {
        auto v = p - ls, u = le - ls;
        float dis = fmaxf(fminf(v.dot(u) / u.dot(u), 1.f), 0.f);
        auto g = ls + dis * u;
        return (p - g).length();
    }
}
#endif