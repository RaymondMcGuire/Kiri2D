/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 10:58:21
 * @LastEditTime: 2021-07-23 16:30:57
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\straight_skeleton\sskel_convex.h
 */

#ifndef _KIRI2D_SSKEL_CONVEX_H_
#define _KIRI2D_SSKEL_CONVEX_H_

#pragma once

#include <kiri2d/straight_skeleton/sskel_lav.h>

namespace KIRI2D::SSKEL
{
    class SSkelConvex
    {
    public:
        explicit SSkelConvex(Vec_Vec2F poly)
        {
            // TODO check poly is convex or not
            mLAV = std::make_shared<KIRI2D::SSKEL::SSkelLAV>(poly);
            mLAV->GenInitEvents();
            mLAV->HandleEvents();
        }

        SSkelConvex() = default;
        SSkelConvex(const SSkelConvex &) = delete;
        SSkelConvex &operator=(const SSkelConvex &) = delete;

        ~SSkelConvex() {}

        Vector<std::tuple<Vector2F, Vec_Vec2F>> GetSkeletons() const { return mLAV->GetSkeletons(); }

    private:
        SSkelLAVPtr mLAV;
    };
    typedef SharedPtr<SSkelConvex> SSkelConvexPtr;
}

#endif