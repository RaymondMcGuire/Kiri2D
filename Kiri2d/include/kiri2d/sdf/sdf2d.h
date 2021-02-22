/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 14:40:10
 * @LastEditTime: 2021-02-22 18:20:01
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\sdf\sdf2d.h
 */

#ifndef _KIRI2D_SDF2D_H_
#define _KIRI2D_SDF2D_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D
{
    struct SDF2DInfo
    {
        float ClosestDistance;
        Int LineSegStartIdx;
        Int LineSegEndIdx;

        // Constructor
        SDF2DInfo(
            float closestDistance;
            Int lineSegStartIdx;
            Int lineSegEndIdx)
            : ClosestDistance(closestDistance),
              LineSegStartIdx(lineSegStartIdx),
              LineSegEndIdx(lineSegEndIdx)
        {
        }
    };

    class KiriSDF2D
    {
    public:
        KiriSDF2D(
            String name,
            Vector3F color,
            bool moveable)
            : mName(name),
              mColor(color),
              bMoveable(moveable)
        {
            mBBoxMin = Vector2F(Huge<float>());
            mBBoxMax = Vector2F(Tiny<float>());
        }

        ~KiriSDF2D() noexcept {}

        void Append(Vector2F p, Vector2F v = Vector2F::setZero());
        const Int FindRegion(Vector2F p);
        const SDF2DInfo CalcClosestDistance(Vector2F p);

    protected:
        String mName;
        bool bMoveable;

        Vector2F mBBoxMin, mBBoxMax;
        Vector3F mColor;

        Vec_Vec2F mPoints;
        Vec_Vec2F mVelocities;

        void ReCalcBBox(Vector2F p);
    };
    typedef SharedPtr<KiriSDF2D> KiriSDF2DPtr;
}
#endif