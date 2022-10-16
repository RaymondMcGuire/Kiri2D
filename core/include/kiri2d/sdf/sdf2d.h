/***
 * @Author: Xu.WANG
 * @Date: 2021-09-26 16:12:57
 * @LastEditTime: 2022-02-20 13:27:25
 * @LastEditors: Xu.WANG
 * @Description:
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
            float closestDistance,
            Int lineSegStartIdx,
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
            bool moveable,
            Vector2F offset)
            : mName(name),
              mColor(color),
              bMoveable(moveable),
              mOffset(offset)
        {
            mBBoxMin = Vector2F(Huge<float>());
            mBBoxMax = Vector2F(Tiny<float>());
        }

        ~KiriSDF2D() {}

        const Vec_Vec2F points() const { return mPoints; }

        void Append(Vector2F p, Vector2F v = Vector2F(0.f));
        const Int FindRegion(Vector2F p);
        const SDF2DInfo CalcClosestDistance(Vector2F p);

        void setOffset(const Vector2F &offset) { mOffset = offset; }
        Vector2F offset() { return mOffset; }

        void clear()
        {
            mBBoxMin = Vector2F(Huge<float>());
            mBBoxMax = Vector2F(Tiny<float>());

            mOffset = Vector2F(0.f);
            mName = "sdf_poly2d",
            mColor = Vector3F(0.f),
            bMoveable = false,

            mPoints.clear();
            mVelocities.clear();
        }

    protected:
        String mName;
        bool bMoveable;

        Vector2F mBBoxMin, mBBoxMax;
        Vector3F mColor;
        Vector2F mOffset;

        Vec_Vec2F mPoints;
        Vec_Vec2F mVelocities;

        void ReCalcBBox(Vector2F p);
    };
    typedef SharedPtr<KiriSDF2D> KiriSDF2DPtr;
}
#endif