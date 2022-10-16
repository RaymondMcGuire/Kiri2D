/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-10-16 14:56:24
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-16 15:24:24
 * @FilePath: \Kiri2D\core\include\kiri2d\sdf\sdf2d.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
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

        void append(Vector2F p, Vector2F v = Vector2F(0.f));
        const Int findRegion(Vector2F p);
        const SDF2DInfo calcClosestDistance(Vector2F p);

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

        void reCalcBBox(Vector2F p);
    };
    typedef SharedPtr<KiriSDF2D> KiriSDF2DPtr;
}
#endif