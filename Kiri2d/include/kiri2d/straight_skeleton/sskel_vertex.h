/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 10:58:21
 * @LastEditTime: 2021-07-23 15:15:13
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\straight_skeleton\sskel_vertex.h
 */

#ifndef _KIRI2D_SSKEL_VERTEX_H_
#define _KIRI2D_SSKEL_VERTEX_H_

#pragma once

#include <kiri_pch.h>
namespace KIRI2D::SSKEL
{
    class SSkelVertex
    {
    public:
        explicit SSkelVertex(Vector2F left,
                             Vector2F mid,
                             Vector2F right,
                             Vector4F dir = Vector4F(0.f))
            : mPoint(mid),
              mLeft(left),
              mRight(right),
              mDir(dir)
        {
            auto prev_dir = (mPoint - left).normalized() * -1.f;
            auto next_dir = (right - mid).normalized();

            auto bi_dir = Vector4F(prev_dir.x, prev_dir.y, next_dir.x, next_dir.y);
            if (dir == Vector4F(0.f))
                mDir = bi_dir;

            mIsReflex = (Vector2F(mDir.x, mDir.y).cross(Vector2F(mDir.z, mDir.w))) < 0.f;
            mIsValid = true;

            auto bisector_end = (Vector2F(bi_dir.x, bi_dir.y) + Vector2F(bi_dir.z, bi_dir.w)) * (mIsReflex ? -1.f : 1.f);
            mBisector = Vector4F(mPoint.x, mPoint.y, bisector_end.x, bisector_end.y);
        }

        SSkelVertex() = default;
        ~SSkelVertex() {}

        SharedPtr<SSkelVertex> prev;
        SharedPtr<SSkelVertex> next;

        bool GetIsReflex() { return mIsReflex; }
        bool GetIsValid() { return mIsValid; }
        Vector2F GetPoint() { return mPoint; }
        Vector2F GetLeftPoint() { return mLeft; }
        Vector2F GetRightPoint() { return mRight; }
        Vector4F GetBisector() { return mBisector; }

        void SetInValid() { mIsValid = false; }

        void Print()
        {
            KIRI_LOG_DEBUG("prev_dir=({0},{1}),next_dir=({2},{3}),mIsReflex={4},bisector_end=({5},{6})",
                           mDir.x, mDir.y, mDir.z, mDir.w, mIsReflex, mBisector.z, mBisector.w);
        }

    private:
        Vector2F mPoint, mLeft, mRight;

        // v2:prev edge dir, v2:next edge dir
        Vector4F mDir;

        // v2:start point, v2:end point
        Vector4F mBisector;

        bool mIsReflex, mIsValid;
    };
    typedef SharedPtr<SSkelVertex> SSkelVertexPtr;
}

#endif