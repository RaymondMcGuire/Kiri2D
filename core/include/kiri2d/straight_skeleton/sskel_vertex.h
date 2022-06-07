/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-10-05 00:08:58
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 17:22:30
 * @FilePath: \core\include\kiri2d\straight_skeleton\sskel_vertex.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
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
        explicit SSkelVertex(UInt lavId,
                             Vector4F leftEdge,
                             Vector2F point,
                             Vector4F rightEdge,
                             Vector4F dir = Vector4F(0.f))
            : mLAVId(lavId),
              mPoint(point),
              mLeftEdge(leftEdge),
              mRightEdge(rightEdge),
              mDir(dir)
        {
            auto prev_dir = (Vector2F(leftEdge.z, leftEdge.w) - Vector2F(leftEdge.x, leftEdge.y)).normalized() * -1.f;
            auto next_dir = (Vector2F(rightEdge.z, rightEdge.w) - Vector2F(rightEdge.x, rightEdge.y)).normalized();

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
        WeakPtr<SSkelVertex> next;

        UInt GetLAVId() { return mLAVId; }
        bool GetIsReflex() { return mIsReflex; }
        bool GetIsValid() { return mIsValid; }
        Vector2F GetPoint() { return mPoint; }
        Vector4F GetLeftEdge() { return mLeftEdge; }
        Vector4F GetRightEdge() { return mRightEdge; }
        Vector4F GetBisector() { return mBisector; }
        Vector4F GetDir() { return mDir; }

        void SetLAVId(UInt id) { mLAVId = id; }
        void SetInValid() { mIsValid = false; }

        void print()
        {
            // KIRI_LOG_DEBUG("left edge=({0},{1})--({2},{3}),right edge=({4},{5})--({6},{7}),mIsReflex={8},bisector_end=({9},{10}),point={11},{12}",
            //                mLeftEdge.x, mLeftEdge.y, mLeftEdge.z, mLeftEdge.w,
            //                mRightEdge.x, mRightEdge.y, mRightEdge.z, mRightEdge.w,
            //                mIsReflex, mBisector.z, mBisector.w, mPoint.x, mPoint.y);

            KIRI_LOG_DEBUG("point={0},{1}", mPoint.x, mPoint.y);
        }

    private:
        UInt mLAVId;

        Vector2F mPoint;
        Vector4F mLeftEdge, mRightEdge;

        // v2:prev edge dir, v2:next edge dir
        Vector4F mDir;

        // v2:start point, v2:end point
        Vector4F mBisector;

        bool mIsReflex, mIsValid;
    };
    typedef SharedPtr<SSkelVertex> SSkelVertexPtr;
}

#endif