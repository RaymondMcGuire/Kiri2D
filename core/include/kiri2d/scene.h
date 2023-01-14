/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-03-27 01:28:37
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-14 14:56:04
 * @FilePath: \Kiri2D\core\include\kiri2d\scene.h
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _KIRI2D_SCENE_H_
#define _KIRI2D_SCENE_H_

#pragma once

#include <kiri2d/camera/camera.h>
#include <kiri2d/sdf/sdf2d.h>
#include <kiri2d/data/shape_struct.h>

namespace KIRI2D
{

    class KiriScene2D
    {
    public:
        KiriScene2D()
        {
            KiriScene2D(700, 700);
        }

        KiriScene2D(
            size_t windowWidth,
            size_t windowHeight)
            : mWindowWidth(windowWidth),
              mWindowHeight(windowHeight),
              mViewWidth(windowWidth)
        {
            float view_height = mWindowHeight * mViewWidth / mWindowWidth;
            auto look_at = Vector2F(mViewWidth / 2.f, view_height / 2.f);
            auto window_size = Vector2F(mViewWidth / mWindowWidth, view_height / mWindowHeight);
            auto window_center = Vector2F(mViewWidth / (2.f * window_size.x), view_height / (2.f * window_size.y));
            mCamera = std::make_shared<KiriCamera2D>(
                Camera2DProperty(
                    look_at,
                    window_size,
                    window_center));
        }

        void AddObject(KiriSDF2D object);
        void AddLine(KiriLine2 line);
        void AddLines(std::vector<KiriLine2> lines);
        void AddParticle(KiriPoint2 particle);
        void AddParticles(std::vector<KiriPoint2> particles);
        void AddRect(KiriRect2 rect);
        void AddRects(std::vector<KiriRect2> rects);
        void AddCircle(KiriCircle2 circle);
        void AddCircles(std::vector<KiriCircle2> circles);

        inline const auto GetSDFObjects() { return mSDFObjects; }
        inline const auto GetPoints() { return mPoints; }
        inline const auto GetLines() { return mLines; }
        inline const auto GetRects() { return mRects; }
        inline const auto GetCircles() { return mCircles; }

        inline const auto GetCamera() { return mCamera; }
        inline const auto GetWindowWidth() { return mWindowWidth; }
        inline const auto GetWindowHeight() { return mWindowHeight; }

        void Clear()
        {
            mSDFObjects.clear();
            mPoints.clear();
            mLines.clear();
            mRects.clear();
            mCircles.clear();
        }

        ~KiriScene2D() {}

    private:
        std::vector<KiriSDF2D> mSDFObjects;
        std::vector<KiriPoint2> mPoints;
        std::vector<KiriLine2> mLines;
        std::vector<KiriRect2> mRects;
        std::vector<KiriCircle2> mCircles;

        KiriCamera2DPtr mCamera;

        size_t mViewWidth;
        size_t mWindowWidth;
        size_t mWindowHeight;
    };
    typedef SharedPtr<KiriScene2D> KiriScene2DPtr;
}
#endif