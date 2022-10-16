/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-03-27 01:28:37
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-16 14:02:39
 * @FilePath: \Kiri2D\core\include\kiri2d\scene.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
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

        void addObject(KiriSDF2D object);
        void addLine(KiriLine2 line);
        void addLines(std::vector<KiriLine2> lines);
        void addParticle(KiriPoint2 particle);
        void addParticles(std::vector<KiriPoint2> particles);
        void addRect(KiriRect2 rect);
        void addRects(std::vector<KiriRect2> rects);
        void addCircle(KiriCircle2 circle);
        void addCircles(std::vector<KiriCircle2> circles);

        inline const auto sdfObjects() { return mSDFObjects; }
        inline const auto points() { return mPoints; }
        inline const auto lines() { return mLines; }
        inline const auto rects() { return mRects; }
        inline const auto circles() { return mCircles; }

        inline const auto camera() { return mCamera; }
        inline const auto windowWidth() { return mWindowWidth; }
        inline const auto windowHeight() { return mWindowHeight; }

        void clear()
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