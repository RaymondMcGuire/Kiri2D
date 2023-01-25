/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-03-27 01:28:37
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-25 23:12:27
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
    template <class RealType>
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
            RealType view_height = mWindowHeight * mViewWidth / mWindowWidth;
            auto look_at = VectorX<2, RealType>(mViewWidth / 2.f, view_height / 2.f);
            auto window_size = VectorX<2, RealType>(mViewWidth / mWindowWidth, view_height / mWindowHeight);
            auto window_center = VectorX<2, RealType>(mViewWidth / (2.f * window_size.x), view_height / (2.f * window_size.y));
            mCamera = std::make_shared<KiriCamera2D<RealType>>(
                Camera2DProperty<RealType>(
                    look_at,
                    window_size,
                    window_center));
        }

        void AddObject(KiriSDF2D object)
        {
            mSDFObjects.emplace_back(object);
        }

        void AddParticle(KiriPoint2<RealType> particle)
        {
            mPoints.emplace_back(particle);
        }

        void AddParticles(std::vector<KiriPoint2<RealType>> particles)
        {
            for (size_t i = 0; i < particles.size(); i++)
            {
                mPoints.emplace_back(particles[i]);
            }
        }

        void AddCircle(KiriCircle2<RealType> circle)
        {
            mCircles.emplace_back(circle);
        }

        void AddCircles(std::vector<KiriCircle2<RealType>> circles)
        {
            for (size_t i = 0; i < circles.size(); i++)
            {
                mCircles.emplace_back(circles[i]);
            }
        }

        void AddLine(KiriLine2<RealType> line)
        {
            mLines.emplace_back(line);
        }

        void AddLines(std::vector<KiriLine2<RealType>> lines)
        {
            for (size_t i = 0; i < lines.size(); i++)
            {
                mLines.emplace_back(lines[i]);
            }
        }

        void AddRect(KiriRect2<RealType> rect)
        {
            mRects.emplace_back(rect);
        }

        void AddRects(std::vector<KiriRect2<RealType>> rects)
        {
            for (size_t i = 0; i < rects.size(); i++)
            {
                mRects.emplace_back(rects[i]);
            }
        }

        // inline const auto GetSDFObjects() { return mSDFObjects; }
        inline const auto GetPoints() { return mPoints; }
        inline const auto GetLines() { return mLines; }
        inline const auto GetRects() { return mRects; }
        inline const auto GetCircles() { return mCircles; }

        inline const auto GetCamera() { return mCamera; }
        inline const auto GetWindowWidth() { return mWindowWidth; }
        inline const auto GetWindowHeight() { return mWindowHeight; }

        void Clear()
        {
            // mSDFObjects.clear();
            mPoints.clear();
            mLines.clear();
            mRects.clear();
            mCircles.clear();
        }

        ~KiriScene2D() {}

    private:
        // std::vector<KiriSDF2D> mSDFObjects;
        std::vector<KiriPoint2<RealType>> mPoints;
        std::vector<KiriLine2<RealType>> mLines;
        std::vector<KiriRect2<RealType>> mRects;
        std::vector<KiriCircle2<RealType>> mCircles;

        SharedPtr<KiriCamera2D<RealType>> mCamera;

        size_t mViewWidth;
        size_t mWindowWidth;
        size_t mWindowHeight;
    };
    typedef SharedPtr<KiriScene2D<float>> KiriScene2DFPtr;
    typedef SharedPtr<KiriScene2D<double>> KiriScene2DDPtr;
}
#endif