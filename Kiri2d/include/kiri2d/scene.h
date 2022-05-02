/***
 * @Author: Xu.WANG
 * @Date: 2021-02-22 14:13:45
 * @LastEditTime: 2021-05-29 22:35:09
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\scene.h
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
            float viewHeight = mWindowHeight * mViewWidth / mWindowWidth;
            auto lookAt = Vector2F(mViewWidth / 2.f, viewHeight / 2.f);
            auto windowCellSize = Vector2F(mViewWidth / mWindowWidth, viewHeight / mWindowHeight);
            auto windowCenter = Vector2F(mViewWidth / (2.f * windowCellSize.x), viewHeight / (2.f * windowCellSize.y));
            mCamera = std::make_shared<KiriCamera2D>(
                Camera2DProperty(
                    lookAt,
                    windowCellSize,
                    windowCenter));
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