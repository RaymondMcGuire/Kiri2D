/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 14:13:45
 * @LastEditTime: 2021-02-22 18:37:47
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\scene.h
 */

#ifndef _KIRI2D_SCENE_H_
#define _KIRI2D_SCENE_H_

#pragma once

#include <kiri2d/camera/camera.h>
#include <kiri2d/sdf/sdf2d.h>

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
            Int windowWidth,
            Int windowHeight)
            : mWindowWidth(windowWidth),
              mWindowHeight(windowHeight)
        {
            float viewWidth = 5.f;
            float viewHeight = mWindowHeight * viewWidth / mWindowWidth;
            auto lookAt = Vector2F(viewWidth / 2.f, viewHeight / 2.f);
            auto windowCellSize = Vector2F(viewWidth / mWindowWidth, viewHeight / mWindowHeight);
            auto windowCenter = Vector2F(viewWidth / (2.f * windowCellSize.x), viewHeight / (2.f * windowCellSize.y));
            mCamera = std::make_shared<KiriCamera2D>(
                Camera2DProperty(
                    lookAt,
                    windowCellSize,
                    windowCenter));
        }

        void AddObject(KiriSDF2D object);
        inline constexpr auto GetSDFObjects() constexpr { return mSDFObjects; }

        ~KiriScene2D() noexcept {}

    private:
        std::vector<KiriSDF2D> mSDFObjects;

        KiriCamera2DPtr mCamera;

        Int mWindowWidth;
        Int mWindowHeight;
    };
    typedef SharedPtr<KiriScene2D> KiriScene2DPtr;
}
#endif