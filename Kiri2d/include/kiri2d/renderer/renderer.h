/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:26:52
 * @LastEditTime: 2021-02-22 18:34:57
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\renderer\renderer.h
 */
#ifndef _KIRI2D_RENDERER_H_
#define _KIRI2D_RENDERER_H_

#pragma once

#include <kiri2d/scene.h>
#include <opencv2/opencv.hpp>

namespace KIRI2D
{
    class KiriRenderer2D
    {
    public:
        KiriRenderer2D()
        {
            KiriRenderer2D(700, 700);
        }

        KiriRenderer2D(
            Int windowWidth,
            Int windowHeight)
            : mWindowWidth(windowWidth),
              mWindowHeight(windowHeight)
        {
            mScene = std::make_shared<KiriScene2D>(mWindowWidth, mWindowHeight);
            mCanvas = cv::Mat::zeros(mWindowHeight, mWindowWidth, CV_8UC3);
            this->ClearCanvas();
        }

        void ClearCanvas();
        void DrawCanvas();

        ~KiriRenderer2D() noexcept {}

    private:
        cv::Mat mCanvas;

        KiriScene2DPtr mScene;
        Int mWindowWidth, mWindowHeight;
    };
    typedef SharedPtr<KiriRenderer2D> KiriRenderer2DPtr;
}
#endif