/***
 * @Author: Xu.WANG
 * @Date: 2021-02-23 00:18:39
 * @LastEditTime: 2021-11-23 18:29:38
 * @LastEditors: Xu.WANG
 * @Description:
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
            KiriRenderer2D(std::make_shared<KiriScene2D>(700, 700));
        }

        KiriRenderer2D(
            KiriScene2DPtr scene)
            : mScene(std::move(scene))
        {
            mWindowWidth = mScene->GetWindowWidth();
            mWindowHeight = mScene->GetWindowHeight();
            mCanvas = cv::Mat::zeros(mWindowHeight, mWindowWidth, CV_8UC3);
            this->ClearCanvas();
        }

        void ClearCanvas();
        void DrawCanvas();
        void SaveImages2File();
        void SaveImages2FileWithPrefix(std::string prefix);
        inline auto GetCanvas() { return mCanvas; }

        ~KiriRenderer2D() noexcept {}

    private:
        cv::Mat mCanvas;
        UInt counter = 0;
        KiriScene2DPtr mScene;
        Int mWindowWidth, mWindowHeight;
    };
    typedef SharedPtr<KiriRenderer2D> KiriRenderer2DPtr;
}
#endif