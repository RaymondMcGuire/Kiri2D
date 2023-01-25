/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-02-23 00:18:39
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-25 23:21:33
 * @FilePath: \Kiri2D\core\include\kiri2d\renderer\renderer.h
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _KIRI2D_RENDERER_H_
#define _KIRI2D_RENDERER_H_

#pragma once

#include <kiri2d/scene.h>
#include <opencv2/opencv.hpp>

namespace KIRI2D
{
    template <class RealType>
    class KiriRenderer2D
    {
    public:
        KiriRenderer2D()
        {
            KiriRenderer2D(std::make_shared<KiriScene2D<float>>(700, 700));
        }

        KiriRenderer2D(
            std::shared_ptr<KiriScene2D<RealType>> scene)
            : mScene(std::move(scene))
        {
            mWindowWidth = mScene->GetWindowWidth();
            mWindowHeight = mScene->GetWindowHeight();
            mCanvas = cv::Mat::zeros(mWindowHeight, mWindowWidth, CV_8UC3);
            this->ClearCanvas();
        }

        String UInt2Str4Digit(UInt Input)
        {
            char output[5];
            snprintf(output, 5, "%04d", Input);
            return String(output);
        };

        void SaveImages2File()
        {
            cv::imwrite(String(EXPORT_PATH) + "images/" + UInt2Str4Digit(counter++) + ".png", mCanvas);
        }

        void SaveImages2FileWithPrefix(std::string prefix)
        {
            cv::imwrite(String(EXPORT_PATH) + "images/" + prefix + "_" + UInt2Str4Digit(counter++) + ".png", mCanvas);
        }

        void ClearCanvas()
        {
            for (int j = 0; j < mWindowHeight; j++)
            {
                for (int i = 0; i < mWindowWidth; i++)
                {
                    mCanvas.at<cv::Vec3b>(j, i)(0) = 255;
                    mCanvas.at<cv::Vec3b>(j, i)(1) = 255;
                    mCanvas.at<cv::Vec3b>(j, i)(2) = 255;
                }
            }
        }

        void DrawCanvas()
        {

            auto circles = mScene->GetCircles();
            for (int i = 0; i < circles.size(); i++)
            {
                auto relate_position = mScene->GetCamera()->project(circles[i].pos);
                int cx = relate_position[0];
                int cy = mWindowHeight - relate_position[1];
                if (cx < 0 || cx >= mWindowWidth || cy < 0 || cy >= mWindowHeight)
                    continue;
                auto col = circles[i].col * static_cast<RealType>(255.0);
                if (circles[i].fill)
                    cv::circle(mCanvas, cv::Point(cx, cy), circles[i].radius, cv::Scalar(col.z, col.y, col.x, -1), -1);
                else
                    cv::circle(mCanvas, cv::Point(cx, cy), circles[i].radius, cv::Scalar(col.z, col.y, col.x, -1), 1);
            }

            auto lines = mScene->GetLines();
            for (int i = 0; i < lines.size(); i++)
            {
                VectorX<2, RealType> start_relate_position = mScene->GetCamera()->project(lines[i].start);
                int sx = start_relate_position[0];
                int sy = mWindowHeight - start_relate_position[1];
                VectorX<2, RealType> end_relate_position = mScene->GetCamera()->project(lines[i].end);
                int ex = end_relate_position[0];
                int ey = mWindowHeight - end_relate_position[1];
                auto col = lines[i].col * static_cast<RealType>(255.0);
                cv::line(mCanvas, cv::Point(sx, sy), cv::Point(ex, ey), cv::Scalar(col.x, col.y, col.z), lines[i].thick * mScene->GetCamera()->viewScale());
            }

            auto rects = mScene->GetRects();
            for (int i = 0; i < rects.size(); i++)
            {
                VectorX<2, RealType> original = mScene->GetCamera()->project(rects[i].original + VectorX<2, RealType>(0.f, rects[i].size.y));
                int ox = original[0];
                int oy = mWindowHeight - original[1];

                cv::Rect rect(ox, oy, rects[i].size.x, rects[i].size.y);
                cv::rectangle(mCanvas, rect, cv::Scalar(253, 185, 134), 2.f * mScene->GetCamera()->viewScale());
            }

            // auto GetSDFObjects = mScene->GetSDFObjects();
            // for (size_t i = 0; i < GetSDFObjects.size(); i++)
            // {
            //     auto points = GetSDFObjects[i].points();
            //     auto offset = GetSDFObjects[i].offset();
            //     for (int j = 0, k = points.size() - 1, l = points.size(); j < l; k = j++)
            //     {
            //         VectorX<2, RealType> start_relate_position = mScene->GetCamera()->project(points[k] + offset);
            //         auto sx = (size_t)start_relate_position.x;
            //         auto sy = mWindowHeight - (size_t)start_relate_position.y;
            //         VectorX<2, RealType> end_relate_position = mScene->GetCamera()->project(points[j] + offset);
            //         auto ex = (size_t)end_relate_position.x;
            //         auto ey = mWindowHeight - (size_t)end_relate_position.y;
            //         cv::line(mCanvas, cv::Point(sx, sy), cv::Point(ex, ey), cv::Scalar(253, 185, 134), 2.f * mScene->GetCamera()->viewScale());
            //     }
            // }

            auto particles = mScene->GetPoints();
            for (int i = 0; i < particles.size(); i++)
            {
                auto relate_position = mScene->GetCamera()->project(particles[i].pos);
                int cx = relate_position[0];
                int cy = mWindowHeight - relate_position[1];
                if (cx < 0 || cx >= mWindowWidth || cy < 0 || cy >= mWindowHeight)
                    continue;
                auto col = particles[i].col * static_cast<RealType>(255.0);

                cv::circle(mCanvas, cv::Point(cx, cy), particles[i].radius, cv::Scalar(col.z, col.y, col.x, -1), -1);
            }
        }

        inline auto GetCanvas() { return mCanvas; }

        ~KiriRenderer2D() {}

    private:
        cv::Mat mCanvas;
        UInt counter = 0;
        std::shared_ptr<KiriScene2D<RealType>> mScene;
        Int mWindowWidth, mWindowHeight;
    };
    typedef SharedPtr<KiriRenderer2D<float>> KiriRenderer2DFPtr;
    typedef SharedPtr<KiriRenderer2D<double>> KiriRenderer2DDPtr;
}
#endif