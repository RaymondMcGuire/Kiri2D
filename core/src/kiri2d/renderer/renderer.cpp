/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-09-26 16:12:57
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-16 14:07:52
 * @FilePath: \Kiri2D\core\src\kiri2d\renderer\renderer.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d/renderer/renderer.h>
#include <root_directory.h>
namespace KIRI2D
{
    String UInt2Str4Digit(UInt Input)
    {
        char output[5];
        snprintf(output, 5, "%04d", Input);
        return String(output);
    };

    void KiriRenderer2D::saveImages2File()
    {
        cv::imwrite(String(EXPORT_PATH) + "images/" + UInt2Str4Digit(counter++) + ".png", mCanvas);
    }

    void KiriRenderer2D::saveImages2FileWithPrefix(std::string prefix)
    {
        cv::imwrite(String(EXPORT_PATH) + "images/" + prefix + "_" + UInt2Str4Digit(counter++) + ".png", mCanvas);
    }

    void KiriRenderer2D::clearCanvas()
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

    void KiriRenderer2D::drawCanvas()
    {
        auto particles = mScene->points();
        for (int i = 0; i < particles.size(); i++)
        {
            auto relate_position = mScene->camera()->project(particles[i].pos);
            int cx = relate_position[0];
            int cy = mWindowHeight - relate_position[1];
            if (cx < 0 || cx >= mWindowWidth || cy < 0 || cy >= mWindowHeight)
                continue;
            auto col = particles[i].col * 255.f;

            cv::circle(mCanvas, cv::Point(cx, cy), particles[i].radius, cv::Scalar(col.z, col.y, col.x, -1), -1);
        }

        auto circles = mScene->circles();
        for (int i = 0; i < circles.size(); i++)
        {
            auto relate_position = mScene->camera()->project(circles[i].pos);
            int cx = relate_position[0];
            int cy = mWindowHeight - relate_position[1];
            if (cx < 0 || cx >= mWindowWidth || cy < 0 || cy >= mWindowHeight)
                continue;
            auto col = circles[i].col * 255.f;
            if (circles[i].fill)
                cv::circle(mCanvas, cv::Point(cx, cy), circles[i].radius, cv::Scalar(col.z, col.y, col.x, -1), -1);
            else
                cv::circle(mCanvas, cv::Point(cx, cy), circles[i].radius, cv::Scalar(col.z, col.y, col.x, -1), 1);
        }

        auto lines = mScene->lines();
        for (int i = 0; i < lines.size(); i++)
        {
            Vector2F start_relate_position = mScene->camera()->project(lines[i].start);
            int sx = start_relate_position[0];
            int sy = mWindowHeight - start_relate_position[1];
            Vector2F end_relate_position = mScene->camera()->project(lines[i].end);
            int ex = end_relate_position[0];
            int ey = mWindowHeight - end_relate_position[1];
            auto col = lines[i].col * 255.f;
            cv::line(mCanvas, cv::Point(sx, sy), cv::Point(ex, ey), cv::Scalar(col.x, col.y, col.z), lines[i].thick * mScene->camera()->viewScale());
        }

        auto rects = mScene->rects();
        for (int i = 0; i < rects.size(); i++)
        {
            Vector2F original = mScene->camera()->project(rects[i].original + Vector2F(0.f, rects[i].size.y));
            int ox = original[0];
            int oy = mWindowHeight - original[1];

            cv::Rect rect(ox, oy, rects[i].size.x, rects[i].size.y);
            cv::rectangle(mCanvas, rect, cv::Scalar(253, 185, 134), 2.f * mScene->camera()->viewScale());
        }

        auto sdfObjects = mScene->sdfObjects();
        for (size_t i = 0; i < sdfObjects.size(); i++)
        {
            auto points = sdfObjects[i].points();
            auto offset = sdfObjects[i].offset();
            for (int j = 0, k = points.size() - 1, l = points.size(); j < l; k = j++)
            {
                Vector2F start_relate_position = mScene->camera()->project(points[k] + offset);
                auto sx = (size_t)start_relate_position.x;
                auto sy = mWindowHeight - (size_t)start_relate_position.y;
                Vector2F end_relate_position = mScene->camera()->project(points[j] + offset);
                auto ex = (size_t)end_relate_position.x;
                auto ey = mWindowHeight - (size_t)end_relate_position.y;
                cv::line(mCanvas, cv::Point(sx, sy), cv::Point(ex, ey), cv::Scalar(253, 185, 134), 2.f * mScene->camera()->viewScale());
            }
        }
    }
}