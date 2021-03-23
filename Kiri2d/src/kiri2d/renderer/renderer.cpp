/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:33:21
 * @LastEditTime: 2021-03-24 02:09:16
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\renderer\renderer.cpp
 */

#include <kiri2d/renderer/renderer.h>

namespace KIRI2D
{
    void KiriRenderer2D::ClearCanvas()
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

    void KiriRenderer2D::DrawCanvas()
    {
        auto particles = mScene->GetPoints();
        for (int i = 0; i < particles.size(); i++)
        {
            auto relate_position = mScene->GetCamera()->Project(particles[i].pos);
            int cx = relate_position[0];
            int cy = mWindowHeight - relate_position[1];
            if (cx < 0 || cx >= mWindowWidth || cy < 0 || cy >= mWindowHeight)
                continue;
            auto col = particles[i].col * 255.f;

            cv::circle(mCanvas, cv::Point(cx, cy), 5, cv::Scalar(col.z, col.y, col.x, -1), -1);
        }

        auto lines = mScene->GetLines();
        for (int i = 0; i < lines.size(); i++)
        {
            Vector2F start_relate_position = mScene->GetCamera()->Project(lines[i].start);
            int sx = start_relate_position[0];
            int sy = mWindowHeight - start_relate_position[1];
            Vector2F end_relate_position = mScene->GetCamera()->Project(lines[i].end);
            int ex = end_relate_position[0];
            int ey = mWindowHeight - end_relate_position[1];
            cv::line(mCanvas, cv::Point(sx, sy), cv::Point(ex, ey), cv::Scalar(205, 252, 107), 3.0 * mScene->GetCamera()->ViewScale());
        }

        auto sdfObjects = mScene->GetSDFObjects();
        for (size_t i = 0; i < sdfObjects.size(); i++)
        {
            auto points = sdfObjects[i].GetPoints();
            for (int j = 0, k = points.size() - 1, l = points.size(); j < l; k = j++)
            {
                Vector2F start_relate_position = mScene->GetCamera()->Project(points[k]);
                auto sx = (Int)start_relate_position.x;
                auto sy = mWindowHeight - (Int)start_relate_position.y;
                Vector2F end_relate_position = mScene->GetCamera()->Project(points[j]);
                auto ex = (Int)end_relate_position.x;
                auto ey = mWindowHeight - (Int)end_relate_position.y;
                cv::line(mCanvas, cv::Point(sx, sy), cv::Point(ex, ey), cv::Scalar(253, 185, 134), 2.0 * mScene->GetCamera()->ViewScale());
            }
        }
    }
}