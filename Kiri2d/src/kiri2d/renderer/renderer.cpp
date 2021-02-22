/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:33:21
 * @LastEditTime: 2021-02-22 18:39:23
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
        auto sdfObjects = mScene->GetSDFObjects();
        for (size_t i = 0; i < sdfObjects.size(); i++)
        {
            auto obj = sdfObjects[i];
            cv::line(mCanvas, cv::Point(sx, sy), cv::Point(ex, ey), cv::Scalar(253, 185, 134), 2.0 * sc->camera.Scale());
        }
    }
}