/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-03-27 01:28:37
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-14 15:01:38
 * @FilePath: \Kiri2D\core\src\kiri2d\scene.cpp
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#include <kiri2d/scene.h>

namespace KIRI2D
{
    void KiriScene2D::AddObject(KiriSDF2D object)
    {
        mSDFObjects.emplace_back(object);
    }

    void KiriScene2D::AddParticle(KiriPoint2 particle)
    {
        mPoints.emplace_back(particle);
    }

    void KiriScene2D::AddParticles(std::vector<KiriPoint2> particles)
    {
        for (size_t i = 0; i < particles.size(); i++)
        {
            mPoints.emplace_back(particles[i]);
        }
    }

    void KiriScene2D::AddCircle(KiriCircle2 circle)
    {
        mCircles.emplace_back(circle);
    }

    void KiriScene2D::AddCircles(std::vector<KiriCircle2> circles)
    {
        for (size_t i = 0; i < circles.size(); i++)
        {
            mCircles.emplace_back(circles[i]);
        }
    }

    void KiriScene2D::AddLine(KiriLine2 line)
    {
        mLines.emplace_back(line);
    }

    void KiriScene2D::AddLines(std::vector<KiriLine2> lines)
    {
        for (size_t i = 0; i < lines.size(); i++)
        {
            mLines.emplace_back(lines[i]);
        }
    }

    void KiriScene2D::AddRect(KiriRect2 rect)
    {
        mRects.emplace_back(rect);
    }

    void KiriScene2D::AddRects(std::vector<KiriRect2> rects)
    {
        for (size_t i = 0; i < rects.size(); i++)
        {
            mRects.emplace_back(rects[i]);
        }
    }
}