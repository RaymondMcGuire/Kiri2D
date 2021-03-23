/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:24:22
 * @LastEditTime: 2021-03-24 02:20:33
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\scene.cpp
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
}