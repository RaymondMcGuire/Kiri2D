/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 16:35:26
 * @LastEditTime: 2021-09-22 16:28:24
 * @LastEditors: Xu.WANG
 * @Description: Reference-> Computing Distance, Erin Catto, Blizzard Entertainment, GDC2010
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\sampling\poisson_disk_sampling.h
 */

#ifndef _KIRI2D_POISSON_DISK_SAMPLING_H_
#define _KIRI2D_POISSON_DISK_SAMPLING_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D
{
    class KiriPoissonDiskSampling2D
    {
    public:
        KiriPoissonDiskSampling2D()
        {
        }

        ~KiriPoissonDiskSampling2D() noexcept {}

        void InitUniSampler(float radius, Vector2F boundarySize);
        void InitMultiRadiiSampler(float minRadius, float maxRadius, Vector2F boundarySize);
        Vec_Vec2F UniSampling(float radius, Vector2F boundarySize, size_t attemptNum = 30);
        Vec_Vec3F MultiRadiiSampling(Vec_Float radiusRange, Vec_Float radiusProb, Vector2F boundarySize, size_t attemptNum = 30);

    private:
        float mCellSize, mMaxCellSize;
        Vector<Vec_Int> mGrid;
        Vec_Vec2F mPoints, mSpawnPoints;
        Vec_Vec3F mPointsPlusRadius;
        bool IsValid(Vector2F candidate, Vector2F boundarySize, float cellSize, float radius, Vec_Vec2F points, Vector<Vec_Int> grid);
        bool IsValid(Vector2F candidate, Vector2F boundarySize, float cellSize, float radius, Vec_Vec2F points, Vec_Vec3F pointsPlusRadius, Vector<Vec_Int> grid);
    };
    typedef SharedPtr<KiriPoissonDiskSampling2D> KiriPoissonDiskSampling2DPtr;

}
#endif