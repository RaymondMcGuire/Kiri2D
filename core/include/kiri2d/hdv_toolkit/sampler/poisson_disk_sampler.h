/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-09-26 16:12:57
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-03 10:41:23
 * @FilePath: \Kiri2D\core\include\kiri2d\hdv_toolkit\sampler\poisson_disk_sampler.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _HDV_POISSON_DISK_SAMPLER_H_
#define _HDV_POISSON_DISK_SAMPLER_H_

#pragma once

#include <kiri_pch.h>

namespace HDV::Sampler
{
    class PoissonDiskSampler2D
    {
    public:
        explicit PoissonDiskSampler2D()
        {
        }

        virtual ~PoissonDiskSampler2D() {}

        void initUniRadiiSampler(float radius, Vector2F boundarySize)
        {
            mFinished = false;
            mCellSize = radius / std::sqrtf(2.f);
            mGrid = Vector<Vec_Int>(static_cast<size_t>(boundarySize.x / mCellSize) + 1, Vec_Int(static_cast<size_t>(boundarySize.y / mCellSize) + 1, 0));
            mSpawnPoints.emplace_back(boundarySize / 2.f);
        }

        void initMultiRadiiSampler(float minRadius, float maxRadius, Vector2F boundarySize)
        {
            mFinished = false;
            mCellSize = minRadius / std::sqrtf(2.f);
            mMaxCellSize = maxRadius / std::sqrtf(2.f);
            mGrid = Vector<Vec_Int>(static_cast<size_t>(boundarySize.x / mCellSize) + 1, Vec_Int(static_cast<size_t>(boundarySize.y / mCellSize) + 1, 0));
            mSpawnPoints.emplace_back(boundarySize / 2.f);
        }

        Vec_Vec2F generateUniRadii(float radius, Vector2F boundarySize, size_t attemptNum = 30)
        {

            if (mSpawnPoints.empty())
            {
                mFinished = true;
                return mPoints;
            }

            std::random_device seed;
            std::default_random_engine rnd_engine(seed());
            std::uniform_real_distribution<float> dist(0.f, 1.f);

            auto spawn_index = static_cast<size_t>(mSpawnPoints.size() * dist(rnd_engine));
            auto spawn_centre = mSpawnPoints[spawn_index];
            auto candidate_accepted = false;

            for (auto i = 0; i < attemptNum; i++)
            {
                auto angle = dist(rnd_engine) * KIRI_PI<float>() * 2.f;
                auto dir = Vector2F(std::sinf(angle), std::cosf(angle));
                auto candidate = spawn_centre + dir * radius * std::sqrtf(dist(rnd_engine) * 4.f);
                if (isValid(candidate, boundarySize, mCellSize, radius, mPoints, mGrid))
                {
                    mPoints.emplace_back(candidate);
                    mSpawnPoints.emplace_back(candidate);
                    // KIRI_LOG_DEBUG("size={0},{1}", static_cast<size_t>(candidate.x / mCellSize), static_cast<size_t>(candidate.y / mCellSize));
                    mGrid[static_cast<size_t>(candidate.x / mCellSize)][static_cast<size_t>(candidate.y / mCellSize)] = mPoints.size();
                    candidate_accepted = true;
                    break;
                }
            }
            if (!candidate_accepted)
                mSpawnPoints.erase(mSpawnPoints.begin() + spawn_index);

            return mPoints;
        }

        Vec_Vec3F generateMultiRadii(Vec_Float radiusRange, Vec_Float radiusProb, Vector2F boundarySize, size_t attemptNum = 30)
        {
            if (mSpawnPoints.empty())
            {
                mFinished = true;
                return mPointsPlusRadius;
            }

            std::random_device seed;
            std::default_random_engine rnd_engine(seed());
            std::uniform_real_distribution<float> dist(0.f, 1.f);

            std::mt19937 mt_engine(seed());
            std::piecewise_constant_distribution<float> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusProb)};

            auto spawn_index = static_cast<size_t>(mSpawnPoints.size() * dist(rnd_engine));
            auto spawn_centre = mSpawnPoints[spawn_index];
            auto candidate_accepted = false;

            for (auto i = 0; i < attemptNum; i++)
            {
                float radius = pcdis(mt_engine);
                auto angle = dist(rnd_engine) * KIRI_PI<float>() * 2.f;
                auto dir = Vector2F(std::sinf(angle), std::cosf(angle));
                auto candidate = spawn_centre + dir * radius * std::sqrtf(dist(rnd_engine) * 4.f);
                if (isValid(candidate, boundarySize, mCellSize, radius, mPoints, mPointsPlusRadius, mGrid))
                {
                    mPoints.emplace_back(candidate);
                    mSpawnPoints.emplace_back(candidate);
                    mPointsPlusRadius.emplace_back(Vector3F(candidate.x, candidate.y, radius));
                    mGrid[static_cast<size_t>(candidate.x / mCellSize)][static_cast<size_t>(candidate.y / mCellSize)] = mPoints.size();
                    candidate_accepted = true;
                    break;
                }
            }
            if (!candidate_accepted)
                mSpawnPoints.erase(mSpawnPoints.begin() + spawn_index);

            return mPointsPlusRadius;
        }

        bool sampledFinished() const { return mFinished; }

    private:
        float mCellSize, mMaxCellSize;
        bool mFinished = false;
        Vector<Vec_Int> mGrid;
        Vec_Vec2F mPoints, mSpawnPoints;
        Vec_Vec3F mPointsPlusRadius;

        bool isValid(Vector2F candidate, Vector2F boundarySize, float cellSize, float radius, Vec_Vec2F points, Vector<Vec_Int> grid)
        {
            if (candidate.x >= 0 &&
                candidate.x < boundarySize.x &&
                candidate.y >= 0 &&
                candidate.y < boundarySize.y &&
                !grid.empty())
            {
                auto cell_x = static_cast<size_t>(candidate.x / cellSize);
                auto cell_y = static_cast<size_t>(candidate.y / cellSize);

                auto search_startx = static_cast<size_t>(0);
                auto search_starty = static_cast<size_t>(0);
                if (cell_x > 2)
                    search_startx = cell_x - 2;
                if (cell_y > 2)
                    search_starty = cell_y - 2;

                auto search_endx = std::min(cell_x + 2, grid.size() - 1);
                auto search_endy = std::min(cell_y + 2, grid[0].size() - 1);

                for (auto x = search_startx; x <= search_endx; x++)
                {
                    for (auto y = search_starty; y <= search_endy; y++)
                    {
                        auto point_index = grid[x][y] - 1;
                        if (point_index != -1)
                        {
                            float sqr_dst = (candidate - points[point_index]).lengthSquared();
                            if (sqr_dst < radius * radius)
                            {
                                return false;
                            }
                        }
                    }
                }
                return true;
            }
            return false;
        }

        bool isValid(Vector2F candidate, Vector2F boundarySize, float cellSize, float radius, Vec_Vec2F points, Vec_Vec3F pointsPlusRadius, Vector<Vec_Int> grid)
        {
            if (candidate.x >= 0 &&
                candidate.x < boundarySize.x &&
                candidate.y >= 0 &&
                candidate.y < boundarySize.y &&
                !grid.empty())
            {
                auto cell_x = static_cast<size_t>(candidate.x / cellSize);
                auto cell_y = static_cast<size_t>(candidate.y / cellSize);

                auto search_addition = static_cast<size_t>(10.f * mMaxCellSize / cellSize);

                auto search_startx = static_cast<size_t>(0);
                auto search_starty = static_cast<size_t>(0);
                if (cell_x > search_addition)
                    search_startx = cell_x - search_addition;
                if (cell_y > search_addition)
                    search_starty = cell_y - search_addition;

                auto search_endx = std::min(cell_x + search_addition, grid.size() - 1);
                auto search_endy = std::min(cell_y + search_addition, grid[0].size() - 1);

                for (auto x = search_startx; x <= search_endx; x++)
                {
                    for (auto y = search_starty; y <= search_endy; y++)
                    {
                        auto point_index = grid[x][y] - 1;
                        if (point_index != -1)
                        {
                            auto sqr_dst = (candidate - points[point_index]).lengthSquared();
                            if (sqr_dst < pointsPlusRadius[point_index].z * pointsPlusRadius[point_index].z)
                                return false;
                        }
                    }
                }
                return true;
            }
            return false;
        }
    };
    typedef SharedPtr<PoissonDiskSampler2D> PoissonDiskSampler2DPtr;

}
#endif