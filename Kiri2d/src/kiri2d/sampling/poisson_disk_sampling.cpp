/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 15:22:11
 * @LastEditTime: 2021-09-21 17:46:01
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\sampling\poisson_disk_sampling.cpp
 */

#include <kiri2d/sampling/poisson_disk_sampling.h>
#include <random>

namespace KIRI2D
{

    bool KiriPoissonDiskSampling2D::IsValid(Vector2F candidate, Vector2F boundarySize, float cellSize, float radius, Vec_Vec2F mPoints, Vector<Vec_Int> mGrid)
    {
        if (candidate.x >= 0 && candidate.x < boundarySize.x && candidate.y >= 0 && candidate.y < boundarySize.y && !mGrid.empty())
        {
            auto cellx = static_cast<size_t>(candidate.x / cellSize);
            auto celly = static_cast<size_t>(candidate.y / cellSize);

            auto search_startx = static_cast<size_t>(0);
            auto search_starty = static_cast<size_t>(0);
            if (cellx >= 2)
                search_startx = cellx - 2;
            if (celly >= 2)
                search_starty = celly - 2;

            auto search_endx = std::min(cellx + 2, mGrid.size() - 1);
            auto search_endy = std::min(celly + 2, mGrid[0].size() - 1);

            // KIRI_LOG_DEBUG("cellx={0},mGrid.size()-1={1};celly={2}, mGrid[0].size() - 1={3}", cellx, mGrid.size() - 1, celly, mGrid[0].size() - 1);
            // KIRI_LOG_DEBUG("search xy={0},{1};{2},{3}", search_startx, search_endx, search_starty, search_endy);
            for (auto x = search_startx; x <= search_endx; x++)
            {
                for (auto y = search_starty; y <= search_endy; y++)
                {
                    auto point_index = mGrid[x][y] - 1;
                    if (point_index != -1)
                    {
                        float sqr_dst = (candidate - mPoints[point_index]).lengthSquared();
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

    void KiriPoissonDiskSampling2D::InitUniSampler(float radius, Vector2F boundarySize)
    {
        mCellSize = radius / std::sqrtf(2.f);
        mGrid = Vector<Vec_Int>(static_cast<size_t>(boundarySize.x / mCellSize) + 1, Vec_Int(static_cast<size_t>(boundarySize.y / mCellSize) + 1, 0));
        mSpawnPoints.emplace_back(boundarySize / 2.f);
    }

    Vec_Vec2F KiriPoissonDiskSampling2D::UinSampling(float radius, Vector2F boundarySize, size_t attemptNum)
    {

        if (mSpawnPoints.empty())
            return mPoints;

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
            if (IsValid(candidate, boundarySize, mCellSize, radius, mPoints, mGrid))
            {
                mPoints.emplace_back(candidate);
                mSpawnPoints.emplace_back(candidate);
                //KIRI_LOG_DEBUG("size={0},{1}", static_cast<size_t>(candidate.x / mCellSize), static_cast<size_t>(candidate.y / mCellSize));
                mGrid[static_cast<size_t>(candidate.x / mCellSize)][static_cast<size_t>(candidate.y / mCellSize)] = mPoints.size();
                candidate_accepted = true;
                break;
            }
        }
        if (!candidate_accepted)
        {
            mSpawnPoints.erase(mSpawnPoints.begin() + spawn_index);
        }
        return mPoints;
    }
}