/*** 
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-08-01 11:32:37
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-08-23 16:48:43
 * @FilePath: \Kiri2D\demos\pdipm_sampler2\include\gridding.h
 * @Description: 
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved. 
 */
/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-20 16:30:47
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-20 16:30:47
 * @FilePath: \Kiri2D\demos\interior_point_method\include\gridding copy.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _GRIDDING_H_
#define _GRIDDING_H_

#pragma once
#include <data.h>

#include <tuple>

namespace OPTIMIZE::IPM
{
    class Gridding
    {
    public:
        explicit Gridding(
            const std::vector<particle> &data,
            int x, int y, int z)
        {
            mGridSize.set(x, y, z);

            // auto max_radius = 0.0;
            for (auto i = 0; i < data.size(); i++)
            {
                mBbox.merge(data[i].pos);
                mData.emplace_back(data[i]);
                // max_radius = std::max(max_radius, data[i].radius);
                //  KIRI_LOG_DEBUG("radius={0},{1},{2}", data[i].pos.x, data[i].pos.y, data[i].pos.z);
            }
            // KIRI_LOG_DEBUG("low={0},{1},{2}", mBbox.LowestPoint.x, mBbox.LowestPoint.y, mBbox.LowestPoint.z);
            // KIRI_LOG_DEBUG(" high={0},{1},{2}", mBbox.HighestPoint.x, mBbox.HighestPoint.y, mBbox.HighestPoint.z);
            // mBbox.expand(max_radius * 2.5);
            // KIRI_LOG_DEBUG("max_radius={0}", max_radius);
            // KIRI_LOG_DEBUG("expand low={0},{1},{2}", mBbox.LowestPoint.x, mBbox.LowestPoint.y, mBbox.LowestPoint.z);
            // KIRI_LOG_DEBUG("expand high={0},{1},{2}", mBbox.HighestPoint.x, mBbox.HighestPoint.y, mBbox.HighestPoint.z);

            mCellSize.set(mBbox.length(0) / static_cast<double>(mGridSize.x), mBbox.length(1) / static_cast<double>(mGridSize.y), mBbox.length(2) / static_cast<double>(mGridSize.z));
            // KIRI_LOG_DEBUG("mBbox={0},{1},{2}", mBbox.length(0), mBbox.length(1), mBbox.length(2));
            // KIRI_LOG_DEBUG("mCellSize={0},{1},{2}", mCellSize.x, mCellSize.y, mCellSize.z);
        }

        virtual ~Gridding()
        {
        }

        std::tuple<std::vector<particle>, std::vector<int>> getDataByGridHash(int hash)
        {
            std::vector<int> index;
            std::vector<particle> data;
            for (auto i = 0; i < mData.size(); i++)
            {
                auto data_grid_hash = computePos2GridHash(mData[i].pos);
                // KIRI_LOG_DEBUG("data_grid_hash={0}", data_grid_hash);
                if (data_grid_hash == hash)
                {
                    index.emplace_back(i);

                    // auto rel_pos = mData[i].pos - mBbox.LowestPoint;
                    // auto grid_xyz = computeGridXYZByPos3(rel_pos);
                    // auto cell_lowest = mBbox.LowestPoint + Vector3D(grid_xyz.x * mCellSize.x, grid_xyz.y * mCellSize.y, grid_xyz.z * mCellSize.z);
                    // // KIRI_LOG_DEBUG("eee low={0},{1},{2}", mBbox.LowestPoint.x, mBbox.LowestPoint.y, mBbox.LowestPoint.z);
                    // // KIRI_LOG_DEBUG("eee grid_xyz={0},{1},{2}", grid_xyz.x * mCellSize.x, grid_xyz.y * mCellSize.y, grid_xyz.z * mCellSize.z);
                    // // KIRI_LOG_DEBUG("eee cell_lowest={0},{1},{2}", cell_lowest.x, cell_lowest.y, cell_lowest.z);

                    // auto cell_highest = cell_lowest + mCellSize;
                    // auto min_dist = computeMinDist(mData[i].pos, cell_lowest, cell_highest);
                    // mData[i].min_dist = min_dist;
                    data.emplace_back(mData[i]);
                }
            }
            return std::make_tuple(data, index);
        }

        int maxGridHash() const { return mGridSize.x * mGridSize.y * mGridSize.z; }

        void updateData(const std::vector<particle> &data)
        {
            mData = data;
        }

    private:
        Vector3D mCellSize;
        Size3 mGridSize;
        BoundingBox3D mBbox;
        std::vector<particle> mData;

        Point3I computeGridXYZByPos3(
            const Vector3D &pos)
        {
            // KIRI_LOG_DEBUG("pos.x / mCellSize.x={0},{1},{2}", pos.x / mCellSize.x, pos.y / mCellSize.y, pos.z / mCellSize.z);
            int x = std::min(std::max((int)(pos.x / mCellSize.x), 0), (int)(mGridSize.x - 1)),
                y = std::min(std::max((int)(pos.y / mCellSize.y), 0), (int)(mGridSize.y - 1)),
                z = std::min(std::max((int)(pos.z / mCellSize.z), 0), (int)(mGridSize.z - 1));

            return Point3I(x, y, z);
        }

        int computePos2GridHash(
            const Vector3D &pos)
        {
            auto rel_pos = pos - mBbox.LowestPoint;
            auto grid_xyz = computeGridXYZByPos3(rel_pos);
            // KIRI_LOG_DEBUG("grid_xyz={0},{1},{2}", grid_xyz.x, grid_xyz.y, grid_xyz.z);
            return grid_xyz.x * mGridSize.y * mGridSize.z + grid_xyz.y * mGridSize.z +
                   grid_xyz.z;
        }
    };

    typedef std::shared_ptr<Gridding> GraddingPtr;
}

#endif