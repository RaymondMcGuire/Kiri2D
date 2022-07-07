/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-06 13:03:58
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-06 13:03:59
 * @FilePath: \Kiri2D\demos\interior_point_method\include\gridding.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _GRADDING_H_
#define _GRADDING_H_

#pragma once
#include <kiri2d.h>
using namespace KIRI2D;

#include <tuple>

namespace OPTIMIZE::IPM
{
    class Gradding
    {
    public:
        explicit Gradding(
            const std::vector<Vector3D> &data,
            const std::vector<double> &radius,
            int x, int y, int z)
        {
            mGridSize.set(x, y, z);

            for (auto i = 0; i < data.size(); i++)
            {
                mBbox.merge(data[i]);
                mData.emplace_back(Vector4D(data[i].x, data[i].y, data[i].z, radius[i]));
            }

            mCellSize.set(mBbox.length(0) / static_cast<double>(mGridSize.x), mBbox.length(1) / static_cast<double>(mGridSize.y), mBbox.length(2) / static_cast<double>(mGridSize.z));
            // KIRI_LOG_DEBUG("mBbox={0},{1},{2}", mBbox.length(0), mBbox.length(1), mBbox.length(2));
            // KIRI_LOG_DEBUG("mCellSize={0},{1},{2}", mCellSize.x, mCellSize.y, mCellSize.z);
        }

        virtual ~Gradding()
        {
        }

        std::tuple<std::vector<Vector4D>, std::vector<int>> getDataByGridHash(int hash)
        {
            std::vector<int> index;
            std::vector<Vector4D> data;
            for (auto i = 0; i < mData.size(); i++)
            {
                auto data_grid_hash = computePos2GridHash(Vector3D(mData[i].x, mData[i].y, mData[i].z));
                // KIRI_LOG_DEBUG("data_grid_hash={0}", data_grid_hash);
                if (data_grid_hash == hash)
                {
                    index.emplace_back(i);
                    data.emplace_back(mData[i]);
                }
            }
            return std::make_tuple(data, index);
        }

        int maxGridHash() const { return mGridSize.x * mGridSize.y * mGridSize.z; }

    private:
        Vector3D mCellSize;
        Size3 mGridSize;
        BoundingBox3D mBbox;
        std::vector<Vector4D> mData;

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

    typedef std::shared_ptr<Gradding> GraddingPtr;
}

#endif