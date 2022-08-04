/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-08-03 14:45:45
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-08-03 14:46:12
 * @FilePath: \Kiri2D\demos\pdipm_sampler2\include\offset_gridding.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _OFFSET_Gridding_H_
#define _OFFSET_Gridding_H_

#pragma once
#include <data.h>

#include <tuple>

namespace OPTIMIZE::IPM {
class OffsetGridding {
public:
  explicit OffsetGridding(const std::vector<particle> &data, int x, int y,
                          int z) {
    mGridSize.set(x, y, z);
    mChildGridSize.set(x * 2, y * 2, z * 2);
    mCompositeGridSize.set((x - 1) / 0.5 + 1, (y - 1) / 0.5 + 1,
                           (z - 1) / 0.5 + 1);

    // construct composite grid data
    auto composite_grid_size =
        mCompositeGridSize.x * mCompositeGridSize.y * mCompositeGridSize.z;

    auto child_grid_size =
        mChildGridSize.x * mChildGridSize.y * mChildGridSize.z;
    auto composite_grid_counter = 0;
    for (int i = 0; i < child_grid_size; i++) {
      int x000 = i;
      int x010 = i + 1;
      int x100 = i + mChildGridSize.y;
      int x110 = i + mChildGridSize.y + 1;
      int x001 = i + mChildGridSize.y * mChildGridSize.z;
      int x011 = i + mChildGridSize.y * mChildGridSize.z + 1;
      int x101 = i + mChildGridSize.y * mChildGridSize.z + mChildGridSize.y;
      int x111 = i + mChildGridSize.y * mChildGridSize.z + mChildGridSize.y + 1;

      if (x010 % mChildGridSize.y == 0 ||
          x100 % (mChildGridSize.y * mChildGridSize.z) < mChildGridSize.y ||
          x001 >= child_grid_size)
        continue;

      compositeGrid copt_grid;
      copt_grid.id = composite_grid_counter++;
      std::vector<int> sub_grid_id;
      sub_grid_id.emplace_back(x000);
      sub_grid_id.emplace_back(x010);
      sub_grid_id.emplace_back(x100);
      sub_grid_id.emplace_back(x110);
      sub_grid_id.emplace_back(x001);
      sub_grid_id.emplace_back(x011);
      sub_grid_id.emplace_back(x101);
      sub_grid_id.emplace_back(x111);
      copt_grid.subid = sub_grid_id;
      mCompositeGrid.emplace_back(copt_grid);
      // KIRI_LOG_DEBUG("composite_grid_id={0}", x000);
    }

    for (auto i = 0; i < data.size(); i++) {
      mBbox.merge(data[i].pos);
      mData.emplace_back(data[i]);
    }

    mCellSize.set(mBbox.length(0) / static_cast<double>(mChildGridSize.x),
                  mBbox.length(1) / static_cast<double>(mChildGridSize.y),
                  mBbox.length(2) / static_cast<double>(mChildGridSize.z));
  }

  virtual ~OffsetGridding() {}

  std::tuple<std::vector<particle>, std::vector<int>>
  getDataByGridHash(int hash) {
    auto child_hash = mCompositeGrid[hash].subid;
    std::vector<int> index;
    std::vector<particle> data;
    for (auto i = 0; i < mData.size(); i++) {
      auto data_grid_hash = computePos2GridHash(mData[i].pos);
      if (std::find(child_hash.begin(), child_hash.end(), data_grid_hash) !=
          child_hash.end()) {
        index.emplace_back(i);
        data.emplace_back(mData[i]);
      }
    }
    return std::make_tuple(data, index);
  }

  int maxGridHash() const { return mCompositeGrid.size(); }

  void updateData(const std::vector<particle> &data) { mData = data; }

private:
  Vector3D mCellSize;
  Size3 mGridSize;
  Size3 mCompositeGridSize;
  Size3 mChildGridSize;
  BoundingBox3D mBbox;
  std::vector<particle> mData;
  std::vector<compositeGrid> mCompositeGrid;

  Point3I computeGridXYZByPos3(const Vector3D &pos) {

    int x = std::min(std::max((int)(pos.x / mCellSize.x), 0),
                     (int)(mChildGridSize.x - 1)),
        y = std::min(std::max((int)(pos.y / mCellSize.y), 0),
                     (int)(mChildGridSize.y - 1)),
        z = std::min(std::max((int)(pos.z / mCellSize.z), 0),
                     (int)(mChildGridSize.z - 1));

    return Point3I(x, y, z);
  }

  int computePos2GridHash(const Vector3D &pos) {
    auto rel_pos = pos - mBbox.LowestPoint;
    auto grid_xyz = computeGridXYZByPos3(rel_pos);

    return grid_xyz.x * mChildGridSize.y * mChildGridSize.z +
           grid_xyz.y * mChildGridSize.z + grid_xyz.z;
  }
};

typedef std::shared_ptr<OffsetGridding> OffsetGriddingPtr;
} // namespace OPTIMIZE::IPM

#endif