/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-08 15:23:00
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-08 15:24:27
 * @FilePath: \Kiri2D\core\include\kiri2d\hdv_toolkit\sdf\sdf2d.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _HDV_SDF_POLYGON2D_H_
#define _HDV_SDF_POLYGON2D_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_polygon.h>

namespace HDV::SDF {
class PolygonSDF2D {
public:
  explicit PolygonSDF2D(const Voronoi::VoronoiPolygon2Ptr &polygon,
                        double cellSize)
      : mPolygon(polygon), mCellSize(cellSize) {

    mBbox = mPolygon->bbox();
    mWidthCellNum = mBbox.width() / mCellSize;
    mHeightCellNum = mBbox.height() / mCellSize;
    mSDFData.resize(mWidthCellNum, mHeightCellNum,
                    (mHeightCellNum + mWidthCellNum) * mCellSize);
  }

  virtual ~PolygonSDF2D() {}

  void computeSDF() {
    for (auto i = 0; i < mWidthCellNum; i++) {
      for (auto j = 0; j < mHeightCellNum; j++) {
        auto pos = mBbox.LowestPoint + Vector2D(i * mCellSize, j * mCellSize);
        auto sgn = mPolygon->contains(pos) ? 1.0 : -1.0;
        auto dist = mPolygon->computeMinDisInPoly(pos);
        auto val = sgn * dist;
        if (abs(val) < mSDFData(i, j)) {
          mSDFData(i, j) = val;
        }
      }
    }
  }

private:
  int mWidthCellNum, mHeightCellNum;
  double mCellSize;
  BoundingBox2D mBbox;
  Array2D mSDFData;
  Voronoi::VoronoiPolygon2Ptr mPolygon;
};
} // namespace HDV::SDF

#endif