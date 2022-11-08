/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-08 18:11:34
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-08 23:56:20
 * @FilePath: \Kiri2D\core\include\kiri2d\hdv_toolkit\sdf\sdf2d.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _HDV_SDF_POLYGON2D_H_
#define _HDV_SDF_POLYGON2D_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_polygon.h>

namespace HDV::SDF
{
  class PolygonSDF2D
  {
  public:
    explicit PolygonSDF2D(const Voronoi::VoronoiPolygon2Ptr &polygon,
                          double cellSize)
        : mPolygon(polygon), mCellSize(cellSize)
    {

      mBbox = mPolygon->bbox();
      mWidthCellNum = mBbox.width() / mCellSize;
      mHeightCellNum = mBbox.height() / mCellSize;
      mSDFData.resize(mWidthCellNum, mHeightCellNum,
                      (mHeightCellNum + mWidthCellNum) * mCellSize);
    }

    virtual ~PolygonSDF2D() {}

    inline void get_barycentric(double x, Int &i, double &f, Int i_low, Int i_high)
    {
      auto s = std::floor(x);
      i = static_cast<Int>(s);

      if (i < i_low)
      {
        i = i_low;
        f = 0;
      }
      else if (i > i_high - 2)
      {
        i = i_high - 2;
        f = 1;
      }
      else
      {
        f = x - s;
      }
    }

    template <class S, class T>
    inline S lerp(const S &value0, const S &value1, T f)
    {
      return (T(1.0) - f) * value0 + f * value1;
    }

    template <class S, class T>
    inline S bilerp(const S &v00, const S &v10,
                    const S &v01, const S &v11,
                    T fx, T fy)
    {
      return lerp(
          lerp(v00, v10, fx),
          lerp(v01, v11, fx),
          fy);
    }

    void computeSDF()
    {
      for (auto i = 0; i < mWidthCellNum; i++)
      {
        for (auto j = 0; j < mHeightCellNum; j++)
        {
          auto pos = mBbox.LowestPoint + Vector2D(i * mCellSize, j * mCellSize);
          auto sgn = mPolygon->contains(pos) ? 1.0 : -1.0;
          auto dist = mPolygon->computeMinDisInPoly(pos);
          auto val = sgn * dist;
          if (abs(val) < mSDFData(i, j))
          {
            mSDFData(i, j) = val;
          }
        }
      }
    }

    double getSDF(Vector2D pos)
    {
      auto grid_pos = (pos - mBbox.LowestPoint) / mCellSize;

      Int i, j;
      double fi, fj;
      get_barycentric(grid_pos.x, i, fi, 0, mWidthCellNum);
      get_barycentric(grid_pos.y, j, fj, 0, mHeightCellNum);
      return bilerp(mSDFData(i, j), mSDFData(i + 1, j), mSDFData(i, j + 1), mSDFData(i + 1, j + 1), fi, fj);
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