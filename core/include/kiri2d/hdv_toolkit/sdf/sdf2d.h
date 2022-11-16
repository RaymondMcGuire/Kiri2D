/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-15 18:49:55
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-16 20:14:54
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
      mMaxSDFVal = 0.0;
      mBbox = mPolygon->bbox();
      mWidthCellNum = mBbox.width() / mCellSize;
      mHeightCellNum = mBbox.height() / mCellSize;
      mSDFData.resize(mWidthCellNum, mHeightCellNum,
                      (mHeightCellNum + mWidthCellNum) * mCellSize);
      mSDFClosestPoint.resize(mWidthCellNum, mHeightCellNum, Vector2D());
    }

    virtual ~PolygonSDF2D() {}

    inline void get_barycentric(double x, Int &i, double &f, Int i_low,
                                Int i_high)
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
    inline S bilerp(const S &v00, const S &v10, const S &v01, const S &v11, T fx,
                    T fy)
    {
      return lerp(lerp(v00, v10, fx), lerp(v01, v11, fx), fy);
    }

    std::vector<Vector2D> placeGridPoints()
    {
      std::vector<Vector2D> data;
      for (auto i = 0; i < mWidthCellNum; i++)
      {
        for (auto j = 0; j < mHeightCellNum; j++)
        {
          auto pos = mBbox.LowestPoint +
                     Vector2D((i + 0.5) * mCellSize, (j + 0.5) * mCellSize);
          auto [sdf_val, sdf_cloest_point] = getSDF(pos);
          if (sdf_val > 0.0)
          {
            data.emplace_back(pos);
          }
        }
      }
      return data;
    }

    void computeSDF()
    {
      for (auto i = 0; i < mWidthCellNum; i++)
      {
        for (auto j = 0; j < mHeightCellNum; j++)
        {
          auto pos = mBbox.LowestPoint + Vector2D(i * mCellSize, j * mCellSize);
          auto sgn = mPolygon->contains(pos) ? 1.0 : -1.0;
          auto [min_dist, min_dist_point] =
              mPolygon->computeMinDistInfoInPoly(pos);
          auto val = sgn * min_dist;
          mSDFClosestPoint(i, j) = min_dist_point;
          if (abs(val) < mSDFData(i, j))
          {
            mSDFData(i, j) = val;
          }

          if (val > mMaxSDFVal)
            mMaxSDFVal = val;
        }
      }
    }

    void updateSDFWithSpheres(std::vector<Vector3D> spheres)
    {
      for (auto i = 0; i < mWidthCellNum; i++)
      {
        for (auto j = 0; j < mHeightCellNum; j++)
        {
          auto pos = mBbox.LowestPoint + Vector2D(i * mCellSize, j * mCellSize);

          for (auto s = 0; s < spheres.size(); s++)
          {
            auto ss = spheres[s];
            auto dist_ps = (pos - Vector2D(ss.x, ss.y)).length() - ss.z;
            if (dist_ps < mSDFData(i, j))
            {
              mSDFData(i, j) = dist_ps;
              // KIRI_LOG_DEBUG("updated sdf ij={0}", mSDFData(i, j));
              mSDFClosestPoint(i, j) =
                  Vector2D(ss.x, ss.y) +
                  ss.z * (pos - Vector2D(ss.x, ss.y)).normalized();
            }
          }
        }
      }
    }

    void updateSDFWithSpheres(std::vector<Vector4D> spheres)
    {
      for (auto i = 0; i < mWidthCellNum; i++)
      {
        for (auto j = 0; j < mHeightCellNum; j++)
        {
          auto pos = mBbox.LowestPoint + Vector2D(i * mCellSize, j * mCellSize);

          for (auto s = 0; s < spheres.size(); s++)
          {
            auto ss = spheres[s];
            auto dist_ps = (pos - Vector2D(ss.x, ss.y)).length() - ss.z;
            if (dist_ps < mSDFData(i, j))
            {
              mSDFData(i, j) = dist_ps;
              // KIRI_LOG_DEBUG("updated sdf ij={0}", mSDFData(i, j));
              mSDFClosestPoint(i, j) =
                  Vector2D(ss.x, ss.y) +
                  ss.z * (pos - Vector2D(ss.x, ss.y)).normalized();
            }
          }
        }
      }
    }

    std::tuple<double, Vector2D> getSDF(Vector2D pos)
    {
      auto grid_pos = (pos - mBbox.LowestPoint) / mCellSize;

      Int i, j;
      double fi, fj;
      get_barycentric(grid_pos.x, i, fi, 0, mWidthCellNum);
      get_barycentric(grid_pos.y, j, fj, 0, mHeightCellNum);
      auto sdf_val = bilerp(mSDFData(i, j), mSDFData(i + 1, j),
                            mSDFData(i, j + 1), mSDFData(i + 1, j + 1), fi, fj);
      auto sdf_cloest_point = bilerp(
          mSDFClosestPoint(i, j), mSDFClosestPoint(i + 1, j),
          mSDFClosestPoint(i, j + 1), mSDFClosestPoint(i + 1, j + 1), fi, fj);
      return std::make_tuple(sdf_val, sdf_cloest_point);
    }

    std::tuple<double, Vector2D> getSDFWithRndOffset(Vector2D pos)
    {
      auto grid_pos = (pos - mBbox.LowestPoint) / mCellSize;

      Int i, j;
      double fi, fj;
      get_barycentric(grid_pos.x, i, fi, 0, mWidthCellNum);
      get_barycentric(grid_pos.y, j, fj, 0, mHeightCellNum);
      auto sdf_val = bilerp(mSDFData(i, j), mSDFData(i + 1, j),
                            mSDFData(i, j + 1), mSDFData(i + 1, j + 1), fi, fj);
      auto sdf_cloest_point = bilerp(
          mSDFClosestPoint(i, j), mSDFClosestPoint(i + 1, j),
          mSDFClosestPoint(i, j + 1), mSDFClosestPoint(i + 1, j + 1), fi, fj);

      if (sdf_val > 0.0)
      {
        auto rnd_offset = Random::get(0.1, 0.5) * mMaxSDFVal;
        auto step_num = sdf_val / rnd_offset;
        auto cell_sdf = std::ceil(step_num) * rnd_offset - sdf_val;
        auto floor_sdf = sdf_val - std::floor(step_num) * rnd_offset;
        sdf_val = std::min(std::min(sdf_val, floor_sdf), floor_sdf);
      }

      return std::make_tuple(sdf_val, sdf_cloest_point);
    }

  private:
    int mWidthCellNum, mHeightCellNum;
    double mCellSize;
    double mMaxSDFVal;
    BoundingBox2D mBbox;
    Array2D mSDFData;
    Array2Vec2D mSDFClosestPoint;
    Voronoi::VoronoiPolygon2Ptr mPolygon;
  };

  typedef std::shared_ptr<PolygonSDF2D> PolygonSDF2DPtr;

} // namespace HDV::SDF

#endif