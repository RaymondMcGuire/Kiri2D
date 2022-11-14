/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-14 12:26:31
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-14 12:27:10
 * @FilePath:
 * \Kiri2D\core\include\kiri2d\proto_sphere\proto_sphere_packing_sdf_opti.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _PROTO_SPHERE_PACK_SDF_OPTI_H_
#define _PROTO_SPHERE_PACK_SDF_OPTI_H_

#pragma once

#include <kiri2d/hdv_toolkit/sdf/sdf2d.h>

namespace PSPACK {
class ProtoSpherePackingSDFOpti {
public:
  explicit ProtoSpherePackingSDFOpti(
      const HDV::Voronoi::VoronoiPolygon2Ptr &boundary)
      : mBoundary(std::move(boundary)) {
    mSDF2D = std::make_shared<HDV::SDF::PolygonSDF2D>(mBoundary, 10.0);
    mSDF2D->computeSDF();
    initParticles();
  }

  virtual ~ProtoSpherePackingSDFOpti() {}

  const std::vector<Vector4D> &currentSpheres() const {
    return mCurrentSpheres;
  }
  const std::vector<Vector4D> &insertedSpheres() const {
    return mInsertedSpheres;
  }
  const std::vector<Vector3F> &insertedSpheresColor() const {
    return mInsertedSpheresColor;
  }

  const bool needDrawSpheres() const { return mDrawCurrentSpheres; }

  void statisticDistribution() {

    int a = 0, b = 0, c = 0, d = 0;
    double v = 0.0, rmspe = 0.0;
    for (auto i = 0; i < mInsertedSpheres.size(); i++) {
      if (mInsertedSpheres[i].z >= 1 && mInsertedSpheres[i].z <= 5)
        a++;

      if (mInsertedSpheres[i].z > 5 && mInsertedSpheres[i].z <= 10)
        b++;

      if (mInsertedSpheres[i].z > 10 && mInsertedSpheres[i].z <= 100)
        c++;

      if (mInsertedSpheres[i].z > 100)
        d++;

      v += KIRI_PI<double>() * mInsertedSpheres[i].z * mInsertedSpheres[i].z;
      rmspe += std::pow((mInsertedSpheres[i].z - mInsertedSpheres[i].w) /
                            mInsertedSpheres[i].w,
                        2.0);
    }

    rmspe = std::sqrt(rmspe / (double)mInsertedSpheres.size());

    KIRI_LOG_DEBUG(
        "1-5:{0}; 5-10:{1}; 10-100:{2}; 100:{3}; porosity={4}; rmspe={5}",
        a / (double)mInsertedSpheres.size(),
        b / (double)mInsertedSpheres.size(),
        c / (double)mInsertedSpheres.size(),
        d / (double)mInsertedSpheres.size(), 1.0 - v / mBoundary->area(),
        rmspe);
  }

  void reAllocateParticles() {
    if (mDrawCurrentSpheres) {
      mDrawCurrentSpheres = false;
      mSDF2D->updateSDFWithSpheres(mInsertedSpheres);
      initParticles();
      KIRI_LOG_DEBUG("re append particles!={0}; inserted number={1}; min "
                     "radius={2}; max radius={3}",
                     mCurrentSpheres.size(), mInsertedSpheres.size(),
                     mInsertedMinRadius, mInsertedMaxRadius);
      statisticDistribution();
    }
  }

  virtual void convergePrototype() {

    mAllConverged = true;
    for (auto i = 0; i < mCurrentSpheres.size(); i++) {
      if (mConverges[i])
        continue;
      else
        mAllConverged = false;

      // KIRI_LOG_DEBUG("itertaion!");

      auto sphere = mCurrentSpheres[i];
      auto target_radius = sphere.w;
      auto pos = Vector2D(sphere.x, sphere.y);
      auto [current_radius, q_c] = mSDF2D->getSDF(pos);

      auto radius_dist = target_radius - current_radius;

      mLearningRate[i] *= timeBasedDecay(mIterNums[i], 0.1);
      // mLearningRate[i] = expBasedDecay(0.1, mIterNums[i], 0.1);

      auto current_move =
          radius_dist * mLearningRate[i] * (pos - q_c).normalized();

      if (current_move.length() <= current_radius)
        pos += current_move;

      if (current_radius < 0.0)
        KIRI_LOG_ERROR("radius is minus!!!");

      if (abs(radius_dist) < target_radius * mErrorRate ||
          mIterNums[i] > mMaxSingleIter) {
        mConvergesTotalNum++;
        mConverges[i] = true;
      }

      mCurrentSpheres[i] = Vector4D(pos.x, pos.y, current_radius, sphere.w);
      mIterNums[i]++;
    }

    if (mAllConverged && !mDrawCurrentSpheres) {
      // KIRI_LOG_DEBUG("sort and add !");

      std::sort(mCurrentSpheres.begin(), mCurrentSpheres.end(),
                [](const auto &lhs, const auto &rhs) { return lhs.z > rhs.z; });

      for (auto i = 0; i < mCurrentSpheres.size(); i++) {
        auto cur_pos = Vector2D(mCurrentSpheres[i].x, mCurrentSpheres[i].y);
        auto cur_radius = mCurrentSpheres[i].z;
        auto tar_radius = mCurrentSpheres[i].w;
        if (cur_radius < 1.0 || cur_radius > 100.0)
          continue;

        auto overlapping = false;
        for (auto j = 0; j < mInsertedSpheres.size(); j++) {
          auto other_pos =
              Vector2D(mInsertedSpheres[j].x, mInsertedSpheres[j].y);
          auto other_radius = mInsertedSpheres[j].z;
          auto dist =
              (other_pos - cur_pos).length() - (cur_radius + other_radius);
          if (dist < 0) {
            overlapping = true;
            break;
          }
        }

        if (!overlapping) {
          mInsertedSpheres.emplace_back(mCurrentSpheres[i]);
          mInsertedSpheresColor.emplace_back(Vector3F(Random::get(0.0, 1.0),
                                                      Random::get(0.0, 1.0),
                                                      Random::get(0.0, 1.0)));
          if (mInsertedMaxRadius < mCurrentSpheres[i].z)
            mInsertedMaxRadius = mCurrentSpheres[i].z;

          if (mInsertedMinRadius > mCurrentSpheres[i].z)
            mInsertedMinRadius = mCurrentSpheres[i].z;
        }
      }

      mDrawCurrentSpheres = true;
    }
  }

protected:
  double timeBasedDecay(const int iter, const double decay) {
    return 1.0 / (1.0 + decay * iter);
  }

  double expBasedDecay(const double init, const int iter, const double k) {
    return init * std::exp(-k * iter);
  }

  void initParticles() {
    mCurrentSpheres.clear();
    mIterNums.clear();
    mConverges.clear();
    mLearningRate.clear();

    // predefine radius dist
    std::vector<double> radius_range;
    radius_range.push_back(1.0);
    radius_range.push_back(5.0);
    radius_range.push_back(10.0);
    radius_range.push_back(100.0);

    std::vector<double> radius_range_prob;
    radius_range_prob.push_back(0.7);
    radius_range_prob.push_back(0.2);
    radius_range_prob.push_back(0.1);

    std::random_device engine;
    std::mt19937 gen(engine());
    std::piecewise_constant_distribution<double> pcdis{
        std::begin(radius_range), std::end(radius_range),
        std::begin(radius_range_prob)};

    auto data = mSDF2D->placeGridPoints();
    KIRI_LOG_DEBUG("sphere array num={0}; data num={1}", mCurrentSpheres.size(),
                   data.size());

    for (auto i = 0; i < data.size(); i++) {
      mCurrentSpheres.emplace_back(
          Vector4D(data[i].x, data[i].y, 0.0, pcdis(gen)));
    }

    mIterNums.resize(mCurrentSpheres.size(), 0);
    mConverges.resize(mCurrentSpheres.size(), false);
    mLearningRate.resize(mCurrentSpheres.size(), 1.0);
  }

  bool mAllConverged = false, mDrawCurrentSpheres = false;
  double mErrorRate = 0.01;
  int mMaxSingleIter = 10000, mConvergesTotalNum = 0;
  double mInsertedMaxRadius = Tiny<double>(),
         mInsertedMinRadius = Huge<double>();

  std::vector<double> mLearningRate;
  std::vector<UInt> mIterNums;
  std::vector<bool> mConverges;
  std::vector<Vector4D> mCurrentSpheres;
  std::vector<Vector4D> mInsertedSpheres;
  std::vector<Vector3F> mInsertedSpheresColor;
  HDV::Voronoi::VoronoiPolygon2Ptr mBoundary;
  HDV::SDF::PolygonSDF2DPtr mSDF2D;
};

} // namespace PSPACK

#endif /* _PROTO_SPHERE_PACK_SDF_OPTI_H_ */