/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-14 12:26:31
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-15 11:54:33
 * @FilePath:
 * \Kiri2D\core\include\kiri2d\proto_sphere\proto_sphere_packing_sdf_opti.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _PROTO_SPHERE_PACK_SDF_OPTI_H_
#define _PROTO_SPHERE_PACK_SDF_OPTI_H_

#pragma once

#include <kiri2d/hdv_toolkit/sdf/sdf2d.h>
std::vector<double> mCurrentRadiusRange, mPreDefinedRadiusRange;
std::vector<double> mCurrentRadiusRangeProb, mPreDefinedRadiusRangeProb;
namespace PSPACK {
class ProtoSpherePackingSDFOpti {
public:
  explicit ProtoSpherePackingSDFOpti(
      const HDV::Voronoi::VoronoiPolygon2Ptr &boundary,
      const std::vector<double> radiusRange,
      const std::vector<double> radiusRangeProb)
      : mBoundary(std::move(boundary)), mCurrentRadiusRange(radiusRange),
        mPreDefinedRadiusRange(radiusRange),
        mCurrentRadiusRangeProb(radiusRangeProb),
        mPreDefinedRadiusRangeProb(radiusRangeProb) {
    mSDF2D = std::make_shared<HDV::SDF::PolygonSDF2D>(mBoundary, 10.0);
    mSDF2D->computeSDF();

    estimateIdealTotalNum();

    // realloc generator: method2 failed
    mCurrentRadiusRangeProb[mCurrentRadiusRangeProb.size() - 1] = 1.0;
    if (mCurrentRadiusRangeProb.size() - 1 != 0)
      for (int j = mCurrentRadiusRangeProb.size() - 1 - 1; j >= 0; j--)
        mCurrentRadiusRangeProb[j] = 0.0;

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

    std::vector<int> counter;
    counter.resize(mPreDefinedRadiusRange.size() - 1, 0);
    double v = 0.0, rmspe = 0.0;
    for (auto i = 0; i < mInsertedSpheres.size(); i++) {

      for (auto ri = 0; ri < mPreDefinedRadiusRange.size() - 1; ri++) {
        auto rj = ri + 1;
        if (mInsertedSpheres[i].z > mPreDefinedRadiusRange[ri] &&
            mInsertedSpheres[i].z <= mPreDefinedRadiusRange[rj])
          counter[ri]++;
      }

      v += KIRI_PI<double>() * mInsertedSpheres[i].z * mInsertedSpheres[i].z;
      rmspe += std::pow((mInsertedSpheres[i].z - mInsertedSpheres[i].w) /
                            mInsertedSpheres[i].w,
                        2.0);
    }

    rmspe = std::sqrt(rmspe / (double)mInsertedSpheres.size());
    std::string distribution_str = "";
    for (auto ri = 0; ri < mPreDefinedRadiusRange.size() - 1; ri++)
      distribution_str +=
          std::to_string((int)mPreDefinedRadiusRange[ri]) + "---" +
          std::to_string((int)mPreDefinedRadiusRange[ri + 1]) + ":" +
          std::to_string(counter[ri] / (double)mInsertedSpheres.size()) + "; ";
    KIRI_LOG_DEBUG("{0}; porosity={1}; rmspe={2}", distribution_str,
                   1.0 - v / mBoundary->area(), rmspe);

    auto sum_remain = 0;
    for (auto i = 0; i < counter.size(); i++) {
      auto remain_num =
          mRemainSamples[i] -
          (counter[i] - mRemainSamples[i] * mInsertedIntervalCounter);
      remain_num = remain_num > 0 ? remain_num : 0;
      sum_remain += remain_num;

      KIRI_LOG_DEBUG(
          "dist:{0}-{1}; remain samples num={2}; remain iter num={3}",
          mCurrentRadiusRange[i], mCurrentRadiusRange[i + 1], remain_num,
          mInsertedIntervalCounter);
    }

    if (sum_remain == 0) {
      mInsertedIntervalCounter++;
    } else {

      // adjust distribution generator based on remained particles: failed!
      // for (auto i = 0; i < mCurrentRadiusRangeProb.size(); i++) {
      //   auto remain_num = mRemainSamples[i] - counter[i];
      //   remain_num = remain_num > 0 ? remain_num : 0;
      //   mCurrentRadiusRangeProb[i] = remain_num / (double)sum_remain;
      //   KIRI_LOG_DEBUG("new dist:{0}-{1}:{2}; remain samples num={3}",
      //                  mCurrentRadiusRange[i], mCurrentRadiusRange[i + 1],
      //                  mCurrentRadiusRangeProb[i], remain_num);
      // }

      // method2 failed
      for (int i = mCurrentRadiusRangeProb.size() - 1; i >= 0; i--) {
        auto remain_num =
            mRemainSamples[i] -
            (counter[i] - mRemainSamples[i] * mInsertedIntervalCounter);
        remain_num = remain_num > 0 ? remain_num : 0;
        if (remain_num == 0) {
          mCurrentRadiusRangeProb[i] = 0.0;
          continue;
        }

        mCurrentRadiusRangeProb[i] = 1.0;
        if (i != 0)
          for (int j = i - 1; j >= 0; j--)
            mCurrentRadiusRangeProb[j] = 0.0;

        break;
      }

      // for (auto i = 0; i < mCurrentRadiusRangeProb.size(); i++) {
      //   auto remain_num = mRemainSamples[i] - counter[i];
      //   remain_num = remain_num > 0 ? remain_num : 0;
      //   if (remain_num == 0)
      //     mCurrentRadiusRangeProb[i] = 0.0;

      //   KIRI_LOG_DEBUG("new dist:{0}-{1}:{2}; remain samples num={3}",
      //                  mCurrentRadiusRange[i], mCurrentRadiusRange[i + 1],
      //                  mCurrentRadiusRangeProb[i], remain_num);
      // }
    }
  }

  void reAllocateParticles() {
    if (mDrawCurrentSpheres) {
      mDrawCurrentSpheres = false;
      mSDF2D->updateSDFWithSpheres(mInsertedSpheres);
      KIRI_LOG_DEBUG("re append particles!={0}; inserted number={1}; min "
                     "radius={2}; max radius={3}",
                     mCurrentSpheres.size(), mInsertedSpheres.size(),
                     mInsertedMinRadius, mInsertedMaxRadius);
      statisticDistribution();
      initParticles();
    }
  }

  virtual void convergePrototype() {

    if (mInsertedIntervalCounter > mInsertedIntervalNum)
      return;

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

      // compute current sphere radius
      auto [current_radius, q_c] = mSDF2D->getSDF(pos);

      // radius is not correct
      if (current_radius < 0.0) {
        mConvergesTotalNum++;
        mConverges[i] = true;
        mCurrentSpheres[i] = mLastSpheres[i];
        continue;
      }

      mCurrentSpheres[i] =
          Vector4D(pos.x, pos.y, current_radius, target_radius);

      // check sphere radius is converges or not
      auto radius_dist = target_radius - current_radius;
      if (abs(radius_dist) < target_radius * mErrorRate ||
          mIterNums[i] > mMaxSingleIter) {
        mConvergesTotalNum++;
        mConverges[i] = true;
        continue;
      }

      mLearningRate[i] *= timeBasedDecay(mIterNums[i], 0.1);
      // mLearningRate[i] = expBasedDecay(0.1, mIterNums[i], 0.1);

      auto current_move =
          radius_dist * mLearningRate[i] * (pos - q_c).normalized();

      if (current_move.length() <= current_radius)
        pos += current_move;

      mLastSpheres[i] = mCurrentSpheres[i];
      mCurrentSpheres[i] = Vector4D(pos.x, pos.y, 0.0, sphere.w);
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

        if (!overlapping &&
            abs(cur_radius - tar_radius) < tar_radius * mErrorRate) {
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
  void estimateIdealTotalNum() {
    auto area = mBoundary->area();
    auto denominator = 0.0;
    for (auto ri = 0; ri < mPreDefinedRadiusRange.size() - 1; ri++) {
      auto rj = ri + 1;
      auto avg_radius =
          0.5 * (mPreDefinedRadiusRange[ri] + mPreDefinedRadiusRange[rj]);
      denominator += avg_radius * avg_radius * KIRI_PI<double>() *
                     mPreDefinedRadiusRangeProb[ri];
    }
    mIdealTotalNum = area / denominator;
    KIRI_LOG_DEBUG("ideal total num={0}", mIdealTotalNum);

    mRemainSamples.resize(mPreDefinedRadiusRangeProb.size(), 0);
    for (auto i = 0; i < mPreDefinedRadiusRangeProb.size(); i++) {
      mRemainSamples[i] = int(mIdealTotalNum * mPreDefinedRadiusRangeProb[i] /
                              mInsertedIntervalNum);
    }
  }

  double timeBasedDecay(const int iter, const double decay) {
    return 1.0 / (1.0 + decay * iter);
  }

  double expBasedDecay(const double init, const int iter, const double k) {
    return init * std::exp(-k * iter);
  }

  void initParticles() {
    mCurrentSpheres.clear();
    mLastSpheres.clear();
    mIterNums.clear();
    mConverges.clear();
    mLearningRate.clear();

    std::random_device engine;
    std::mt19937 gen(engine());
    std::piecewise_constant_distribution<double> pcdis{
        std::begin(mCurrentRadiusRange), std::end(mCurrentRadiusRange),
        std::begin(mCurrentRadiusRangeProb)};

    auto data = mSDF2D->placeGridPoints();

    for (auto i = 0; i < data.size(); i++) {
      auto new_sphere = Vector4D(data[i].x, data[i].y, 0.0, pcdis(gen));
      mCurrentSpheres.emplace_back(new_sphere);
      mLastSpheres.emplace_back(new_sphere);
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

  int mIdealTotalNum = 0;

  int mInsertedIntervalCounter = 0, mInsertedIntervalNum = 10;
  std::vector<int> mRemainSamples;

  std::vector<double> mLearningRate;
  std::vector<UInt> mIterNums;
  std::vector<bool> mConverges;
  std::vector<Vector4D> mCurrentSpheres, mLastSpheres;
  std::vector<Vector4D> mInsertedSpheres;
  std::vector<Vector3F> mInsertedSpheresColor;
  std::vector<double> mCurrentRadiusRange, mPreDefinedRadiusRange;
  std::vector<double> mCurrentRadiusRangeProb, mPreDefinedRadiusRangeProb;
  HDV::Voronoi::VoronoiPolygon2Ptr mBoundary;
  HDV::SDF::PolygonSDF2DPtr mSDF2D;
};

} // namespace PSPACK

#endif /* _PROTO_SPHERE_PACK_SDF_OPTI_H_ */