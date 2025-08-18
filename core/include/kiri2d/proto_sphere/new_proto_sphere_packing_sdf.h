/*
 * File: new_proto_sphere_packing_sdf.h
 * Module: proto_sphere
 * Created Date: 2025-08-18
 * Author: Xu WANG
 * -----
 * Last Modified: 2025-08-18
 * Modified By: Xu WANG
 * -----
 * Copyright (c) 2025 Xu WANG
 */

#ifndef _NEW_PROTO_SPHERE_PACK_SDF_H_
#define _NEW_PROTO_SPHERE_PACK_SDF_H_

#pragma once

#include <algorithm>
#include <kiri2d/hdv_toolkit/sdf/sdf2d.h>
#include <vector>

namespace PSPACK {
class NewProtoSpherePackingSDF2D {
public:
  static constexpr int MAX_ITERATIONS = 50;
  static constexpr double CONVERGENCE_THRESHOLD = 1e-6;
  static constexpr double DEFAULT_COOLING_INIT = 0.08;
  static constexpr double DEFAULT_COOLING_K = 0.8;
  static constexpr double DEFAULT_CELL_SIZE = 10.0;

  explicit NewProtoSpherePackingSDF2D(
      const HDV::Voronoi::VoronoiPolygon2Ptr &boundary,
      double cell_size = DEFAULT_CELL_SIZE,
      double cooling_init = DEFAULT_COOLING_INIT,
      double cooling_k = DEFAULT_COOLING_K)
      : mBoundary(boundary), mCoolingInit(cooling_init), mCoolingK(cooling_k) {
    mSDF2D = std::make_shared<HDV::SDF::PolygonSDF2D>(mBoundary, cell_size);
    mSDF2D->computeSDF();
    initParticles();
  }

  virtual ~NewProtoSpherePackingSDF2D() = default;

  const std::vector<Vector3D> &currentSpheres() const {
    return mCurrentSpheres;
  }

  const std::vector<Vector3D> &insertedSpheres() const {
    return mInsertedSpheres;
  }

  const std::vector<Vector3F> &insertedSpheresColor() const {
    return mInsertedSpheresColor;
  }

  void setCoolingParameters(double init, double k) {
    mCoolingInit = init;
    mCoolingK = k;
  }

  virtual void convergePrototype() {
    if (mDrawCurrentSpheres) {
      mDrawCurrentSpheres = false;
      mSDF2D->updateSDFWithSpheres(mInsertedSpheres);
      initParticles();
      KIRI_LOG_DEBUG("re append particles!={0}; inserted number={1}",
                     mCurrentSpheres.size(), mInsertedSpheres.size());
    }

    mAllConverged = true;

    for (size_t i = 0; i < mCurrentSpheres.size(); ++i) {
      if (mConverges[i] || mIterNums[i] > MAX_ITERATIONS)
        continue;

      mAllConverged = false;

      updateSpherePosition(i);
    }

    if (mAllConverged && !mDrawCurrentSpheres) {
      processFinalSpheres();
      mDrawCurrentSpheres = true;
    }
  }

protected:
  void initParticles() {
    mCurrentSpheres.clear();
    mIterNums.clear();
    mConverges.clear();

    auto data = mSDF2D->placeGridPoints();
    KIRI_LOG_DEBUG("sphere array num={0}; data num={1}", mCurrentSpheres.size(),
                   data.size());

    mCurrentSpheres.reserve(data.size());
    for (const auto &point : data) {
      mCurrentSpheres.emplace_back(point.x, point.y, 0.0);
    }

    mIterNums.resize(mCurrentSpheres.size(), 0);
    mConverges.resize(mCurrentSpheres.size(), false);
  }

  void updateSpherePosition(size_t index) {
    auto &sphere = mCurrentSpheres[index];
    Vector2D pos(sphere.x, sphere.y);

    auto [min_dist, q_c] = mSDF2D->getSDF(pos);

    if (min_dist <= 0) {
      mConverges[index] = true;
      return;
    }

    double epsilon = coolingFunc(mIterNums[index], mCoolingInit, mCoolingK);
    Vector2D current_move = epsilon * (pos - q_c);
    double current_move_len = current_move.length();

    if (current_move_len < CONVERGENCE_THRESHOLD) {
      mConverges[index] = true;
      sphere.z = min_dist;
      return;
    }

    if (current_move_len <= min_dist) {
      pos += current_move;
    }

    sphere = Vector3D(pos.x, pos.y, min_dist);
    mIterNums[index]++;
  }

  void processFinalSpheres() {
    std::sort(
        mCurrentSpheres.begin(), mCurrentSpheres.end(),
        [](const Vector3D &lhs, const Vector3D &rhs) { return lhs.z > rhs.z; });
    size_t estimated_size =
        mInsertedSpheres.size() + mCurrentSpheres.size() / 2;
    mInsertedSpheres.reserve(estimated_size);
    mInsertedSpheresColor.reserve(estimated_size);

    for (const auto &current : mCurrentSpheres) {
      if (!checkOverlapping(current)) {
        mInsertedSpheres.push_back(current);
        mInsertedSpheresColor.emplace_back(Random::get(0.0, 1.0),
                                           Random::get(0.0, 1.0),
                                           Random::get(0.0, 1.0));
      }
    }
  }

  bool checkOverlapping(const Vector3D &current) const {
    Vector2D cur_pos(current.x, current.y);
    double cur_radius = current.z;

    for (const auto &other : mInsertedSpheres) {
      Vector2D other_pos(other.x, other.y);
      double other_radius = other.z;

      double dist_sq = (other_pos - cur_pos).lengthSquared();
      double threshold_sq =
          (cur_radius + other_radius) * (cur_radius + other_radius);

      if (dist_sq < threshold_sq) {
        return true;
      }
    }
    return false;
  }

  virtual double coolingFunc(int iter, double init, double k) const {
    return init * std::exp(-k * iter);
  }

private:
  bool mAllConverged = false;
  bool mDrawCurrentSpheres = false;

  std::vector<unsigned int> mIterNums;
  std::vector<bool> mConverges;

  std::vector<Vector3D> mCurrentSpheres;
  std::vector<Vector3D> mInsertedSpheres;
  std::vector<Vector3F> mInsertedSpheresColor;

  double mCoolingInit;
  double mCoolingK;

  HDV::Voronoi::VoronoiPolygon2Ptr mBoundary;
  HDV::SDF::PolygonSDF2DPtr mSDF2D;
};

} // namespace PSPACK

#endif /* _PROTO_SPHERE_PACK_SDF_H_ */