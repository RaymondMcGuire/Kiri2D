/*
 * File: improved_proto_sphere_packing_sdf.h
 * Module: proto_sphere
 * Created Date: 2025-08-18
 * Author: Xu WANG
 * -----
 * Last Modified: 2025-08-18
 * Modified By: Xu WANG
 * -----
 * Copyright (c) 2025 Xu WANG
 *
 * Improved version using soft-min approximation and adaptive line search
 * instead of cooling function for better convergence and parallelizability
 */

#ifndef _IMPROVED_PROTO_SPHERE_PACK_SDF_H_
#define _IMPROVED_PROTO_SPHERE_PACK_SDF_H_

#pragma once

#include <algorithm>
#include <cmath>
#include <kiri2d/hdv_toolkit/sdf/sdf2d.h>
#include <vector>

namespace PSPACK {
class ImprovedProtoSpherePackingSDF2D {
public:
  static constexpr int MAX_ITERATIONS = 10;
  static constexpr double CONVERGENCE_THRESHOLD = 1e-6;
  static constexpr double DEFAULT_CELL_SIZE = 10.0;
  static constexpr double DEFAULT_TAU =
      5.5; // Temperature parameter for soft-min
  static constexpr double LINE_SEARCH_ARMIJO_C1 =
      1e-4; // Armijo condition constant
  static constexpr double LINE_SEARCH_BACKTRACK_RHO =
      0.5; // Backtracking factor
  static constexpr double LINE_SEARCH_INITIAL_ALPHA = 1.0; // Initial step size
  static constexpr double MIN_ALPHA = 1e-10;               // Minimum step size

  explicit ImprovedProtoSpherePackingSDF2D(
      const HDV::Voronoi::VoronoiPolygon2Ptr &boundary,
      double cell_size = DEFAULT_CELL_SIZE, double tau = DEFAULT_TAU)
      : mBoundary(boundary), mTau(tau) {
    mSDF2D = std::make_shared<HDV::SDF::PolygonSDF2D>(mBoundary, cell_size);
    mSDF2D->computeSDF();
    initParticles();
  }

  virtual ~ImprovedProtoSpherePackingSDF2D() = default;

  const std::vector<Vector3D> &currentSpheres() const {
    return mCurrentSpheres;
  }

  const std::vector<Vector3D> &insertedSpheres() const {
    return mInsertedSpheres;
  }

  const std::vector<Vector3F> &insertedSpheresColor() const {
    return mInsertedSpheresColor;
  }

  void setTau(double tau) { mTau = tau; }

  virtual void convergePrototype() {
    if (mDrawCurrentSpheres) {
      mDrawCurrentSpheres = false;
      // Update SDF with inserted spheres for next iteration
      mSDF2D->updateSDFWithSpheres(mInsertedSpheres);
      initParticles();
      KIRI_LOG_DEBUG("re append particles!={0}; inserted number={1}",
                     mCurrentSpheres.size(), mInsertedSpheres.size());
    }

    mAllConverged = true;

    // Parallelize this loop for better performance
    for (size_t i = 0; i < mCurrentSpheres.size(); ++i) {
      if (mConverges[i] || mIterNums[i] > MAX_ITERATIONS)
        continue;

      mAllConverged = false;
      updateSpherePositionWithLineSearch(i);
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

  // Compute soft-min radius using log-sum-exp trick for numerical stability
  double computeSoftMinRadius(const Vector2D &pos) const {
    double min_dist = std::numeric_limits<double>::max();

    // Get distance to polygon boundary
    auto [d_poly, _] = mSDF2D->getSDF(pos);
    min_dist = std::min(min_dist, d_poly);

    // Get distances to existing spheres
    for (const auto &sphere : mInsertedSpheres) {
      Vector2D sphere_center(sphere.x, sphere.y);
      double d_sphere = (pos - sphere_center).length() - sphere.z;
      min_dist = std::min(min_dist, d_sphere);
    }

    // Use log-sum-exp for numerical stability
    // First find the minimum for shifting

    // !Trick
    double shift = -min_dist / mTau;

    // Compute sum of exponentials
    double sum_exp = 0.0;

    // Add polygon boundary term
    sum_exp += std::exp(-d_poly / mTau - shift);

    // Add sphere terms
    for (const auto &sphere : mInsertedSpheres) {
      Vector2D sphere_center(sphere.x, sphere.y);
      double d_sphere = (pos - sphere_center).length() - sphere.z;
      sum_exp += std::exp(-d_sphere / mTau - shift);
    }

    // Return soft-min approximation
    return -mTau * (std::log(sum_exp) + shift);
  }

  // Compute gradient of soft-min radius function
  Vector2D computeSoftMinGradient(const Vector2D &pos) const {
    Vector2D gradient(0.0, 0.0);
    double sum_weights = 0.0;

    // Get polygon boundary distance and gradient
    auto [d_poly, _] = mSDF2D->getSDF(pos);
    Vector2D grad_poly = mSDF2D->getSDFGradient(pos);

    // Compute weights using soft-min
    double w_poly = std::exp(-d_poly / mTau);
    sum_weights += w_poly;
    gradient += w_poly * grad_poly;

    // Add contributions from existing spheres
    for (const auto &sphere : mInsertedSpheres) {
      Vector2D sphere_center(sphere.x, sphere.y);
      Vector2D diff = pos - sphere_center;
      double dist = diff.length();

      // Avoid singularity at sphere center
      if (dist < 1e-10)
        continue;

      double d_sphere = dist - sphere.z;
      double w_sphere = std::exp(-d_sphere / mTau);
      Vector2D grad_sphere =
          diff / dist; // Unit vector pointing away from sphere

      sum_weights += w_sphere;
      gradient += w_sphere * grad_sphere;
    }

    // Normalize by sum of weights
    if (sum_weights > 1e-10) {
      gradient /= sum_weights;
    }

    return gradient;
  }

  // Backtracking line search with Armijo condition
  Vector2D lineSearch(const Vector2D &pos, const Vector2D &gradient) const {
    double alpha = LINE_SEARCH_INITIAL_ALPHA;

    // Current objective value
    double f0 = computeSoftMinRadius(pos);

    // Directional derivative (gradient in ascent direction)
    double grad_dot_dir = gradient.lengthSquared();

    // If gradient is too small, don't move
    if (grad_dot_dir < 1e-20) {
      return pos;
    }

    // Backtracking line search
    while (alpha > MIN_ALPHA) {
      // Try new position
      Vector2D new_pos = pos + alpha * gradient;

      // Check if new position is valid (inside boundary)
      auto [new_dist, _] = mSDF2D->getSDF(new_pos);
      if (new_dist <= 0) {
        // Outside boundary, reduce step size
        alpha *= LINE_SEARCH_BACKTRACK_RHO;
        continue;
      }

      // Compute new objective value
      double f_new = computeSoftMinRadius(new_pos);

      // Check Armijo condition for sufficient increase
      // Note: We're maximizing, so we check for increase
      if (f_new >= f0 + LINE_SEARCH_ARMIJO_C1 * alpha * grad_dot_dir) {
        return new_pos; // Accept this step
      }

      // Reduce step size and try again
      alpha *= LINE_SEARCH_BACKTRACK_RHO;
    }

    // No improvement found, return original position
    return pos;
  }

  // Update sphere position using gradient ascent with line search
  void updateSpherePositionWithLineSearch(size_t index) {
    auto &sphere = mCurrentSpheres[index];
    Vector2D pos(sphere.x, sphere.y);

    // Check if already outside boundary
    auto [boundary_dist, _] = mSDF2D->getSDF(pos);
    if (boundary_dist <= 0) {
      mConverges[index] = true;
      return;
    }

    // Compute gradient of soft-min radius
    Vector2D gradient = computeSoftMinGradient(pos);

    // Check convergence based on gradient magnitude
    if (gradient.length() < CONVERGENCE_THRESHOLD) {
      mConverges[index] = true;
      sphere.z = computeSoftMinRadius(pos);
      return;
    }

    // Perform line search to find optimal step size
    Vector2D new_pos = lineSearch(pos, gradient);

    // Update position and radius
    double new_radius = computeSoftMinRadius(new_pos);
    sphere = Vector3D(new_pos.x, new_pos.y, new_radius);

    // Increment iteration counter
    mIterNums[index]++;

    // Check if position changed significantly
    if ((new_pos - pos).length() < CONVERGENCE_THRESHOLD) {
      mConverges[index] = true;
    }
  }

  void processFinalSpheres() {

    int count_converge = 0;
    for (const auto &converged : mConverges) {
      if (converged) {
        count_converge++;
      }
    }
    // printf("Converged spheres: %d / %d\n", count_converge,
    // mConverges.size());

    // Sort spheres by radius in descending order (larger spheres first)
    std::sort(
        mCurrentSpheres.begin(), mCurrentSpheres.end(),
        [](const Vector3D &lhs, const Vector3D &rhs) { return lhs.z > rhs.z; });

    size_t estimated_size =
        mInsertedSpheres.size() + mCurrentSpheres.size() / 2;
    mInsertedSpheres.reserve(estimated_size);
    mInsertedSpheresColor.reserve(estimated_size);

    // Insert non-overlapping spheres
    for (const auto &current : mCurrentSpheres) {
      if (!checkOverlapping(current)) {
        mInsertedSpheres.push_back(current);

        // printf("Inserted sphere radius: %f\n", current.z);

        // Assign random color for visualization
        mInsertedSpheresColor.emplace_back(Vector3F(Random::get(0.0, 1.0),
                                                    Random::get(0.0, 1.0),
                                                    Random::get(0.0, 1.0)));
      }
    }
  }

  bool checkOverlapping(const Vector3D &current) const {
    Vector2D cur_pos(current.x, current.y);
    double cur_radius = current.z;
    if (cur_radius < 1e-6) {
      return true; // Ignore spheres with negligible radius
    }

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

private:
  bool mAllConverged = false;
  bool mDrawCurrentSpheres = false;

  std::vector<unsigned int> mIterNums;
  std::vector<bool> mConverges;

  std::vector<Vector3D> mCurrentSpheres;  // Current spheres being optimized
  std::vector<Vector3D> mInsertedSpheres; // Successfully placed spheres
  std::vector<Vector3F> mInsertedSpheresColor; // Colors for visualization

  double mTau; // Temperature parameter for soft-min approximation

  HDV::Voronoi::VoronoiPolygon2Ptr mBoundary;
  HDV::SDF::PolygonSDF2DPtr mSDF2D;
};

} // namespace PSPACK

#endif /* _IMPROVED_PROTO_SPHERE_PACK_SDF_H_ */