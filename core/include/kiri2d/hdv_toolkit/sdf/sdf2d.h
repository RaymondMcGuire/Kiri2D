/*
 * File: sdf2d.h
 * Module: sdf
 * Created Date: 2025-08-18
 * Author: Xu WANG
 * -----
 * Last Modified: 2025-08-18
 * Modified By: Xu WANG
 * -----
 * Copyright (c) 2025 Xu WANG
 */

#ifndef _HDV_SDF_POLYGON2D_H_
#define _HDV_SDF_POLYGON2D_H_

#pragma once

#include <algorithm>
#include <kiri2d/hdv_toolkit/voronoi/voronoi_polygon.h>
#include <limits>
#include <stdexcept>

namespace HDV::SDF {

/**
 * @brief A class for computing and managing 2D Signed Distance Fields for
 * polygons
 *
 * This class provides functionality to:
 * - Compute SDF values on a regular grid
 * - Update SDF with sphere obstacles
 * - Query SDF values at arbitrary positions using bilinear interpolation
 * - Compute SDF gradients for normal and force calculations
 * - Generate grid points inside the polygon
 */
class PolygonSDF2D {
public:
  /**
   * @brief Constructor for PolygonSDF2D
   * @param polygon Shared pointer to the polygon geometry
   * @param cellSize Size of each grid cell for SDF computation
   * @throws std::invalid_argument if polygon is null or cellSize <= 0
   */
  explicit PolygonSDF2D(const Voronoi::VoronoiPolygon2Ptr &polygon,
                        double cellSize)
      : mPolygon(polygon), mCellSize(cellSize), mMaxSDFVal(0.0) {

    // Validate input parameters
    if (!polygon) {
      throw std::invalid_argument("Polygon pointer cannot be null");
    }
    if (cellSize <= 0) {
      throw std::invalid_argument("Cell size must be positive");
    }

    // Initialize bounding box and grid dimensions
    mBbox = mPolygon->bbox();
    mWidthCellNum = static_cast<int>(std::ceil(mBbox.width() / mCellSize));
    mHeightCellNum = static_cast<int>(std::ceil(mBbox.height() / mCellSize));

    // Initialize SDF data arrays with maximum possible distance
    const double maxInitialValue = std::numeric_limits<double>::max();
    mSDFData.resize(mWidthCellNum, mHeightCellNum, maxInitialValue);
    mSDFClosestPoint.resize(mWidthCellNum, mHeightCellNum, Vector2D());
  }

  virtual ~PolygonSDF2D() = default;

  /**
   * @brief Compute the signed distance field for the polygon
   *
   * Positive values indicate points inside the polygon,
   * negative values indicate points outside
   */
  void computeSDF() {
    const Vector2D basePoint = mBbox.LowestPoint;

    // Reset max SDF value
    mMaxSDFVal = -std::numeric_limits<double>::max();

    // Compute SDF for each grid point
    for (int i = 0; i < mWidthCellNum; ++i) {
      for (int j = 0; j < mHeightCellNum; ++j) {
        // Calculate world position of grid point
        const Vector2D pos = basePoint + Vector2D(i * mCellSize, j * mCellSize);

        // Determine sign based on point containment
        const double sign = mPolygon->contains(pos) ? 1.0 : -1.0;

        // Compute minimum distance to polygon boundary
        const auto [minDist, minDistPoint] =
            mPolygon->computeMinDistInfoInPoly(pos);

        // Calculate signed distance value
        const double sdfValue = sign * minDist;

        // Store results
        mSDFData(i, j) = sdfValue;
        mSDFClosestPoint(i, j) = minDistPoint;

        mMaxSDFVal = std::max(mMaxSDFVal, sdfValue);
      }
    }
  }

  /**
   * @brief Update SDF field with sphere obstacles
   * @param spheres Vector of spheres (x, y, radius) or (x, y, radius, w)
   *
   * This method modifies the SDF to account for sphere obstacles,
   * taking the minimum distance between the original SDF and sphere surfaces
   */
  template <typename SphereType>
  void updateSDFWithSpheres(const std::vector<SphereType> &spheres) {
    const Vector2D basePoint = mBbox.LowestPoint;

    for (int i = 0; i < mWidthCellNum; ++i) {
      for (int j = 0; j < mHeightCellNum; ++j) {
        const Vector2D pos = basePoint + Vector2D(i * mCellSize, j * mCellSize);

        // Check distance to each sphere
        for (const auto &sphere : spheres) {
          const Vector2D sphereCenter(sphere.x, sphere.y);
          const double sphereRadius = sphere.z;

          // Calculate signed distance to sphere surface
          const Vector2D toPoint = pos - sphereCenter;
          const double distToCenter = toPoint.length();
          const double distToSurface = distToCenter - sphereRadius;

          // Update SDF if sphere provides closer boundary
          if (distToSurface < mSDFData(i, j)) {
            mSDFData(i, j) = distToSurface;

            // Update closest point on sphere surface
            if (distToCenter > 1e-10) { // Avoid division by zero
              mSDFClosestPoint(i, j) =
                  sphereCenter + sphereRadius * (toPoint / distToCenter);
            } else {
              // Point is at sphere center, any point on surface is valid
              mSDFClosestPoint(i, j) = sphereCenter + Vector2D(sphereRadius, 0);
            }
          }
        }
      }
    }
  }

  /**
   * @brief Get SDF value and closest point at arbitrary position using bilinear
   * interpolation
   * @param pos Query position in world coordinates
   * @return Tuple of (SDF value, closest point on boundary)
   */
  std::tuple<double, Vector2D> getSDF(const Vector2D &pos) const {
    // Convert world position to grid coordinates
    const Vector2D gridPos = worldToGrid(pos);

    // Get interpolation indices and factors
    int i, j;
    double fi, fj;
    getBarycentric(gridPos.x, i, fi, 0, mWidthCellNum);
    getBarycentric(gridPos.y, j, fj, 0, mHeightCellNum);

    // Bilinear interpolation of SDF value
    const double sdfValue =
        bilerp(mSDFData(i, j), mSDFData(i + 1, j), mSDFData(i, j + 1),
               mSDFData(i + 1, j + 1), fi, fj);

    // Bilinear interpolation of closest point
    const Vector2D closestPoint = bilerp(
        mSDFClosestPoint(i, j), mSDFClosestPoint(i + 1, j),
        mSDFClosestPoint(i, j + 1), mSDFClosestPoint(i + 1, j + 1), fi, fj);

    return std::make_tuple(sdfValue, closestPoint);
  }

  /**
   * @brief Compute the gradient of the SDF at a given position using central
   * differences
   * @param pos Query position in world coordinates
   * @return Gradient vector (∂SDF/∂x, ∂SDF/∂y)
   *
   * The gradient points in the direction of increasing SDF value.
   * For points on the boundary, this gives the outward normal.
   * The magnitude of the gradient should be approximately 1 for a proper SDF.
   */
  Vector2D getSDFGradient(const Vector2D &pos) const {
    // Use central differences with a small epsilon
    const double eps = mCellSize * 0.5;

    // Compute SDF values at neighboring points
    const auto [sdfXPlus, _1] = getSDF(pos + Vector2D(eps, 0));
    const auto [sdfXMinus, _2] = getSDF(pos - Vector2D(eps, 0));
    const auto [sdfYPlus, _3] = getSDF(pos + Vector2D(0, eps));
    const auto [sdfYMinus, _4] = getSDF(pos - Vector2D(0, eps));

    // Central difference approximation
    const double gradX = (sdfXPlus - sdfXMinus) / (2.0 * eps);
    const double gradY = (sdfYPlus - sdfYMinus) / (2.0 * eps);

    return Vector2D(gradX, gradY);
  }

  /**
   * @brief Compute the normalized gradient (unit normal) of the SDF
   * @param pos Query position in world coordinates
   * @return Unit normal vector pointing outward from the surface
   *
   * This is useful for collision response, rendering, and physics simulations.
   * Returns zero vector if gradient magnitude is too small.
   */
  Vector2D getSDFNormal(const Vector2D &pos) const {
    Vector2D gradient = getSDFGradient(pos);
    const double mag = gradient.length();

    // Avoid division by zero for very small gradients
    if (mag > 1e-10) {
      return gradient / mag;
    }

    return Vector2D(0, 0);
  }

  /**
   * @brief Compute SDF gradient using bilinear interpolation of grid gradients
   * @param pos Query position in world coordinates
   * @return Interpolated gradient vector
   *
   * This method is faster than getSDFGradient() but may be less accurate
   * at boundaries between grid cells.
   */
  Vector2D getSDFGradientInterpolated(const Vector2D &pos) const {
    // Convert world position to grid coordinates
    const Vector2D gridPos = worldToGrid(pos);

    // Get interpolation indices and factors
    int i, j;
    double fi, fj;
    getBarycentric(gridPos.x, i, fi, 0, mWidthCellNum);
    getBarycentric(gridPos.y, j, fj, 0, mHeightCellNum);

    // Compute gradients at grid corners using forward/backward differences
    Vector2D grad00 = computeGridGradient(i, j);
    Vector2D grad10 = computeGridGradient(i + 1, j);
    Vector2D grad01 = computeGridGradient(i, j + 1);
    Vector2D grad11 = computeGridGradient(i + 1, j + 1);

    // Bilinear interpolation of gradient
    return bilerp(grad00, grad10, grad01, grad11, fi, fj);
  }

  /**
   * @brief Compute analytical gradient using closest point information
   * @param pos Query position in world coordinates
   * @return Analytical gradient vector
   *
   * This method uses the fact that the gradient of an SDF points from
   * the closest point on the boundary to the query point.
   * Most accurate near the boundary.
   */
  Vector2D getSDFGradientAnalytical(const Vector2D &pos) const {
    const auto [sdfValue, closestPoint] = getSDF(pos);

    // Gradient points from closest point to query point
    Vector2D gradient = pos - closestPoint;
    const double dist = gradient.length();

    // Normalize to get unit gradient
    if (dist > 1e-10) {
      gradient = gradient / dist;

      // Ensure correct sign based on SDF value
      if (sdfValue < 0) {
        gradient = -gradient;
      }
    } else {
      // Point is on the boundary, gradient is undefined
      gradient = Vector2D(0, 0);
    }

    return gradient;
  }

  /**
   * @brief Get SDF with random offset for artistic/stylized effects
   * @param pos Query position in world coordinates
   * @return Tuple of (modified SDF value, closest point on boundary)
   *
   * This method applies a stepping function to create contour-like effects
   * in the positive SDF region (inside the polygon)
   */
  std::tuple<double, Vector2D> getSDFWithRndOffset(const Vector2D &pos) const {
    // Get base SDF value
    auto [sdfValue, closestPoint] = getSDF(pos);

    // Apply stepping effect only inside the polygon
    if (sdfValue > 0.0 && mMaxSDFVal > 0.0) {
      // Generate random offset based on maximum SDF value
      const double rndOffset = Random::get(0.1, 0.5) * mMaxSDFVal;

      // Calculate stepped SDF values
      const double stepNum = sdfValue / rndOffset;
      const double ceilSdf = std::ceil(stepNum) * rndOffset - sdfValue;
      const double floorSdf = sdfValue - std::floor(stepNum) * rndOffset;

      // Take minimum to create contour effect
      sdfValue = std::min({sdfValue, ceilSdf, floorSdf});
    }

    return std::make_tuple(sdfValue, closestPoint);
  }

  /**
   * @brief Generate grid points that lie inside the polygon
   * @return Vector of 2D points inside the polygon
   */
  std::vector<Vector2D> placeGridPoints() const {
    std::vector<Vector2D> interiorPoints;
    interiorPoints.reserve(mWidthCellNum * mHeightCellNum /
                           4); // Rough estimate

    for (int i = 0; i < mWidthCellNum; ++i) {
      for (int j = 0; j < mHeightCellNum; ++j) {
        const Vector2D pos = gridToWorld(i, j);
        const auto [sdfValue, _] = getSDF(pos);

        // Add point if inside polygon
        if (sdfValue > 0.0) {
          interiorPoints.emplace_back(pos);
        }
      }
    }

    interiorPoints.shrink_to_fit();
    return interiorPoints;
  }

  // Getters for read-only access
  double getCellSize() const { return mCellSize; }
  double getMaxSDFValue() const { return mMaxSDFVal; }
  int getWidthCellNum() const { return mWidthCellNum; }
  int getHeightCellNum() const { return mHeightCellNum; }
  const BoundingBox2D &getBoundingBox() const { return mBbox; }

private:
  /**
   * @brief Convert grid indices to world coordinates (cell center)
   */
  Vector2D gridToWorld(int i, int j) const {
    return mBbox.LowestPoint +
           Vector2D((i + 0.5) * mCellSize, (j + 0.5) * mCellSize);
  }

  /**
   * @brief Convert world coordinates to grid space
   */
  Vector2D worldToGrid(const Vector2D &pos) const {
    return (pos - mBbox.LowestPoint) / mCellSize;
  }

  /**
   * @brief Check if grid indices are valid
   */
  bool isValidGridIndex(int i, int j) const {
    return i >= 0 && i < mWidthCellNum && j >= 0 && j < mHeightCellNum;
  }

  /**
   * @brief Compute gradient at a grid point using finite differences
   * @param i Grid index in x direction
   * @param j Grid index in y direction
   * @return Gradient vector at grid point (i, j)
   */
  Vector2D computeGridGradient(int i, int j) const {
    double gradX = 0.0;
    double gradY = 0.0;

    // Compute x-component of gradient
    if (i == 0) {
      // Forward difference at left boundary
      gradX = (mSDFData(i + 1, j) - mSDFData(i, j)) / mCellSize;
    } else if (i == mWidthCellNum - 1) {
      // Backward difference at right boundary
      gradX = (mSDFData(i, j) - mSDFData(i - 1, j)) / mCellSize;
    } else {
      // Central difference in interior
      gradX = (mSDFData(i + 1, j) - mSDFData(i - 1, j)) / (2.0 * mCellSize);
    }

    // Compute y-component of gradient
    if (j == 0) {
      // Forward difference at bottom boundary
      gradY = (mSDFData(i, j + 1) - mSDFData(i, j)) / mCellSize;
    } else if (j == mHeightCellNum - 1) {
      // Backward difference at top boundary
      gradY = (mSDFData(i, j) - mSDFData(i, j - 1)) / mCellSize;
    } else {
      // Central difference in interior
      gradY = (mSDFData(i, j + 1) - mSDFData(i, j - 1)) / (2.0 * mCellSize);
    }

    return Vector2D(gradX, gradY);
  }

  /**
   * @brief Compute barycentric coordinates for interpolation
   * @param x Continuous grid coordinate
   * @param[out] i Integer grid index
   * @param[out] f Fractional part for interpolation [0, 1]
   * @param iLow Minimum allowed index
   * @param iHigh Maximum allowed index + 1
   */
  inline void getBarycentric(double x, int &i, double &f, int iLow,
                             int iHigh) const {
    const double floorX = std::floor(x);

    // Clamp integer index to valid range
    i = std::clamp(static_cast<int>(floorX), iLow, iHigh - 2);

    // Calculate fractional part with boundary handling
    if (x < iLow) {
      f = 0.0;
    } else if (x >= iHigh - 1) {
      f = 1.0;
    } else {
      f = x - floorX;
    }
  }

  /**
   * @brief Linear interpolation between two values
   */
  template <typename S, typename T>
  inline S lerp(const S &value0, const S &value1, T f) const {
    return (T(1.0) - f) * value0 + f * value1;
  }

  /**
   * @brief Bilinear interpolation on a 2x2 grid
   */
  template <typename S, typename T>
  inline S bilerp(const S &v00, const S &v10, const S &v01, const S &v11, T fx,
                  T fy) const {
    return lerp(lerp(v00, v10, fx), lerp(v01, v11, fx), fy);
  }

private:
  // Grid dimensions
  int mWidthCellNum;
  int mHeightCellNum;

  // Grid cell size in world units
  double mCellSize;

  // Maximum SDF value in the field
  double mMaxSDFVal;

  // Bounding box of the polygon
  BoundingBox2D mBbox;

  // 2D array storing SDF values at grid points
  Array2D mSDFData;

  // 2D array storing closest boundary points for each grid point
  Array2Vec2D mSDFClosestPoint;

  // Shared pointer to the polygon geometry
  Voronoi::VoronoiPolygon2Ptr mPolygon;
};

// Type alias for shared pointer
typedef std::shared_ptr<PolygonSDF2D> PolygonSDF2DPtr;

} // namespace HDV::SDF

#endif // _HDV_SDF_POLYGON2D_H_