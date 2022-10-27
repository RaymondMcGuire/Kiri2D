/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-10-27 12:06:56
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-27 13:22:02
 * @FilePath: \Kiri2D\core\include\kiri2d\hdv_toolkit\voronoi\voronoi_polygon.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _HDV_VORONOI_POLYGON_H_
#define _HDV_VORONOI_POLYGON_H_

#pragma once

#include <Mathematics/DistPointTriangle.h>
#include <kiri2d/hdv_toolkit/delaunay/delaunay_cell.h>
#include <kiri2d/straight_skeleton/sskel_slav.h>

namespace HDV::Voronoi {
class VoronoiPolygon2 {
public:
  explicit VoronoiPolygon2() {}

  virtual ~VoronoiPolygon2() {}

  void add(Vector2D vertices) {
    mPositions.emplace_back(vertices);
    mBBox.merge(vertices);
  }

  void updateBBox() {
    mBBox.reset();
    for (auto i = 0; i < mPositions.size(); i++)
      mBBox.merge(mPositions[i]);
  }

  bool checkBBox() {
    if (mBBox.isEmpty()) {
      if (!mPositions.empty()) {
        updateBBox();
        return true;
      } else {
        KIRI_LOG_ERROR("contains:: No polygon data!!");
        return false;
      }
    }
    return true;
  }

  Vector2D rndInnerPoint() {
    if (!checkBBox()) {
      KIRI_LOG_ERROR("rndInnerPoint:: Get inner point failed!!");
      return Vector2D(0.0);
    }

    Vector2D inner;
    do {
      inner =
          mBBox.LowestPoint + Vector2D(Random::get(0.0, 1.0) * mBBox.width(),
                                       Random::get(0.0, 1.0) * mBBox.height());
    } while (!contains(inner));

    return inner;
  }

  Vector2D rndInnerPointWithDistConstrain(double minDist) {
    if (!checkBBox()) {
      KIRI_LOG_ERROR("rndInnerPoint:: Get inner point failed!!");
      return Vector2D(0.0);
    }

    Vector2D inner;
    do {
      inner =
          mBBox.LowestPoint + Vector2D(Random::get(0.0, 1.0) * mBBox.width(),
                                       Random::get(0.0, 1.0) * mBBox.height());
    } while (!contains(inner) || computeMinDisInPoly(inner) < minDist);

    return inner;
  }

  bool contains(const Vector2D &v) {
    if (!checkBBox())
      return false;

    if (!mBBox.contains(v))
      return false;

    bool contains = false;
    for (size_t i = 0, j = mPositions.size() - 1; i < mPositions.size();
         j = i++) {
      auto vert_i = mPositions[i];
      auto vert_j = mPositions[j];
      if ((((vert_i.y <= v.y) && (v.y < vert_j.y) ||
            ((vert_j.y <= v.y) && (v.y < vert_i.y))) &&
           (v.x <
            (vert_j.x - vert_i.x) * (v.y - vert_i.y) / (vert_j.y - vert_i.y) +
                vert_i.x)))
        contains = !contains;
    }
    return contains;
  }

  double area() {
    auto area = 0.0;
    if (!mPositions.empty())
      for (size_t i = 0; i < mPositions.size(); i++)
        area += mPositions[i].cross(mPositions[(i + 1) % mPositions.size()]);

    return std::abs(area * 0.5);
  }

  Vector2D centroid() {
    std::vector<Vector2D> tmp_poly(mPositions);

    auto first = tmp_poly[0], last = tmp_poly[tmp_poly.size() - 1];
    if (first.x != last.x || first.y != last.y)
      tmp_poly.emplace_back(first);

    auto twice_area = 0.0, x = 0.0, y = 0.0, f = 0.0;
    auto num = tmp_poly.size();
    Vector2D p1, p2;

    for (size_t i = 0, j = num - 1; i < num; j = i++) {
      p1 = tmp_poly[i];
      p2 = tmp_poly[j];
      f = p1.x * p2.y - p2.x * p1.y;
      twice_area += f;
      x += (p1.x + p2.x) * f;
      y += (p1.y + p2.y) * f;
    }
    f = twice_area * 3.0;

    return Vector2D(x / f, y / f);
  }

  bool isClockwise(const std::vector<Vector4F> &poly) {
    auto a = 0.f;
    auto polySize = poly.size();
    for (size_t i = 0; i < polySize; i++) {
      auto s = Vector2F(poly[i].x, poly[i].y);
      auto e = Vector2F(poly[i].z, poly[i].w);
      a += (e.x - s.x) * (e.y + s.y);
    }

    return a < 0.f;
  }

  void computeSSkel1998Convex() {
    mSkeletons.clear();

    std::vector<Vector4F> poly;
    std::vector<Vector2F> reverse_poly;

    for (size_t i = 0; i < mPositions.size(); i++) {
      auto v1 = mPositions[i];
      auto v2 = mPositions[(i + 1) % mPositions.size()];
      poly.emplace_back(Vector4F(v1.x, v1.y, v2.x, v2.y));
      reverse_poly.emplace_back(Vector2F(v1.x, v1.y));
    }

    if (isClockwise(poly))
      std::reverse(reverse_poly.begin(), reverse_poly.end());

    auto sskel_convex =
        std::make_shared<KIRI2D::SSKEL::SSkelSLAV>(reverse_poly);

    auto skeletons = sskel_convex->GetSkeletons();

    for (size_t i = 0; i < skeletons.size(); i++) {
      auto [intersect, sinks] = skeletons[i];
      for (size_t j = 0; j < sinks.size(); j++)
        mSkeletons.emplace_back(
            Vector4F(intersect.x, intersect.y, sinks[j].x, sinks[j].y));
    }
  }

  double minDis2LineSegment2(Vector2D v, Vector2D w, Vector2D p) {
    auto l2 = v.distanceSquaredTo(w);
    if (l2 == 0.f)
      return p.distanceTo(v);

    auto t = std::clamp(((p - v).dot(w - v)) / l2, 0.0, 1.0);
    auto projection = v + t * (w - v);
    return p.distanceTo(projection);
  }

  double computeMinDisInPoly(const Vector2D &p) {
    auto min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < mPositions.size(); i++)
      min_dist = std::min(
          min_dist,
          minDis2LineSegment2(mPositions[i],
                              mPositions[(i + 1) % mPositions.size()], p));
    return min_dist;
  }

  Vector2D minDisPoint2LineSegment2(Vector2D v, Vector2D w, Vector2D p) {
    auto l2 = v.distanceSquaredTo(w);
    if (l2 == 0.f)
      return v;

    auto t = std::clamp(((p - v).dot(w - v)) / l2, 0.0, 1.0);
    auto projection = v + t * (w - v);
    return projection;
  }

  Vector2D computeMinDistPointInPoly(const Vector2D &p) {
    auto min_dist_point = Vector2D(0.0);
    auto min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < mPositions.size(); i++) {
      auto dist_point = minDisPoint2LineSegment2(
          mPositions[i], mPositions[(i + 1) % mPositions.size()], p);
      auto dist = p.distanceTo(dist_point);
      if (dist < min_dist) {
        min_dist = dist;
        min_dist_point = dist_point;
      }
    }

    return min_dist_point;
  }

  Vector3D computeMICByStraightSkeleton() {
    auto max_circle_pos = Vector2D(0.0);
    auto max_circle_radius = std::numeric_limits<double>::min();
    if (!mSkeletons.empty()) {
      for (size_t i = 0; i < mSkeletons.size(); i++) {
        auto v1 = Vector2D(mSkeletons[i].x, mSkeletons[i].y);
        auto v2 = Vector2D(mSkeletons[i].z, mSkeletons[i].w);

        if (contains(v1)) {
          auto min_dist = computeMinDisInPoly(v1);
          if (min_dist > max_circle_radius) {
            max_circle_radius = min_dist;
            max_circle_pos = v1;
          }
        }

        if (contains(v2)) {
          auto min_dist = computeMinDisInPoly(v2);
          if (min_dist > max_circle_radius) {
            max_circle_radius = min_dist;
            max_circle_pos = v2;
          }
        }
      }
    }
    return Vector3D(max_circle_pos.x, max_circle_pos.y, max_circle_radius);
  }

  const BoundingBox2D bbox() const { return mBBox; }

  std::vector<Vector2D> &positions() { return mPositions; }

  std::vector<Vector4F> &skeletons() { return mSkeletons; }

private:
  BoundingBox2D mBBox;
  std::vector<Vector2D> mPositions;
  std::vector<Vector4F> mSkeletons;
};

class VoronoiPolygon3 {
public:
  explicit VoronoiPolygon3() {}

  virtual ~VoronoiPolygon3() {}

  void add(Vector3D vertices) {
    mPositions.emplace_back(vertices);
    mBBox.merge(vertices);
  }

  void updateBBox() {
    mBBox.reset();
    for (auto i = 0; i < mPositions.size(); i++)
      mBBox.merge(mPositions[i]);
  }

  double computeVolume(Vector3D a, Vector3D b, Vector3D c) {
    return b.cross(c).dot(a) / 6.0;
  }

  double computeMinDisInPoly(Vector3D p) {

    auto min_dist = std::numeric_limits<double>::max();

    for (size_t j = 0; j < mIndices.size() / 3; j++) {
      auto a = mPositions[mIndices[j * 3] - mIndexOffset];
      auto b = mPositions[mIndices[j * 3 + 1] - mIndexOffset];
      auto c = mPositions[mIndices[j * 3 + 2] - mIndexOffset];

      gte::Vector<3, double> vec3 = {{p.x, p.y, p.z}};
      gte::Vector<3, double> a3 = {{a.x, a.y, a.z}},
                             b3 = gte::Vector<3, double>{{b.x, b.y, b.z}},
                             c3 = gte::Vector<3, double>{{c.x, c.y, c.z}};
      gte::Triangle<3, double> tri3(a3, b3, c3);

      gte::DCPPoint3Triangle3<double> dpt3;
      auto res = dpt3(vec3, tri3);
      auto dis = res.distance * 2.0;
      min_dist = std::min(min_dist, dis);
    }

    KIRI_LOG_DEBUG("min_dist={0}", min_dist);
    return min_dist;
  }

  double volume() {
    mIndexOffset = mIsLoadedFromObj ? 1 : 0;

    auto V = 0.0;
    for (size_t j = 0; j < mIndices.size() / 3; j++) {
      // if indices loaded from a .obj file, index need -1
      auto a = mPositions[mIndices[j * 3] - mIndexOffset];
      auto b = mPositions[mIndices[j * 3 + 1] - mIndexOffset];
      auto c = mPositions[mIndices[j * 3 + 2] - mIndexOffset];

      V += computeVolume(a, b, c);
    }

    if (V < 0.0)
      KIRI_LOG_ERROR("Volume is negnative!!!");

    return V;
  }

  Vector3D centroid() {
    mIndexOffset = mIsLoadedFromObj ? 1 : 0;

    auto volume = 0.0;
    Vector3D centroid;
    for (size_t j = 0; j < mIndices.size() / 3; j++) {

      // if indices loaded from a .obj file, index need -1
      auto a = mPositions[mIndices[j * 3] - mIndexOffset];
      auto b = mPositions[mIndices[j * 3 + 1] - mIndexOffset];
      auto c = mPositions[mIndices[j * 3 + 2] - mIndexOffset];

      auto v = computeVolume(a, b, c);
      volume += v;
      centroid += v * (a + b + c) / 4.0;
    }

    if (centroid.x != centroid.x || centroid.y != centroid.y ||
        centroid.z != centroid.z)
      KIRI_LOG_ERROR("error centroid!!!!");

    return centroid / volume;
  }

  std::vector<Vector3D> &positions() { return mPositions; }

  std::vector<Vector3D> &normals() { return mNormals; }

  std::vector<int> &indices() { return mIndices; }

  const BoundingBox3D bbox() const { return mBBox; }

private:
  bool mIsLoadedFromObj = false;
  int mIndexOffset = 0;
  BoundingBox3D mBBox;
  std::vector<Vector3D> mPositions;
  std::vector<Vector3D> mNormals;
  std::vector<int> mIndices;
};

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_POLYGON_H_ */