/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-23 13:08:40
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-23 14:13:00
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\shape.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _SHAPE_H_
#define _SHAPE_H_

#pragma once

#include <kiri2d/physics/rigidbody/rigidbody.h>

namespace KIRI2D::PHY::RIGIDBODY
{

  enum ShapeType
  {
    CIRCLE,
    POLYGON
  };

  template <class RealType>
  class RigidBody;

  template <class RealType>
  class Shape
  {
  public:
    explicit Shape() {}
    virtual ~Shape()
    { // KIRI_LOG_DEBUG("~Shape");
    }

    virtual void ComputeMass(RealType density) = 0;
    virtual void SetOrientation(RealType ori) = 0;
    virtual const ShapeType GetType() = 0;

    void SetRigidBody(const std::shared_ptr<RigidBody<RealType>> &body)
    {
      mBody = body;
    }

    static const int SHAPE_NUM = 2;

  protected:
    std::weak_ptr<RigidBody<RealType>> mBody;
  };

  template <class RealType>
  class Circle : public Shape<RealType>
  {
  public:
    explicit Circle(RealType radius) : mRadius(radius) {}

    virtual void ComputeMass(RealType density) override
    {
      auto mass = KIRI_PI<RealType>() * mRadius * mRadius * density;
      mBody.lock()->SetMass(mass);
      mBody.lock()->SetInteria(mass * mRadius * mRadius);
    }

    virtual void SetOrientation(RealType ori) override
    {
    }

    virtual const ShapeType GetType() override { return CIRCLE; }
    const RealType GetRadius() const { return mRadius; }

  private:
    RealType mRadius;
  };

  template <class RealType>
  class Polygon : public Shape<RealType>
  {
  public:
    explicit Polygon() : mVerticesNum(0) {}

    virtual void ComputeMass(RealType density) override
    {
      if (mVerticesNum < 3)
        return;

      auto area = static_cast<RealType>(0.0);
      auto centroid = VectorX<2, RealType>(static_cast<RealType>(0.0));
      auto interia = static_cast<RealType>(0.0);
      const auto inv3 = static_cast<RealType>(1.0 / 3.0);
      for (auto i = 0; i < mVerticesNum; i++)
      {
        auto p1 = mVertices[i];
        auto p2 = mVertices[(i + 1) % mVerticesNum];
        auto d = p1.cross(p2);
        auto current_area = d * static_cast<RealType>(0.5);
        area += current_area;
        centroid += current_area * inv3 * (p1 + p2);
        interia += (static_cast<RealType>(0.25) * inv3 * d) * (p1.lengthSquared() + p2.lengthSquared() + p1.dot(p2));
      }

      centroid *= static_cast<RealType>(1.0) / area;

      for (auto i = 0; i < mVerticesNum; i++)
        mVertices[i] -= centroid;

      mBody.lock()->SetMass(area * density);
      mBody.lock()->SetInteria(interia * density);
    }

    void set(const std::vector<VectorX<2, RealType>> &data)
    {
      mVertices = data;
      mVerticesNum = data.size();
      mNormals.resize(mVerticesNum);

      for (auto i = 0; i < mVerticesNum; i++)
      {
        auto p1 = mVertices[i];
        auto p2 = mVertices[(i + 1) % mVerticesNum];
        auto face = p2 - p1;
        mNormals[i] = VectorX<2, RealType>(face.y, -face.x);
        if (mNormals[i].length() > MEpsilon<RealType>())
          mNormals[i].normalize();
      }
    }

    void SetAsBox(RealType width, RealType height)
    {
      mVerticesNum = 4;
      mVertices.resize(mVerticesNum);
      mNormals.resize(mVerticesNum);
      mVertices[0].set(-width, -height);
      mVertices[1].set(width, -height);
      mVertices[2].set(width, height);
      mVertices[3].set(-width, height);
      mNormals[0].set(static_cast<RealType>(0.0), -static_cast<RealType>(1.0));
      mNormals[1].set(static_cast<RealType>(1.0), static_cast<RealType>(0.0));
      mNormals[2].set(static_cast<RealType>(0.0), static_cast<RealType>(1.0));
      mNormals[3].set(-static_cast<RealType>(1.0), static_cast<RealType>(0.0));
    }

    virtual void SetOrientation(RealType ori) override
    {
      auto c = std::cos(ori);
      auto s = std::sin(ori);
      mMat.set(c, -s, s, c);
    }

    virtual const ShapeType GetType() override { return CIRCLE; }
    const Matrix2x2<RealType> &GetRotateMatrix() const { return mMat; }
    const int GetVerticesNum() const { return mVerticesNum; }
    const std::vector<VectorX<2, RealType>> &GetVertices() const { return mVertices; }
    const std::vector<VectorX<2, RealType>> &GetNormals() const { return mNormals; }

  private:
    int mVerticesNum;
    Matrix2x2<RealType> mMat;
    std::vector<VectorX<2, RealType>> mVertices, mNormals;
  };

  template <class RealType>
  using ShapePtr = std::shared_ptr<Shape<RealType>>;

  template <class RealType>
  using CirclePtr = std::shared_ptr<Circle<RealType>>;

  template <class RealType>
  using PolygonPtr = std::shared_ptr<Polygon<RealType>>;

} // namespace PHY::RIGIDBODY

#endif /* _SHAPE_H_ */