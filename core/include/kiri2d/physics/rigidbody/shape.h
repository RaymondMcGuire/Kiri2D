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

namespace PHY::RIGIDBODY {

enum ShapeType { CIRCLE, POLYGON };

template <class RealType> class RigidBody;

template <class RealType> class Shape {
public:
  explicit Shape() {}
  virtual ~Shape() { // KIRI_LOG_DEBUG("~Shape");
  }

  virtual void ComputeMass(RealType density) = 0;
  virtual const ShapeType GetType() = 0;

  void SetRigidBody(const std::shared_ptr<RigidBody<RealType>> &body) {
    mBody = body;
  }

  static const int SHAPE_NUM = 2;

protected:
  std::weak_ptr<RigidBody<RealType>> mBody;
};

template <class RealType> class Circle : public Shape<RealType> {
public:
  explicit Circle(RealType radius) : mRadius(radius) {}

  virtual void ComputeMass(RealType density) override {
    mBody.lock()->SetMass(KIRI_PI<RealType>() * mRadius * mRadius * density);
  }

  virtual const ShapeType GetType() override { return CIRCLE; }
  const RealType GetRadius() const { return mRadius; }

private:
  RealType mRadius;
};

template <class RealType> using ShapePtr = std::shared_ptr<Shape<RealType>>;

template <class RealType> using CirclePtr = std::shared_ptr<Circle<RealType>>;

} // namespace PHY::RIGIDBODY

#endif /* _SHAPE_H_ */