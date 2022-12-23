/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-19 16:10:09
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-23 15:07:16
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\rigidbody.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _RIGIDBODY_H_
#define _RIGIDBODY_H_

#pragma once

#include <kiri2d/physics/rigidbody/shape.h>

namespace PHY::RIGIDBODY {
template <class RealType> class RigidBody {
public:
  explicit RigidBody(VectorX<2, RealType> pos)
      : mPosition(pos), mDensity(1.0), mMass(0.0), mInverseMass(0.0) {
    mVelocity = VectorX<2, RealType>(static_cast<RealType>(0.0));
  }

  virtual ~RigidBody() {
    // KIRI_LOG_DEBUG("~RigidBody");
  }

  void SetShape(const std::shared_ptr<Shape<RealType>> &shape) {
    mShape = shape;
  }

  const std::shared_ptr<Shape<RealType>> &GetShape() const { return mShape; }

  void SetMass(RealType mass) {
    mMass = mass;
    mInverseMass =
        mMass ? static_cast<RealType>(1.0) / mMass : static_cast<RealType>(0.0);
  }

  void SetPosition(VectorX<2, RealType> pos) { mPosition = pos; }
  void SetVelocity(VectorX<2, RealType> vel) { mVelocity = vel; }

  const RealType GetMass() const { return mMass; }
  const RealType GetInvMass() const { return mInverseMass; }
  const RealType GetDensity() const { return mDensity; }
  const RealType GetRestitution() const { return mRestitution; }

  const VectorX<2, RealType> GetPosition() const { return mPosition; }
  const VectorX<2, RealType> GetVelocity() const { return mVelocity; }

  friend void CompositeShapeRigidBody(
      const std::shared_ptr<Shape<RealType>> &shape,
      const std::shared_ptr<RigidBody<RealType>> &rigidbody) {
    shape->SetRigidBody(rigidbody);
    rigidbody->SetShape(shape);
    shape->ComputeMass(rigidbody->GetDensity());
  }

  void Print() const {
    KIRI_LOG_INFO("RigidBody mass={0}; inv mass={1}; density={2}", mMass,
                  mInverseMass, mDensity);
  }

private:
  RealType mMass, mInverseMass, mDensity, mRestitution;
  VectorX<2, RealType> mPosition, mVelocity;

  std::shared_ptr<Shape<RealType>> mShape;
};

template <class RealType>
using RigdiBodyPtr = std::shared_ptr<RigidBody<RealType>>;
} // namespace PHY::RIGIDBODY

#endif /* _RIGIDBODY_H_ */