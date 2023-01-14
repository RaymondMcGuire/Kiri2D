/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2023-01-11 14:46:17
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-14 15:37:08
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\rigidbody.h
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _RIGIDBODY_H_
#define _RIGIDBODY_H_

#pragma once

#include <kiri2d/physics/rigidbody/shape.h>

namespace KIRI2D::PHY::RIGIDBODY
{
  template <class RealType>
  class RigidBody
  {
  public:
    explicit RigidBody(VectorX<2, RealType> pos)
        : mPosition(pos), mDensity(1.0), mMass(0.0), mInverseMass(0.0), mTorque(0.0), mAngularVelocity(0.0), mIsStatic(false)
    {
      mVelocity = VectorX<2, RealType>(static_cast<RealType>(0.0));
      mRestitution = static_cast<RealType>(1.0);
      mStaticFriction = static_cast<RealType>(0.5);
      mDynamicFriction = static_cast<RealType>(0.3);
      mOrientation = Random::get<RealType>(-KIRI_PI<RealType>(), KIRI_PI<RealType>());
    }

    virtual ~RigidBody()
    {
      // KIRI_LOG_DEBUG("~RigidBody");
    }

    void SetShape(const std::shared_ptr<Shape<RealType>> &shape)
    {
      mShape = shape;
    }

    const std::shared_ptr<Shape<RealType>> &GetShape() const { return mShape; }

    void SetMass(RealType mass)
    {
      mMass = mass;
      mInverseMass =
          mMass ? static_cast<RealType>(1.0) / mMass : static_cast<RealType>(0.0);
    }

    void SetInteria(RealType interia)
    {
      mInertia = interia;
      mInverseInertia =
          interia ? static_cast<RealType>(1.0) / interia : static_cast<RealType>(0.0);
    }

    void SetPosition(VectorX<2, RealType> pos) { mPosition = pos; }
    void SetVelocity(VectorX<2, RealType> vel) { mVelocity = vel; }
    void SetAngularVelocity(RealType vel) { mAngularVelocity = vel; }
    void SetOrientation(RealType ori)
    {
      mOrientation = ori;
      mShape->SetOrientation(mOrientation);
    }
    void SetRestitution(const RealType val) { mRestitution = val; }
    void SetStaticFriction(const RealType val) { mStaticFriction = val; }
    void SetDynamicFriction(const RealType val) { mDynamicFriction = val; }

    void AddPosition(VectorX<2, RealType> pos) { mPosition += pos; }
    void AddVelocity(VectorX<2, RealType> vel) { mVelocity += vel; }
    void AddAngularVelocity(RealType vel) { mAngularVelocity += vel; }

    void ApplyImpulse(VectorX<2, RealType> impulse, VectorX<2, RealType> contactDir)
    {
      mVelocity += mInverseMass * impulse;
      mAngularVelocity += mInverseInertia * contactDir.cross(impulse);
    }

    void AddOrientation(RealType radian)
    {
      mOrientation += radian;
      mShape->SetOrientation(mOrientation);
    }

    const RealType GetMass() const { return mMass; }
    const RealType GetInvMass() const { return mInverseMass; }

    const RealType GetInertia() const { return mInertia; }
    const RealType GetInvInertia() const { return mInverseInertia; }

    const RealType GetDensity() const { return mDensity; }
    const RealType GetRestitution() const { return mRestitution; }
    const RealType GetStaticFriction() const { return mStaticFriction; }
    const RealType GetDynamicFriction() const { return mDynamicFriction; }

    const RealType GetOrientation() const { return mOrientation; }
    const RealType GetTorque() const { return mTorque; }
    const RealType GetAngularVelocity() const { return mAngularVelocity; }

    const VectorX<2, RealType> &GetPosition() const { return mPosition; }
    const VectorX<2, RealType> &GetVelocity() const { return mVelocity; }
    const VectorX<2, RealType> &GetForce() const { return mForce; }

    const bool IsStatic() const { return mIsStatic; }

    void SetAsStatic()
    {
      mIsStatic = true;
      mMass = static_cast<RealType>(0.0);
      mInverseMass = static_cast<RealType>(0.0);
      mInertia = static_cast<RealType>(0.0);
      mInverseInertia = static_cast<RealType>(0.0);
      SetOrientation(static_cast<RealType>(0.0));
    }

    void ClearForces()
    {
      mForce = VectorX<2, RealType>(static_cast<RealType>(0.0));
      mTorque = static_cast<RealType>(0.0);
    }

    friend void CompositeShapeRigidBody(
        const std::shared_ptr<Shape<RealType>> &shape,
        const std::shared_ptr<RigidBody<RealType>> &rigidbody)
    {
      shape->SetRigidBody(rigidbody);
      rigidbody->SetShape(shape);
      shape->ComputeMass(rigidbody->GetDensity());
    }

    void Print() const
    {
      KIRI_LOG_INFO("RigidBody mass={0}; inv mass={1}; density={2}", mMass,
                    mInverseMass, mDensity);
    }

  private:
    bool mIsStatic;
    RealType mMass, mInverseMass;
    RealType mInertia, mInverseInertia;
    RealType mDensity, mRestitution, mStaticFriction, mDynamicFriction;

    RealType mOrientation, mTorque, mAngularVelocity;
    VectorX<2, RealType> mPosition, mVelocity, mForce;

    std::shared_ptr<Shape<RealType>> mShape;
  };

  template <class RealType>
  using RigdiBodyPtr = std::shared_ptr<RigidBody<RealType>>;
} // namespace PHY::RIGIDBODY

#endif /* _RIGIDBODY_H_ */