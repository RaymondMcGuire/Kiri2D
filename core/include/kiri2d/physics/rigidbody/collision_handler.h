/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-24 18:26:53
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-24 22:07:18
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\collision_handler.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _COLLISION_HANDLER_H_
#define _COLLISION_HANDLER_H_

#pragma once
#include <eventpp/eventdispatcher.h>
#include <kiri2d/physics/rigidbody/rigidbody.h>
#include <memory>

namespace KIRI2D::PHY::RIGIDBODY
{

  template <class RealType>
  class CollisionHandler;

  template <class RealType>
  static void
  Circle2Circle(const std::shared_ptr<CollisionHandler<RealType>> &handler,
                std::shared_ptr<RigidBody<RealType>> bodyA,
                std::shared_ptr<RigidBody<RealType>> bodyB)
  {

    // KIRI_LOG_DEBUG("Dispatch Circle2Circle!!");

    auto shape_a = std::dynamic_pointer_cast<Circle<RealType>>(bodyA->GetShape());
    auto shape_b = std::dynamic_pointer_cast<Circle<RealType>>(bodyB->GetShape());
    auto pos_a = bodyA->GetPosition();
    auto pos_b = bodyB->GetPosition();
    auto rad_a = shape_a->GetRadius();
    auto rad_b = shape_b->GetRadius();

    auto dir = pos_b - pos_a;
    auto dist = dir.length();
    auto radius = rad_a + rad_b;

    if (dist >= radius)
    {
      handler->SetContactNum(0);
      return;
    }

    handler->SetContactNum(1);

    if (dist == static_cast<RealType>(0.0))
    {
      handler->SetPenetration(rad_a);
      handler->SetContactDir(VectorX<2, RealType>(1.0, 0.0));
      handler->SetContactPoint0(pos_a);
    }
    else
    {
      // KIRI_LOG_DEBUG("contact!!");
      handler->SetPenetration(radius - dist);
      auto unit_dir = dir / dist;
      handler->SetContactDir(unit_dir);
      handler->SetContactPoint0(unit_dir * rad_a + pos_a);
    }
  }

  template <class RealType>
  class CollisionDispatchInternal
  {
  public:
    CollisionDispatchInternal()
    {
      mDispatcher.appendListener(
          std::make_pair(ShapeType::CIRCLE, ShapeType::CIRCLE),
          &Circle2Circle<RealType>);
    }

    void Dispatch(const std::shared_ptr<CollisionHandler<RealType>> &handler,
                  std::shared_ptr<RigidBody<RealType>> bodyA,
                  std::shared_ptr<RigidBody<RealType>> bodyB)
    {
      mDispatcher.dispatch(std::make_pair(bodyA->GetShape()->GetType(),
                                          bodyB->GetShape()->GetType()),
                           handler, bodyA, bodyB);
    }

  private:
    eventpp::EventDispatcher<
        std::pair<int, int>,
        void(const std::shared_ptr<CollisionHandler<RealType>> &,
             std::shared_ptr<RigidBody<RealType>>,
             std::shared_ptr<RigidBody<RealType>>)>
        mDispatcher;
  };

  template <class RealType>
  class CollisionDispatcher
  {
  public:
    CollisionDispatcher(CollisionDispatcher &other) = delete;

    void operator=(const CollisionDispatcher &) = delete;

    static std::shared_ptr<CollisionDispatchInternal<RealType>> GetInstance()
    {
      if (mInstance == nullptr)
      {
        mInstance = std::make_shared<CollisionDispatchInternal<RealType>>();
      }
      return mInstance;
    }

  private:
    inline static std::shared_ptr<CollisionDispatchInternal<RealType>> mInstance =
        nullptr;
  };

  template <class RealType>
  class CollisionHandler
      : public std::enable_shared_from_this<CollisionHandler<RealType>>
  {
  public:
    explicit CollisionHandler(const std::shared_ptr<RigidBody<RealType>> &bodyA,
                              const std::shared_ptr<RigidBody<RealType>> &bodyB)
        : mBodyA(bodyA), mBodyB(bodyB)
    {
      // this->CheckContactInfo();
    }

    virtual ~CollisionHandler() {}

    const int GetContactNum() const { return mContactNum; }
    void SetContactNum(int num) { mContactNum = num; }

    void SetContactDir(VectorX<2, RealType> dir) { mContactDir = dir; }
    void SetPenetration(RealType val) { mPenetration = val; }
    void SetContactPoint0(VectorX<2, RealType> p) { mContactPoints[0] = p; }
    void SetContactPoint1(VectorX<2, RealType> p) { mContactPoints[1] = p; }

    const RealType GetMixRestitution() const { return mMixRestitution; }
    const RealType GetMixStaticFriction() const { return mMixStaticFriction; }
    const RealType GetMixDynamicFriction() const { return mMixDynamicFriction; }

    void ComputeImpluse()
    {
      auto a_im = mBodyA->GetInvMass();
      auto b_im = mBodyB->GetInvMass();
      if (std::abs(a_im + b_im) < MEpsilon<RealType>())
      {
        InfiniteMassCorrection();
        return;
      }

      for (auto i = 0; i < mContactNum; i++)
      {

        auto ra = mContactPoints[i] - mBodyA->GetPosition();
        auto rb = mContactPoints[i] - mBodyB->GetPosition();

        auto rv = mBodyB->GetVelocity() + VectorX<2, RealType>(-mBodyB->GetAngularVelocity() * rb.y, mBodyB->GetAngularVelocity() * rb.x) - mBodyA->GetVelocity() - VectorX<2, RealType>(-mBodyA->GetAngularVelocity() * ra.y, mBodyA->GetAngularVelocity() * ra.x);

        auto contact_vel = rv.dot(mContactDir);
        if (contact_vel > 0)
          return;

        auto ra_cross_n = ra.cross(mContactDir);
        auto rb_cross_n = rb.cross(mContactDir);
        auto inverse_mass_sum = a_im + b_im + ra_cross_n * ra_cross_n * mBodyA->GetInvInertia() + rb_cross_n * rb_cross_n * mBodyB->GetInvInertia();

        auto j = -(static_cast<RealType>(1.0) + mMixRestitution) * contact_vel;
        j /= inverse_mass_sum;
        j /= static_cast<RealType>(mContactNum);

        auto impulse = mContactDir * j;
        // KIRI_LOG_DEBUG("Compute first impulse; impulse={0},{1}; j={2};inverse_mass_sum={3};contact_vel={4}",
        //                impulse.x, impulse.y, j, inverse_mass_sum, contact_vel);

        mBodyA->ApplyImpulse(-impulse, ra);
        mBodyB->ApplyImpulse(impulse, rb);

        // friction
        rv = mBodyB->GetVelocity() + VectorX<2, RealType>(-mBodyB->GetAngularVelocity() * rb.y, mBodyB->GetAngularVelocity() * rb.x) - mBodyA->GetVelocity() - VectorX<2, RealType>(-mBodyA->GetAngularVelocity() * ra.y, mBodyA->GetAngularVelocity() * ra.x);

        auto t = rv - (mContactDir * rv.dot(mContactDir));
        if (t.length() > MEpsilon<RealType>())
          t.normalize();
        // KIRI_LOG_DEBUG("Computejt pre; rv={0},{1}; t={2},{3}", rv.x, rv.y, t.x, t.y);
        // KIRI_LOG_DEBUG("mContactDir={0},{1}; rv.dot(mContactDir)={2}", mContactDir.x, mContactDir.y, rv.dot(mContactDir));

        auto jt = -rv.dot(t);
        jt /= inverse_mass_sum;
        jt /= static_cast<RealType>(mContactNum);
        // KIRI_LOG_DEBUG("Computejt; jt={0}; inverse_mass_sum={1}", jt, inverse_mass_sum);

        if (std::abs(jt) < MEpsilon<RealType>())
          return;

        auto tangent_impulse = t * -j * mMixDynamicFriction;
        if (std::abs(jt) < j * mMixStaticFriction)
          tangent_impulse = t * jt;

        // KIRI_LOG_DEBUG("ComputeImpluse; tangent_impulse={0},{1}, ra={2},{3}, rb={4},{5} ", tangent_impulse.x, tangent_impulse.y, ra.x, ra.y, rb.x, rb.y);

        mBodyA->ApplyImpulse(-tangent_impulse, ra);
        mBodyB->ApplyImpulse(tangent_impulse, rb);
      }
    }

    void PositionalCorrection()
    {
      auto k_slop = static_cast<RealType>(0.05);
      auto percent = static_cast<RealType>(0.4);
      auto correction = (std::max(mPenetration - k_slop, static_cast<RealType>(0.0)) / (mBodyA->GetInvMass() + mBodyB->GetInvMass())) * mContactDir * percent;
      mBodyA->AddPosition(-correction * mBodyA->GetInvMass());
      mBodyB->AddPosition(correction * mBodyB->GetInvMass());
    }

    void ComputeContactSurfaceInfo(RealType dt, const VectorX<2, RealType> &gravity)
    {
      if (!mContactNum)
        return;

      mMixRestitution = std::min(mBodyA->GetRestitution(), mBodyB->GetRestitution());
      mMixStaticFriction = std::sqrt(mBodyA->GetStaticFriction() * mBodyA->GetStaticFriction() + mBodyB->GetStaticFriction() * mBodyB->GetStaticFriction());
      mMixDynamicFriction = std::sqrt(mBodyA->GetDynamicFriction() * mBodyA->GetDynamicFriction() + mBodyB->GetDynamicFriction() * mBodyB->GetDynamicFriction());

      for (auto i = 0; i < mContactNum; i++)
      {

        auto ra = mContactPoints[i] - mBodyA->GetPosition();
        auto rb = mContactPoints[i] - mBodyB->GetPosition();

        auto rv = mBodyB->GetVelocity() + VectorX<2, RealType>(-mBodyB->GetAngularVelocity() * rb.y, mBodyB->GetAngularVelocity() * rb.x) - mBodyA->GetVelocity() - VectorX<2, RealType>(-mBodyA->GetAngularVelocity() * ra.y, mBodyA->GetAngularVelocity() * ra.x);

        if (rv.lengthSquared() < (dt * gravity).lengthSquared() + MEpsilon<RealType>())
          mMixRestitution = static_cast<RealType>(0.0);
      }
    }

    void CheckContactInfo()
    {
      CollisionDispatcher<RealType>::GetInstance()->Dispatch(shared_from_this(),
                                                             mBodyA, mBodyB);
    }

  private:
    int mContactNum = 0;
    RealType mMixRestitution, mMixStaticFriction, mMixDynamicFriction;

    RealType mPenetration;
    VectorX<2, RealType> mContactDir;
    VectorX<2, RealType> mContactPoints[2];

    std::shared_ptr<RigidBody<RealType>> mBodyA, mBodyB;

    void InfiniteMassCorrection()
    {
      mBodyA->SetVelocity(VectorX<2, RealType>(static_cast<RealType>(0.0)));
      mBodyB->SetVelocity(VectorX<2, RealType>(static_cast<RealType>(0.0)));
    }
  };

} // namespace PHY::RIGIDBODY

#endif /* _COLLISION_HANDLER_H_ */