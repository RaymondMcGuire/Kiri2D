/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-19 16:10:09
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-23 14:03:50
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

namespace PHY::RIGIDBODY {

template <class RealType> class CollisionHandler;

template <class RealType>
static void
Circle2Circle(const std::shared_ptr<CollisionHandler<RealType>> &handler,
              std::shared_ptr<RigidBody<RealType>> bodyA,
              std::shared_ptr<RigidBody<RealType>> bodyB) {

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

  if (dist >= radius) {
    handler->SetContactNum(0);
    return;
  }

  handler->SetContactNum(1);

  if (dist == static_cast<RealType>(0.0)) {
    handler->SetPenetration(rad_a);
    handler->SetContactDir(VectorX<2, RealType>(1.0, 0.0));
    handler->SetContactPoint0(pos_a);
  } else {
    handler->SetPenetration(radius - dist);
    auto unit_dir = dir / dist;
    handler->SetContactDir(unit_dir);
    handler->SetContactPoint0(unit_dir * rad_a + pos_a);
  }
}

template <class RealType> class CollisionDispatchInternal {
public:
  CollisionDispatchInternal() {
    mDispatcher.appendListener(
        std::make_pair(ShapeType::CIRCLE, ShapeType::CIRCLE),
        &Circle2Circle<RealType>);
  }

  void Dispatch(const std::shared_ptr<CollisionHandler<RealType>> &handler,
                std::shared_ptr<RigidBody<RealType>> bodyA,
                std::shared_ptr<RigidBody<RealType>> bodyB) {
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

template <class RealType> class CollisionDispatcher {
public:
  CollisionDispatcher(CollisionDispatcher &other) = delete;

  void operator=(const CollisionDispatcher &) = delete;

  static std::shared_ptr<CollisionDispatchInternal<RealType>> GetInstance() {
    if (mInstance == nullptr) {
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
    : public std::enable_shared_from_this<CollisionHandler<RealType>> {
public:
  explicit CollisionHandler(const std::shared_ptr<RigidBody<RealType>> &bodyA,
                            const std::shared_ptr<RigidBody<RealType>> &bodyB)
      : mBodyA(bodyA), mBodyB(bodyB) {
    mMixRestitution =
        std::min(mBodyA->GetRestitution(), mBodyB->GetRestitution());
  }

  virtual ~CollisionHandler() {}

  const int GetContactNum() const { return mContactNum; }
  void SetContactNum(int num) { mContactNum = num; }

  void SetContactDir(VectorX<2, RealType> dir) { mContactDir = dir; }
  void SetPenetration(RealType val) { mPenetration = val; }
  void SetContactPoint0(VectorX<2, RealType> p) { mContactPoints[0] = p; }
  void SetContactPoint1(VectorX<2, RealType> p) { mContactPoints[1] = p; }

  void ComputeImpluse() {
    auto a_im = mBodyA->GetInvMass();
    auto b_im = mBodyB->GetInvMass();
    if (std::abs(a_im + b_im) < MEpsilon<RealType>()) {
      InfiniteMassCorrection();
      return;
    }
  }

  void Solve() {
    CollisionDispatcher<RealType>::GetInstance()->Dispatch(shared_from_this(),
                                                           mBodyA, mBodyB);
  }

private:
  int mContactNum = 0;
  RealType mMixRestitution;

  RealType mPenetration;
  VectorX<2, RealType> mContactDir;
  VectorX<2, RealType> mContactPoints[2];

  std::shared_ptr<RigidBody<RealType>> mBodyA, mBodyB;

  void InfiniteMassCorrection() {
    mBodyA->SetVelocity(VectorX<2, RealType>(static_cast<RealType>(0.0)));
    mBodyB->SetVelocity(VectorX<2, RealType>(static_cast<RealType>(0.0)));
  }
};

} // namespace PHY::RIGIDBODY

#endif /* _COLLISION_HANDLER_H_ */