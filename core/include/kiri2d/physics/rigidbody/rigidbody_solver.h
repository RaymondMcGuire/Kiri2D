/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-24 18:26:53
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-24 18:59:25
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\rigidbody_solver.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _RIGIDBODY_SOLVER_H_
#define _RIGIDBODY_SOLVER_H_

#pragma once

#include <kiri2d/physics/rigidbody/collision_handler.h>

namespace PHY::RIGIDBODY
{

  template <class RealType>
  class RigidBodySolver
  {
  public:
    explicit RigidBodySolver()
    {
    }

    virtual ~RigidBodySolver() {}

    void UpdateSolver(
        const std::vector<std::shared_ptr<RigidBody<RealType>>> &rigidbodies,
        RealType dt,
        RealType gravity)
    {
      GenerateCollider(rigidbodies);

      UpdateVelocity(dt, gravity);

      HandleCollision(dt, gravity);
    }

  private:
    std::vector<std::shared_ptr<CollisionHandler<RealType>>> mCollisions;

    void GenerateCollider(const std::vector<std::shared_ptr<RigidBody<RealType>>> &rigidbodies)
    {
      mCollisions.clear();
      for (auto i = 0; i < rigidbodies.size() - 1; i++)
      {
        auto bodyA = rigidbodies[i];
        for (auto j = i + 1; i < rigidbodies.size(); j++)
        {
          auto bodyB = rigidbodies[j];
          if (bodyA->GetInvMass() == 0 && bodyB->GetInvMass() == 0)
            continue;

          auto collision = std::make_shared<CollisionHandler<RealType>>(bodyA, bodyB);
          if (collision->GetContactNum)
            mCollisions.emplace_back(collision);
        }
      }
    }

    void UpdateVelocity(
        const std::vector<std::shared_ptr<RigidBody<RealType>>> &rigidbodies,
        RealType dt,
        RealType gravity)
    {
      for (auto i = 0; i < rigidbodies.size(); i++)
      {
        auto body = rigidbodies[i];
        if (body->GetInverseMass() == static_cast<RealType>(0.0))
          continue;
        body->AddVelocity((body->GetForce() * body->GetInverseMass() + gravity) * (dt / static_cast<RealType>(2.0)));
        body->AddAngularVelocity(body->GetTorque() * body->GetInverseInteria() * (dt / static_cast<RealType>(2.0)));
      }
    }

    void HandleCollision(RealType dt, RealType gravity)
    {
      for (auto i = 0; i < mCollisions.size(); i++)
        mCollisions[i]->ComputeContactSurfaceInfo(dt, gravity);
    }
  };

} // namespace PHY::RIGIDBODY

#endif /* _RIGIDBODY_SOLVER_H_ */