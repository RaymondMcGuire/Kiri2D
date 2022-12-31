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

namespace KIRI2D::PHY::RIGIDBODY
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
        int iteration,
        RealType dt,
        const VectorX<2, RealType> &gravity)
    {
      GenerateCollider(rigidbodies);

      UpdateVelocity(rigidbodies, dt, gravity);

      HandleCollision(iteration, dt, gravity);

      Advect(rigidbodies, dt);

      UpdateVelocity(rigidbodies, dt, gravity);

      PositionalCorrection();

      ClearForces(rigidbodies);
    }

  private:
    std::vector<std::shared_ptr<CollisionHandler<RealType>>> mCollisions;

    void GenerateCollider(const std::vector<std::shared_ptr<RigidBody<RealType>>> &rigidbodies)
    {
      mCollisions.clear();
      for (auto i = 0; i < rigidbodies.size() - 1; i++)
      {
        auto bodyA = rigidbodies[i];
        for (auto j = i + 1; j < rigidbodies.size(); j++)
        {
          auto bodyB = rigidbodies[j];
          if (bodyA->GetInvMass() == 0 && bodyB->GetInvMass() == 0)
            continue;

          auto collision = std::make_shared<CollisionHandler<RealType>>(bodyA, bodyB);
          collision->CheckContactInfo();
          if (collision->GetContactNum())
            mCollisions.emplace_back(collision);
        }
      }
    }

    void UpdateVelocity(
        const std::vector<std::shared_ptr<RigidBody<RealType>>> &rigidbodies,
        RealType dt,
        const VectorX<2, RealType> &gravity)
    {
      for (auto i = 0; i < rigidbodies.size(); i++)
      {
        auto body = rigidbodies[i];
        if (body->GetInvMass() == static_cast<RealType>(0.0))
          continue;
        auto acc = body->GetForce() * body->GetInvMass() + gravity;

        body->AddVelocity(acc * (dt / static_cast<RealType>(2.0)));
        // KIRI_LOG_DEBUG("UpdateVelocity Isstatic={0}; Acc={1},{2}; Vel={3},{4}", body->IsStatic(), acc.x, acc.y, body->GetVelocity().x, body->GetVelocity().y);
        body->AddAngularVelocity(body->GetTorque() * body->GetInvInertia() * (dt / static_cast<RealType>(2.0)));
      }
    }

    void HandleCollision(int iteration, RealType dt, const VectorX<2, RealType> &gravity)
    {
      for (auto i = 0; i < mCollisions.size(); i++)
        mCollisions[i]->ComputeContactSurfaceInfo(dt, gravity);

      for (auto iter = 0; iter < iteration; iter++)
        for (auto i = 0; i < mCollisions.size(); i++)
          mCollisions[i]->ComputeImpluse();
    }

    void PositionalCorrection()
    {
      for (auto i = 0; i < mCollisions.size(); i++)
        mCollisions[i]->PositionalCorrection();
    }

    void Advect(
        const std::vector<std::shared_ptr<RigidBody<RealType>>> &rigidbodies,
        RealType dt)
    {
      for (auto i = 0; i < rigidbodies.size(); i++)
      {
        auto body = rigidbodies[i];
        if (body->GetInvMass() == static_cast<RealType>(0.0))
          continue;
        // KIRI_LOG_DEBUG("Advect Isstatic={0}; body->GetVelocity().x, body->GetVelocity().y={1},{2}", body->IsStatic(), body->GetVelocity().x, body->GetVelocity().y);
        // KIRI_LOG_DEBUG("GetAngularVelocity={0}; Before Update Ori={1}", body->GetAngularVelocity(), body->GetOrientation());
        body->AddPosition(body->GetVelocity() * dt);
        body->AddOrientation(body->GetAngularVelocity() * dt);
        // KIRI_LOG_DEBUG("After Update Ori={0}",body->GetOrientation());
      }
    }

    void ClearForces(const std::vector<std::shared_ptr<RigidBody<RealType>>> &rigidbodies)
    {
      for (auto i = 0; i < rigidbodies.size(); i++)
      {
        auto body = rigidbodies[i];
        body->ClearForces();
      }
    }
  };

} // namespace PHY::RIGIDBODY

#endif /* _RIGIDBODY_SOLVER_H_ */