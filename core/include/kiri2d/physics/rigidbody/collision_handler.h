/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-17 19:27:28
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-17 19:30:22
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\collision_handler.h
 * @Description:,
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _COLLISION_HANDLER_H_
#define _COLLISION_HANDLER_H_

#pragma once

#include <kiri2d/physics/rigidbody/rigidbody.h>

namespace PHY::RIGIDBODY
{
    template <class RealType>
    class CollisionHandler
    {
    public:
        explicit CollisionHandler(
            const std::shared_ptr<RigidBody<RealType>> &bodyA,
            const std::shared_ptr<RigidBody<RealType>> &bodyB)
            : mBodyA(bodyA), mBodyB(bodyB)
        {
            mMixRestitution = std::min(mBodyA->GetRestitution(), mBodyB->GetRestitution());
        }

        virtual ~CollisionHandler()
        {
        }

        void ComputeImpluse()
        {
            auto a_im = mBodyA->GetInvMass();
            auto b_im = mBodyB->GetInvMass();
            if (std::abs(a_im + b_im) < MEpsilon<RealType>())
            {
                InfiniteMassCorrection();
                return;
            }
        }

    private:
        RealType mMixRestitution;
        std::shared_ptr<RigidBody<RealType>> mBodyA, mBodyB;

        void InfiniteMassCorrection()
        {
            mBodyA->SetVelocity(VectorX<2, RealType>(static_cast<RealType>(0.0)));
            mBodyB->SetVelocity(VectorX<2, RealType>(static_cast<RealType>(0.0)));
        }
    };

} // namespace PHY::RIGIDBODY

#endif /* _COLLISION_HANDLER_H_ */