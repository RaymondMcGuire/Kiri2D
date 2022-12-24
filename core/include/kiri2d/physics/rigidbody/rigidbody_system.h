/*** 
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-22 18:59:20
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-24 18:44:34
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\rigidbody_system.h
 * @Description: 
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved. 
 */
#ifndef _RIGIDBODY_SYSTEM_H_
#define _RIGIDBODY_SYSTEM_H_

#pragma once
 
#include <kiri2d/physics/rigidbody/collision_handler.h>

namespace PHY::RIGIDBODY
{
    template <class RealType>
    class RigidBodySystem
    {
    public:
        explicit RigidBodySystem(
            RealType dt , int iteration = 10 
        ):mDt(dt), mIteration(iteration)
        {
        }

        virtual ~RigidBodySystem()
        {
        }

        void AddObject(const ShapePtr& shape, const VectorX<2, RealType>& pos)
        {
            auto rigid_body = std::make_shared<RigdiBody<RealType>>(pos);
            CompositeShapeRigidBody(shape,rigid_body);
            mObjects.emplace_back(rigid_body);
        }

    private:
         RealType mDt;
        int mIteration;

        std::vector<std::shared_ptr<RigdiBody<RealType>>> mObjects;
 
    };

} // namespace PHY::RIGIDBODY

#endif /* _RIGIDBODY_SYSTEM_H_ */