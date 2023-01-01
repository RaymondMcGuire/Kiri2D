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

#include <kiri_timer.h>
#include <kiri2d/physics/rigidbody/rigidbody_solver.h>

namespace KIRI2D::PHY::RIGIDBODY
{
    template <class RealType>
    class RigidBodySystem
    {
    public:
        explicit RigidBodySystem(
            RealType dt = static_cast<RealType>(1.0 / 60.0),
            int iteration = 10,
            RealType gravityScale = static_cast<RealType>(5.0))
            : mDt(dt), mIteration(iteration), mAccumulator(static_cast<RealType>(0.0)),
              mGravityScale(gravityScale)
        {
            mGravity = VectorX<2, RealType>(static_cast<RealType>(0.0), static_cast<RealType>(-9.8 * mGravityScale));
            mSolver = std::make_shared<RigidBodySolver<RealType>>();
        }

        virtual ~RigidBodySystem()
        {
        }

        void UpdateSystem()
        {
            mAccumulator += mPerFrameTimer.elapsed();
            mPerFrameTimer.restart();
            mAccumulator = std::clamp(mAccumulator, static_cast<RealType>(0.0), static_cast<RealType>(0.1));

            while (mAccumulator >= mDt)
            {
                mSolver->UpdateSolver(mObjects, mIteration, mDt, mGravity);
                mAccumulator -= mDt;
            }
        }

        void Render(const KiriScene2DPtr &scene, const KiriRenderer2DPtr &renderer)
        {
            auto offset = VectorX<2, RealType>(250.0, 250.0);
            auto scale = 10;
            std::vector<KiriCircle2> circles;
            std::vector<KiriLine2> lines;
            for (auto i = 0; i < mObjects.size(); i++)
            {
                auto shape = mObjects[i]->GetShape();
                auto obj_pos = mObjects[i]->GetPosition();
                switch (shape->GetType())
                {
                case CIRCLE:
                {
                    auto shape_circle = std::dynamic_pointer_cast<Circle<RealType>>(shape);
                    auto pos = obj_pos.mul(scale) + offset;
                    circles.emplace_back(KiriCircle2(pos, Vector3F(0.f, 1.f, 1.f), shape_circle->GetRadius() * scale, false));
                    auto c = std::cos(mObjects[i]->GetOrientation());
                    auto s = std::sin(mObjects[i]->GetOrientation());
                    auto rline = VectorX<2, RealType>(-s * shape_circle->GetRadius() * scale, c * shape_circle->GetRadius() * scale);
                    rline += pos;
                    lines.emplace_back(KiriLine2(pos, rline));
                    break;
                }

                case POLYGON:
                {
                    auto shape_polygon = std::dynamic_pointer_cast<Polygon<RealType>>(shape);
                    auto vertices = shape_polygon->GetVertices();
                    auto vert_num = shape_polygon->GetVerticesNum();
                    auto rot_mat = shape_polygon->GetRotateMatrix();
                    for (auto v = 0; v < vert_num; v++)
                    {
                        auto point1 = (obj_pos + rot_mat * vertices[v]).mul(scale) + offset;
                        auto point2 = (obj_pos + rot_mat * vertices[(v + 1) % vert_num]).mul(scale) + offset;
                        lines.emplace_back(KiriLine2(point1, point2));
                    }
                    break;
                }
                }
            }

            scene->addLines(lines);
            scene->addCircles(circles);
            renderer->drawCanvas();
        }

        void AddObject(const ShapePtr<RealType> &shape, const VectorX<2, RealType> &pos, const bool staticObj = false)
        {
            auto rigidbody = std::make_shared<RigidBody<RealType>>(pos);
            CompositeShapeRigidBody(shape, rigidbody);
            if (staticObj)
                rigidbody->SetAsStatic();
            mObjects.emplace_back(rigidbody);
        }

    private:
        RealType mDt;
        RealType mAccumulator;
        int mIteration;

        RealType mGravityScale;
        VectorX<2, RealType> mGravity;

        KiriTimer mPerFrameTimer;
        std::shared_ptr<RigidBodySolver<RealType>> mSolver;
        std::vector<std::shared_ptr<RigidBody<RealType>>> mObjects;
    };

} // namespace PHY::RIGIDBODY

#endif /* _RIGIDBODY_SYSTEM_H_ */