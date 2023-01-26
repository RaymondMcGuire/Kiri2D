/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2023-01-11 14:46:17
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-25 23:26:05
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\rigidbody_system.h
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
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
            VectorX<2, RealType> lowest,
            VectorX<2, RealType> highest,
            int maxNum = 100,
            RealType dt = static_cast<RealType>(1.0 / 60.0),
            int iteration = 10,
            RealType gravityScale = static_cast<RealType>(5.0))
            : mDt(dt), mIteration(iteration), mStepNum(0), mAccumulator(static_cast<RealType>(0.0)),
              mGravityScale(gravityScale), mMaxBodyNum(maxNum),
              mSimWorldLowest(lowest), mSimWorldHighest(highest)
        {
            mGravity = VectorX<2, RealType>(static_cast<RealType>(0.0), static_cast<RealType>(-9.8 * mGravityScale));
            mSolver = std::make_shared<RigidBodySolver<RealType>>();
        }

        virtual ~RigidBodySystem()
        {
        }

        void UpdateSystem()
        {
            this->CheckSimWorldBoundary();

            mAccumulator += mPerFrameTimer.elapsed();
            mPerFrameTimer.restart();
            mAccumulator = std::clamp(mAccumulator, static_cast<RealType>(0.0), static_cast<RealType>(0.1));

            while (mAccumulator >= mDt)
            {
                mSolver->UpdateSolver(mObjects, mIteration, mDt, mGravity);
                mAccumulator -= mDt;
            }

            mStepNum++;
        }

        void Render(const std::shared_ptr<KiriScene2D<RealType>> &scene, const std::shared_ptr<KiriRenderer2D<RealType>> &renderer, const RealType scale)
        {
            auto offset = VectorX<2, RealType>(scene->GetWindowWidth(), scene->GetWindowHeight()) / static_cast<RealType>(2.0);
            std::vector<KiriCircle2<RealType>> circles;
            std::vector<KiriLine2<RealType>> lines;
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
                    circles.emplace_back(KiriCircle2<RealType>(pos, VectorX<3, RealType>(0.f, 1.f, 1.f), shape_circle->GetRadius() * scale, false));
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

            scene->AddLines(lines);
            scene->AddCircles(circles);
            renderer->DrawCanvas();
        }

        void AddObject(const ShapePtr<RealType> &shape, const VectorX<2, RealType> &pos, const bool staticObj = false)
        {
            auto rigidbody = std::make_shared<RigidBody<RealType>>(pos);
            CompositeShapeRigidBody(shape, rigidbody);
            if (staticObj)
                rigidbody->SetAsStatic();
            mObjects.emplace_back(rigidbody);
        }

        void EmitRndPolygon(const RealType size, const VectorX<2, RealType> &lowest, const VectorX<2, RealType> &highest, const int frequence)
        {
            if (mObjects.size() >= mMaxBodyNum || mStepNum % frequence != 0)
                return;

            auto shape = std::make_shared<RIGIDBODY::Polygon<RealType>>();
            shape->SetAsRandomConvexShape(size);

            auto rnd_pos = VectorX<2, RealType>(Random::get<RealType>(lowest.x, highest.x), Random::get<RealType>(lowest.y, highest.y));
            this->AddObject(shape, rnd_pos);
            auto body = shape->GetBody();
            body.lock()->SetRestitution(static_cast<RealType>(0.5));
            body.lock()->SetStaticFriction(static_cast<RealType>(0.2));
            body.lock()->SetDynamicFriction(static_cast<RealType>(0.1));
        }

    private:
        RealType mDt;
        RealType mAccumulator;

        int mIteration, mStepNum;
        int mMaxBodyNum;
        VectorX<2, RealType> mSimWorldLowest, mSimWorldHighest;

        RealType mGravityScale;
        VectorX<2, RealType> mGravity;

        KiriTimer mPerFrameTimer;
        std::shared_ptr<RigidBodySolver<RealType>> mSolver;
        std::vector<std::shared_ptr<RigidBody<RealType>>> mObjects;

        void CheckSimWorldBoundary()
        {
            mObjects.erase(
                std::remove_if(
                    mObjects.begin(), mObjects.end(),
                    [=](auto &rigidbody)
                    {
                        auto pos = rigidbody->GetPosition();
                        if (pos.x < mSimWorldLowest.x || pos.x > mSimWorldHighest.x || pos.y < mSimWorldLowest.y || pos.y > mSimWorldHighest.y)
                            return true;
                        return false;
                    }),
                mObjects.end());
        }
    };

} // namespace PHY::RIGIDBODY

#endif /* _RIGIDBODY_SYSTEM_H_ */