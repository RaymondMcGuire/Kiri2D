/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-03 17:18:41
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-03 17:19:05
 * @FilePath: \Kiri2D\core\include\kiri2d\proto_sphere\proto_sphere_bon20.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _PROTO_SPHERE_BON20_H_
#define _PROTO_SPHERE_BON20_H_

#pragma once

#include <kiri2d/proto_sphere/proto_sphere_packing.h>

namespace PSPACK
{
    class ProtoSphereBon20 : public ProtoSpherePacking2D
    {
    public:
        explicit ProtoSphereBon20(const HDV::Voronoi::VoronoiPolygon2Ptr &boundary, double radius_min, double radius_max)
            : ProtoSpherePacking2D(boundary), mRadiusMin(radius_min), mRadiusMax(radius_max)
        {
            mCurrentTargetRadius = Random::get(mRadiusMin, mRadiusMax);
            mCurrentSphere = boundary->rndInnerPointWithDistConstrain(mRadiusMin);
        }

        virtual ~ProtoSphereBon20()
        {
        }

        virtual void convergePrototype() override
        {
            auto q_c = mBoundary->computeMinDistPointInPoly(mCurrentSphere);
            // check inserted sphere dist
            for (auto i = 0; i < mInsertedSpheres.size(); i++)
            {
                auto point = mInsertedSpheres[i];
                auto dist = mCurrentSphere.distanceTo(Vector2D(point.x, point.y)) - point.z;
                if (dist < q_c.distanceTo(mCurrentSphere))
                    q_c = Vector2D(point.x, point.y) + point.z * (mCurrentSphere - Vector2D(point.x, point.y)).normalized();
            }

            auto epsilon = this->coolingFunc(mIterNum, 0.08, 0.8);
            auto current_move = Vector2D(0.0);
            if (mCurrentTargetRadius > mCurrentSphere.distanceTo(q_c))
                current_move = epsilon * (mCurrentSphere - q_c).normalized();
            else
                current_move = epsilon * (q_c - mCurrentSphere).normalized();

            if (current_move.length() <= q_c.distanceTo(mCurrentSphere))
                mCurrentSphere += current_move;

            mCurrentSphereRadius = mCurrentSphere.distanceTo(q_c);
            auto current_move_len = current_move.length();
            if (current_move_len < 1e-6)
            {
                mIterNum = 0;

                // check satifty the min radius
                if (mCurrentSphere.distanceTo(q_c) >= mRadiusMin &&
                    mCurrentSphere.distanceTo(q_c) <= mRadiusMax)
                {
                    mInsertedSpheres.emplace_back(Vector3D(mCurrentSphere.x, mCurrentSphere.y, mCurrentSphereRadius));
                    mInsertedSpheresColor.emplace_back(Vector3F(Random::get(0.0, 1.0), Random::get(0.0, 1.0), Random::get(0.0, 1.0)));
                }

                auto pos_flag = false;
                while (!pos_flag)
                {
                    pos_flag = true;
                    mCurrentSphere = mBoundary->rndInnerPointWithDistConstrain(mRadiusMin);
                    mCurrentTargetRadius = Random::get(mRadiusMin, mRadiusMax);
                    for (auto i = 0; i < mInsertedSpheres.size(); i++)
                    {
                        auto point = mInsertedSpheres[i];
                        auto dist =
                            mCurrentSphere.distanceTo(Vector2D(point.x, point.y)) - point.z;
                        if (dist <= mRadiusMin)
                        {
                            pos_flag = false;
                            break;
                        }
                    }
                }
            }

            mIterNum++;
        }

    private:
        double mCurrentTargetRadius;
        double mRadiusMin, mRadiusMax;
    };

} // namespace PSPACK

#endif /* _PROTO_SPHERE_BON20_H_ */