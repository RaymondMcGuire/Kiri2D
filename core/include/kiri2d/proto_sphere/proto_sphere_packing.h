/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-03 16:19:02
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-03 16:19:05
 * @FilePath: \Kiri2D\core\include\kiri2d\proto_sphere\proto_sphere_packing.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _PROTO_SPHERE_PACK_H_
#define _PROTO_SPHERE_PACK_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_polygon.h>

namespace PSPACK
{
    class ProtoSpherePacking2D
    {
    public:
        explicit ProtoSpherePacking2D(const HDV::Voronoi::VoronoiPolygon2Ptr &boundary)
            : mBoundary(std::move(boundary))
        {
            mIterNum = 0;
            mCurrentSphereRadius = 0.0;
            mCurrentSphere = mBoundary->rndInnerPoint();
        }

        virtual ~ProtoSpherePacking2D()
        {
        }

        const Vector2D currentSphere() const { return mCurrentSphere; }
        const double currentSphereRadius() const { return mCurrentSphereRadius; }
        const std::vector<Vector3D> &insertedSpheres() const { return mInsertedSpheres; }
        const std::vector<Vector3F> &insertedSpheresColor() const { return mInsertedSpheresColor; }

        virtual void convergePrototype()
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
            auto current_move = epsilon * (mCurrentSphere - q_c);

            if (current_move.length() <= q_c.distanceTo(mCurrentSphere))
                mCurrentSphere += current_move;

            mCurrentSphereRadius = mCurrentSphere.distanceTo(q_c);
            auto current_move_len = current_move.length();
            if (current_move_len < 1e-6)
            {
                mIterNum = 0;

                mInsertedSpheres.emplace_back(Vector3D(mCurrentSphere.x, mCurrentSphere.y, mCurrentSphereRadius));
                mInsertedSpheresColor.emplace_back(Vector3F(Random::get(0.0, 1.0), Random::get(0.0, 1.0), Random::get(0.0, 1.0)));

                auto pos_flag = false;
                while (!pos_flag)
                {
                    pos_flag = true;
                    mCurrentSphere = mBoundary->rndInnerPoint();
                    for (auto i = 0; i < mInsertedSpheres.size(); i++)
                    {
                        auto point = mInsertedSpheres[i];
                        auto dist = mCurrentSphere.distanceTo(Vector2D(point.x, point.y)) - point.z;
                        if (dist <= 0)
                        {
                            pos_flag = false;
                            break;
                        }
                    }
                }
            }

            mIterNum++;
        }

    protected:
        virtual double coolingFunc(const int iter, const double init, const double k)
        {
            return init * exp(-k * iter);
        }

        UInt mIterNum;
        double mCurrentSphereRadius;
        Vector2D mCurrentSphere;
        std::vector<Vector3D> mInsertedSpheres;
        std::vector<Vector3F> mInsertedSpheresColor;
        HDV::Voronoi::VoronoiPolygon2Ptr mBoundary;
    };

} // namespace PSPACK

#endif /* _PROTO_SPHERE_PACK_H_ */