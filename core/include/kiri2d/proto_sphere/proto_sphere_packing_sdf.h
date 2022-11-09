/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-09 15:59:29
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-09 16:00:15
 * @FilePath: \Kiri2D\core\include\kiri2d\proto_sphere\proto_sphere_packing_sdf.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _PROTO_SPHERE_PACK_SDF_H_
#define _PROTO_SPHERE_PACK_SDF_H_

#pragma once

#include <kiri2d/hdv_toolkit/sdf/sdf2d.h>

namespace PSPACK
{
    class ProtoSpherePackingSDF2D
    {
    public:
        explicit ProtoSpherePackingSDF2D(const HDV::Voronoi::VoronoiPolygon2Ptr &boundary)
            : mBoundary(std::move(boundary))
        {
            mIterNum = 0;
            mCurrentSphereRadius = 0.0;
            mCurrentSphere = mBoundary->rndInnerPoint();

            mSDF2D = std::make_shared<HDV::SDF::PolygonSDF2D>(mBoundary, 1.0);
            mSDF2D->computeSDF();
        }

        virtual ~ProtoSpherePackingSDF2D()
        {
        }

        const Vector2D currentSphere() const { return mCurrentSphere; }
        const double currentSphereRadius() const { return mCurrentSphereRadius; }
        const std::vector<Vector3D> &insertedSpheres() const { return mInsertedSpheres; }
        const std::vector<Vector3F> &insertedSpheresColor() const { return mInsertedSpheresColor; }

        virtual void convergePrototype()
        {

            auto [min_dist, q_c] = mSDF2D->getSDF(mCurrentSphere);

            auto epsilon = this->coolingFunc(mIterNum, 0.08, 0.8);
            auto current_move = epsilon * (mCurrentSphere - q_c);

            if (current_move.length() <= min_dist)
                mCurrentSphere += current_move;

            mCurrentSphereRadius = min_dist;
            auto current_move_len = current_move.length();
            if (current_move_len < 1e-6)
            {
                mIterNum = 0;

                mInsertedSpheres.emplace_back(Vector3D(mCurrentSphere.x, mCurrentSphere.y, mCurrentSphereRadius));
                mInsertedSpheresColor.emplace_back(Vector3F(Random::get(0.0, 1.0), Random::get(0.0, 1.0), Random::get(0.0, 1.0)));

                mSDF2D->updateSDFWithSpheres(mInsertedSpheres);
                auto pos_flag = false;
                while (!pos_flag)
                {
                    pos_flag = true;
                    mCurrentSphere = mBoundary->rndInnerPoint();
                    auto [new_min_dist, new_q_c] = mSDF2D->getSDF(mCurrentSphere);

                    if (new_min_dist <= 0)
                        pos_flag = false;
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
        HDV::SDF::PolygonSDF2DPtr mSDF2D;
    };

} // namespace PSPACK

#endif /* _PROTO_SPHERE_PACK_SDF_H_ */