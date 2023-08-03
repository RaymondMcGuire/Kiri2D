/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-09 15:59:29
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-08-03 16:44:13
 * @FilePath: \Kiri2D\core\include\kiri2d\proto_sphere\proto_sphere_packing_sdf.h
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG, All Rights Reserved.
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
        explicit ProtoSpherePackingSDF2D(const HDV::Voronoi::VoronoiPolygon2Ptr &boundary, double cell_size = 10.0)
            : mBoundary(std::move(boundary))
        {
            mSDF2D = std::make_shared<HDV::SDF::PolygonSDF2D>(mBoundary, cell_size);
            mSDF2D->computeSDF();
            initParticles();
        }

        virtual ~ProtoSpherePackingSDF2D()
        {
        }

        const std::vector<Vector3D> &currentSpheres() const { return mCurrentSpheres; }
        const std::vector<Vector3D> &insertedSpheres() const { return mInsertedSpheres; }
        const std::vector<Vector3F> &insertedSpheresColor() const { return mInsertedSpheresColor; }

        virtual void convergePrototype()
        {
            if (mDrawCurrentSpheres)
            {
                mDrawCurrentSpheres = false;
                mSDF2D->updateSDFWithSpheres(mInsertedSpheres);
                initParticles();
                KIRI_LOG_DEBUG("re append particles!={0}; inserted number={1}", mCurrentSpheres.size(), mInsertedSpheres.size());
            }

            mAllConverged = true;
            for (auto i = 0; i < mCurrentSpheres.size(); i++)
            {
                if (mConverges[i] || mIterNums[i] > 50)
                    continue;
                else
                    mAllConverged = false;

                // KIRI_LOG_DEBUG("itertaion!");

                auto sphere = mCurrentSpheres[i];
                auto pos = Vector2D(sphere.x, sphere.y);
                auto [min_dist, q_c] = mSDF2D->getSDF(pos);

                auto epsilon = this->coolingFunc(mIterNums[i], 0.08, 0.8);
                auto current_move = epsilon * (pos - q_c);

                if (current_move.length() <= min_dist)
                    pos += current_move;

                auto radius = min_dist;
                auto current_move_len = current_move.length();
                // KIRI_LOG_DEBUG("current_move_len={0}", current_move_len);
                if (current_move_len < 1e-6)
                    mConverges[i] = true;

                mCurrentSpheres[i] = Vector3D(pos.x, pos.y, radius);
                mIterNums[i]++;
            }

            if (mAllConverged && !mDrawCurrentSpheres)
            {
                // KIRI_LOG_DEBUG("sort and add !");

                std::sort(mCurrentSpheres.begin(), mCurrentSpheres.end(), [](const auto &lhs, const auto &rhs)
                          { return lhs.z > rhs.z; });

                for (auto i = 0; i < mCurrentSpheres.size(); i++)
                {
                    auto cur_pos = Vector2D(mCurrentSpheres[i].x, mCurrentSpheres[i].y);
                    auto cur_radius = mCurrentSpheres[i].z;

                    auto overlapping = false;
                    for (auto j = 0; j < mInsertedSpheres.size(); j++)
                    {
                        auto other_pos = Vector2D(mInsertedSpheres[j].x, mInsertedSpheres[j].y);
                        auto other_radius = mInsertedSpheres[j].z;
                        auto dist = (other_pos - cur_pos).length() - (cur_radius + other_radius);
                        if (dist < 0)
                        {
                            overlapping = true;
                            break;
                        }
                    }

                    if (!overlapping)
                    {
                        mInsertedSpheres.emplace_back(mCurrentSpheres[i]);
                        mInsertedSpheresColor.emplace_back(Vector3F(Random::get(0.0, 1.0), Random::get(0.0, 1.0), Random::get(0.0, 1.0)));
                    }
                }

                mDrawCurrentSpheres = true;
                // mInsertedSpheres.insert(std::end(mInsertedSpheres), std::begin(mInsertedSpheresTmp), std::end(mInsertedSpheresTmp));
                // mInsertedSpheresColor.insert(std::end(mInsertedSpheresColor), std::begin(mInsertedSpheresColorTmp), std::end(mInsertedSpheresColorTmp));
            }
        }

    protected:
        void initParticles()
        {
            mCurrentSpheres.clear();
            mIterNums.clear();
            mConverges.clear();
            // mInsertedSpheresTmp.clear();
            // mInsertedSpheresColorTmp.clear();

            auto data = mSDF2D->placeGridPoints();
            KIRI_LOG_DEBUG("sphere array num={0}; data num={1}", mCurrentSpheres.size(), data.size());
            for (auto i = 0; i < data.size(); i++)
                mCurrentSpheres.emplace_back(Vector3D(data[i].x, data[i].y, 0.0));

            mIterNums.resize(mCurrentSpheres.size(), 0);
            mConverges.resize(mCurrentSpheres.size(), false);
        }

        virtual double coolingFunc(const int iter, const double init, const double k)
        {
            return init * exp(-k * iter);
        }

        bool mAllConverged = false, mDrawCurrentSpheres = false;
        std::vector<UInt> mIterNums;
        std::vector<bool> mConverges;
        std::vector<Vector3D> mCurrentSpheres;
        std::vector<Vector3D> mInsertedSpheresTmp, mInsertedSpheres;
        std::vector<Vector3F> mInsertedSpheresColorTmp, mInsertedSpheresColor;
        HDV::Voronoi::VoronoiPolygon2Ptr mBoundary;
        HDV::SDF::PolygonSDF2DPtr mSDF2D;
    };

} // namespace PSPACK

#endif /* _PROTO_SPHERE_PACK_SDF_H_ */