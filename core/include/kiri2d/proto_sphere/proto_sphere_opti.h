/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-06 20:39:42
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-07 10:21:14
 * @FilePath: \Kiri2D\core\include\kiri2d\proto_sphere\proto_sphere_opti.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _PROTO_SPHERE_OPTI_H_
#define _PROTO_SPHERE_OPTI_H_

#pragma once

#include <kiri2d/proto_sphere/proto_sphere_packing.h>

namespace PSPACK
{
    class ProtoSphereOpti : public ProtoSpherePacking2D
    {
    public:
        explicit ProtoSphereOpti(const HDV::Voronoi::VoronoiPolygon2Ptr &boundary, std::vector<double> target_radius)
            : ProtoSpherePacking2D(boundary), mTargetRadiusArray(target_radius)
        {
            std::sort(mTargetRadiusArray.begin(), mTargetRadiusArray.end(), std::greater<double>());
            mRadiusMin = mTargetRadiusArray[0];
            mRadiusMax = mTargetRadiusArray[mTargetRadiusArray.size() - 1];
            ReAllocateTargetRadius();
            mCurrentTargetRadius = mCurrentTargetRadiusArray[mCurrentIdx++];
            mCurrentSphere = boundary->rndInnerPointWithDistConstrain(mRadiusMin);
        }

        virtual ~ProtoSphereOpti()
        {
        }

        virtual void convergePrototype() override
        {
            if (mFinished)
                return;

            if (mInsertedSpheres.size() >= mCurrentTargetRadiusArray.size())
            {
                mFinished = true;
                for (int i = 0; i < mInsertedSpheres.size(); i++)
                {
                    KIRI_LOG_DEBUG("current radius={0}", mInsertedSpheres[i].z);
                }
                return;
            }

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

            auto radius_dist = mCurrentTargetRadius - mCurrentSphereRadius;
            current_move = radius_dist * 0.1 * (mCurrentSphere - q_c).normalized();

            if (current_move.length() <= q_c.distanceTo(mCurrentSphere))
                mCurrentSphere += current_move;

            mCurrentSphereRadius = mCurrentSphere.distanceTo(q_c);
            // KIRI_LOG_DEBUG("current radius ={0}; target radius ={1}", mCurrentSphereRadius, mCurrentTargetRadius);

            if (abs(radius_dist) < mCurrentTargetRadius * mErrorRate || mIterNum > mMaxSingleIter)
            {
                mIterNum = 0;

                // check satifty the min radius

                mInsertedSpheres.emplace_back(Vector3D(mCurrentSphere.x, mCurrentSphere.y, mCurrentSphereRadius));
                mInsertedSpheresColor.emplace_back(Vector3F(Random::get(0.0, 1.0), Random::get(0.0, 1.0), Random::get(0.0, 1.0)));

                if (mInsertedSpheres.size() < mCurrentTargetRadiusArray.size())
                {
                    KIRI_LOG_DEBUG("generate new start!");
                    auto pos_flag = false;
                    auto sample_iter_num = 0;
                    while (!pos_flag)
                    {
                        pos_flag = true;
                        mCurrentSphere = mBoundary->rndInnerPointWithDistConstrain(mRadiusMin);
                        for (auto i = 0; i < mInsertedSpheres.size(); i++)
                        {
                            auto point = mInsertedSpheres[i];
                            auto dist =
                                mCurrentSphere.distanceTo(Vector2D(point.x, point.y)) - point.z;

                            auto constrain_error_rate = 1.0 - mErrorRate;
                            if (sample_iter_num > mMaxSingleIter * 10)
                                constrain_error_rate = 1.0 - mErrorRate * (sample_iter_num / mMaxSingleIter);

                            if (dist <= mCurrentTargetRadius * constrain_error_rate)
                            {
                                pos_flag = false;
                                break;
                            }
                        }
                        sample_iter_num++;
                    }
                    mCurrentTargetRadius = mCurrentTargetRadiusArray[mCurrentIdx++];
                    KIRI_LOG_DEBUG("generate new finish!");
                }
            }

            mIterNum++;
        }

    private:
        bool mFinished = false;
        int mRangeSize = 100, mCurrentIdx = 0, mMaxSingleIter = 100;
        double mErrorRate = 0.05;
        double mCurrentTargetRadius;
        double mRadiusMin, mRadiusMax;
        std::vector<double> mCurrentTargetRadiusArray, mTargetRadiusArray;

        void ReAllocateTargetRadius()
        {
            mCurrentTargetRadiusArray = std::vector<double>(mTargetRadiusArray.begin(), mTargetRadiusArray.begin() + mRangeSize);
            for (int i = 0; i < mCurrentTargetRadiusArray.size(); i++)
            {
                KIRI_LOG_DEBUG("target radius={0}", mCurrentTargetRadiusArray[i]);
            }
        }
    };

} // namespace PSPACK

#endif /* _PROTO_SPHERE_OPTI_H_ */