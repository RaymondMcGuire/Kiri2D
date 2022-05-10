/***
 * @Author: Xu.WANG
 * @Date: 2022-01-10 22:51:36
 * @LastEditTime: 2022-01-10 23:03:16
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_MS_SAMPLER3_H_
#define _HDV_MS_SAMPLER3_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/power_diagram.h>
#include <kiri_pbs_cuda/sampler/cuda_shape_sampler.cuh>

namespace HDV::Sampler
{
    class MultiSizedSampler3D
    {
    public:
        explicit MultiSizedSampler3D(const std::shared_ptr<KIRI::CudaShapeSampler> &sdf)
            : mCudaSDF(std::move(sdf))
        {
            mPowerDiagram = std::make_shared<Voronoi::PowerDiagram3D>();
        }

        virtual ~MultiSizedSampler3D()
        {
        }

        std::vector<Primitives::Vertex3Ptr> sites()
        {
            return mPowerDiagram->sites();
        }

        void addSite(double x, double y, double z, double radius)
        {
            auto site = std::make_shared<Voronoi::VoronoiSite3>(x, y, z, mSiteCounter++);
            site->setRadius(radius);
            mPowerDiagram->addSite(site);
        }

        void setMaxiumNum(int num)
        {
            mMaxiumNum = num;
        }

        void setBoundaryPolygon(std::vector<csgjscpp::Polygon> boundary)
        {
            mPowerDiagram->setBoundaryPolygon(boundary);
        }

        void setRadiusDist(std::vector<double> dist)
        {
            mUsrDefinedRadiusDist = dist;
        }

        void setRadiusDistProb(std::vector<double> prob)
        {
            mUsrDefinedRadiusDistProb = prob;
        }

        void init()
        {
            reset();

            computeBoundaryArea();

            mPowerDiagram->init();
        }

        std::vector<Vector4D> sampledSpheres()
        {
            std::vector<Vector4D> data;
            auto sites = mPowerDiagram->sites();
            for (int i = 0; i < sites.size(); i++)
            {
                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                if (site_i->polygon())
                {
                    auto cen = site_i->polygon()->centroid();
                    if (mCudaSDF->checkPointsInside(make_float3(cen.x, cen.y, cen.z)))
                    {
                        auto radius = site_i->polygon()->computeMinDisInPoly(cen);
                        data.emplace_back(Vector4D(cen.x, cen.y, cen.z, radius));
                    }
                    else
                    {
                        KIRI_LOG_ERROR("point not inside polygon");
                    }
                }
                else
                {
                    KIRI_LOG_ERROR("no polygon");
                }
            }
            return data;
        }

        bool checkVoroCell()
        {
            std::vector<int> remove;

            auto sites = mPowerDiagram->sites();
            for (int i = 0; i < sites.size(); i++)
            {

                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                if (mCudaSDF->checkPointsInside(make_float3(site_i->X(), site_i->Y(), site_i->Z())))
                {
                    if (!site_i->polygon())
                    {
                        remove.emplace_back(site_i->id());
                        auto points = mCudaSDF->GetInsidePoints(1);
                        // KIRI_LOG_INFO("change site pos={0},{1},{2}", points[0].x, points[0].y, points[0].z);
                        site_i->Set(points[0].x, points[0].y, points[0].z);
                    }
                    else
                    {
                        auto centroid = site_i->polygon()->centroid();
                        if (!mCudaSDF->checkPointsInside(make_float3(centroid.x, centroid.y, centroid.z)))
                        {
                            remove.emplace_back(site_i->id());
                            auto points = mCudaSDF->GetInsidePoints(1);
                            // KIRI_LOG_INFO("change site pos={0},{1},{2}", points[0].x, points[0].y, points[0].z);
                            site_i->Set(points[0].x, points[0].y, points[0].z);
                        }
                    }
                }
            }

            // if (!remove.empty())
            // {
            //     mPowerDiagram->removeVoroSitesByIndexArray(remove);
            //     return true;
            // }

            return false;
        }

        void compute()
        {
            mCurIteration++;

            mPowerDiagram->move2Centroid();
            this->computeWeightsError();
            this->adaptWeights();

            mPowerDiagram->compute();

            if (checkVoroCell())
            {
                mPowerDiagram->compute();
            }

            mPowerDiagram->exportObj();
            KIRI_LOG_DEBUG("error={0}", mCurGlobalWeightError);
        }

        void reset()
        {
            mCurIteration = 0;
            mCompleteArea = 0.0;
            mCurGlobalPorosity = 0.0;
        }

        void computeBoundaryArea()
        {
            auto boundary = mPowerDiagram->GetBoundary();
            if (!boundary)
            {
                KIRI_LOG_ERROR("Not Set Boundary!");
                return;
            }
            mCompleteArea = mPowerDiagram->GetBoundary()->volume();
            KIRI_LOG_DEBUG("Boundary Volume={0}", mCompleteArea);
        }

        double globalAreaError()
        {
            auto error = 0.0;
            auto sites = mPowerDiagram->sites();

            for (int i = 0; i < sites.size(); i++)
            {
                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                auto n = site_i->neighbors().size();
                if (n > 0)
                {
                    auto current_area = (!site_i->polygon()) ? 0.0 : site_i->polygon()->volume();
                    auto target_area = 4.0 / 3.0 * kiri_math_mini::pi<double>() * std::pow(site_i->radius(), 3.0);
                    error += std::abs(target_area - current_area) / (mCompleteArea * 2.0);
                    // KIRI_LOG_DEBUG("current_area={0};target_area={1},error={2}", current_area, target_area, error);
                }
                else
                {
                    KIRI_LOG_ERROR("No Neighbor Sites");
                }
            }
            return error;
        }

        double globalAvgDistance()
        {
            double sum = 0.0;
            int num = 0;
            auto site = mPowerDiagram->sites();

            for (int i = 0; i < site.size(); i++)
            {
                if (site[i]->isBoundaryVertex())
                    continue;

                if (!site[i]->neighbors().empty())
                {
                    for (auto neighbor : site[i]->neighbors())
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex3>(neighbor);
                        auto distance = site[i]->Distance(sn);
                        if (distance != distance)
                            KIRI_LOG_ERROR("id={0}:({1},{2},{3})--neigh={4}:({5},{6},{7})",
                                           site[i]->id(), site[i]->X(), site[i]->Y(), site[i]->Z(),
                                           sn->id(), sn->X(), sn->Y(), sn->Z());
                        sum += distance;
                        num++;
                    }
                }
            }

            if (num == 0)
            {
                KIRI_LOG_ERROR("globalAvgDistance:: no neighbor site!!");
                return 0.0;
            }

            // KIRI_LOG_DEBUG("avg distance sum={0}", sum);

            return sum / num;
        }

        double computeMiniumPorosity()
        {
            return 0.0;
        }

        void computeWeightsError()
        {
            mCurGlobalWeightError = 0.0;
            auto site = mPowerDiagram->sites();
            mWeightError.assign(site.size(), 0.0);
            mWeightAbsError.assign(site.size(), 0.0);

            for (int i = 0; i < site.size(); i++)
            {
                if (site[i]->isBoundaryVertex())
                    continue;

                auto total = 0.0;
                auto cnt = 0;
                auto site_i = site[i];

                if (!site_i->neighbors().empty())
                {
                    auto radius_i = site_i->radius();

                    for (auto neighbor : site_i->neighbors())
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex3>(neighbor);
                        total += sn->weight() - sn->radius() * sn->radius();
                        cnt++;
                    }

                    auto avg = total / cnt;

                    for (auto neighbor : site_i->neighbors())
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex3>(neighbor);
                        auto distance = site[i]->distanceTo(sn);
                        auto minW = std::abs(std::sqrt(sn->weight()) - std::sqrt(site_i->weight()));
                        auto maxW = std::sqrt(sn->weight()) + std::sqrt(site_i->weight());
                        if (distance < maxW && distance > minW)
                        {
                            auto sum = sn->weight() - sn->radius() * sn->radius();
                            mWeightError[i] += avg - sum;
                            mWeightAbsError[i] += std::abs(avg - sum);
                            mCurGlobalWeightError += std::abs(avg - sum);
                        }
                        else
                        {

                            auto pw = distance * distance - site_i->weight();
                            mWeightError[i] += pw;
                            mWeightAbsError[i] += std::abs(pw);
                            mCurGlobalWeightError += std::abs(pw);
                        }
                    }
                }
            }
        }

        void adaptWeights()
        {

            auto g_area_error = globalAreaError();
            auto g_avg_distance = globalAvgDistance();

            KIRI_LOG_DEBUG("g_area_error={0}; g_avg_distance={1}; mCurGlobalWeightError={2}", g_area_error, g_avg_distance, mCurGlobalWeightError);

            auto gamma_area = 1e-2;
            auto gamma_bc = 1e-2;

            auto sites = mPowerDiagram->sites();
            for (int i = 0; i < sites.size(); i++)
            {

                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                auto weight = site_i->weight();

                auto weight_area = 0.0;
                auto weight_bc = 0.0;

                auto n = site_i->neighbors().size();
                if (n > 0)
                {
                    auto current_area = (!site_i->polygon()) ? 0.0 : site_i->polygon()->volume();
                    auto target_area = 4.0 / 3.0 * kiri_math_mini::pi<double>() * std::pow(site_i->radius(), 3.0);

                    auto area_percentage = 2.0;
                    if (current_area != 0.0)
                        area_percentage = target_area / current_area;

                    auto area_error_transform = (-(g_area_error - 1.0) * (g_area_error - 1.0) + 1.0);
                    auto area_step = g_avg_distance * area_error_transform * gamma_area;
                    if (area_percentage < (1.0 - std::numeric_limits<double>::epsilon()) && weight > 0.0)
                        weight_area -= area_step;
                    else if (area_percentage > (1.0 + std::numeric_limits<double>::epsilon()))
                        weight_area += area_step;
                }

                auto error = mWeightAbsError[i] / (mCurGlobalWeightError + std::numeric_limits<double>::epsilon());
                auto error_transform = (-(error - 1.0) * (error - 1.0) + 1.0);

                auto step = error_transform * gamma_bc;
                if (mWeightError[i] < 0.0)
                    weight_bc -= step;
                else if (mWeightError[i] > 0.0)
                    weight_bc += step;

                // KIRI_LOG_DEBUG("current site weight={0}; areaweight={1}, bcweight={2}", weight, weight_area, weight_bc);
                site_i->setWeight(weight + weight_area + weight_bc);
            }
        }

    private:
        int mSiteCounter = 0;
        int mCurIteration = 0;

        double mCompleteArea = 0.0;
        double mCurGlobalPorosity = 0.0;
        double mCurGlobalWeightError = 0.0;

        int mMaxiumNum = 1000;
        bool bReachMaximumNum = false;
        double mLastMP = 1.0;

        std::vector<double> mUsrDefinedRadiusDist;
        std::vector<double> mUsrDefinedRadiusDistProb;

        std::vector<double>
            mWeightError,
            mWeightAbsError,
            mGlobalErrorArray,
            mGlobalPorosityArray;

        Voronoi::PowerDiagram3Ptr mPowerDiagram;
        std::shared_ptr<KIRI::CudaShapeSampler> mCudaSDF;
    };

} // namespace HDV::Sampler

#endif /* _HDV_MS_SAMPLER3_H_ */