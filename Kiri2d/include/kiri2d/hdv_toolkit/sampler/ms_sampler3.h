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
    class MultiSizeSampler3D
    {
    public:
        explicit MultiSizeSampler3D(const std::shared_ptr<KIRI::CudaShapeSampler> &sdf)
            : mCudaSDF(std::move(sdf))
        {
            mPowerDiagram = std::make_shared<Voronoi::PowerDiagram3D>();
        }
        virtual ~MultiSizeSampler3D() {}

        std::vector<Primitives::Vertex3Ptr> GetSites() { return mPowerDiagram->GetSites(); }

        void AddSite(double x, double y, double z, double radius)
        {
            auto site = std::make_shared<Voronoi::VoronoiSite3>(x, y, z, mSiteCounter++);
            site->setRadius(radius);
            mPowerDiagram->AddSite(site);
        }

        void SetMaxiumNum(int num)
        {
            mMaxiumNum = num;
        }

        void SetBoundaryPolygon(std::vector<csgjscpp::Polygon> boundary)
        {
            mPowerDiagram->SetBoundaryPolygon(boundary);
        }

        void SetRadiusDist(std::vector<double> dist)
        {
            mUsrDefinedRadiusDist = dist;
        }

        void SetRadiusDistProb(std::vector<double> prob)
        {
            mUsrDefinedRadiusDistProb = prob;
        }

        void Init()
        {
            reset();

            ComputeBoundaryArea();

            mPowerDiagram->Init();
        }

        std::vector<Vector4D> GetSampledSpheres()
        {
            std::vector<Vector4D> data;
            auto sites = mPowerDiagram->GetSites();
            for (int i = 0; i < sites.size(); i++)
            {
                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (siteI->isBoundaryVertex())
                    continue;

                if (siteI->Polygon)
                {
                    auto cen = siteI->Polygon->GetCentroid();
                    if (mCudaSDF->CheckPointsInside(make_float3(cen.x, cen.y, cen.z)))
                    {
                        auto radius = siteI->Polygon->ComputeMinDisInPoly(cen);
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

                // if (mCudaSDF->CheckPointsInside(make_float3(siteI->X(), siteI->Y(), siteI->Z())))
                // {
                //     if (siteI->Polygon)
                //     {
                //         auto radius = siteI->Polygon->ComputeMinDisInPoly(Vector3D(siteI->X(), siteI->Y(), siteI->Z()));
                //         data.emplace_back(Vector4D(siteI->X(), siteI->Y(), siteI->Z(), radius));
                //     }
                //     else
                //     {
                //         KIRI_LOG_ERROR("no polygon");
                //     }
                // }
                // else
                // {
                //     KIRI_LOG_ERROR("point not inside polygon");
                // }
            }
            return data;
        }

        bool CheckVoroCell()
        {
            std::vector<int> remove;

            auto sites = mPowerDiagram->GetSites();
            for (int i = 0; i < sites.size(); i++)
            {

                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (siteI->isBoundaryVertex())
                    continue;

                if (mCudaSDF->CheckPointsInside(make_float3(siteI->X(), siteI->Y(), siteI->Z())))
                {
                    if (!siteI->Polygon)
                    {
                        remove.emplace_back(siteI->id());
                        auto points = mCudaSDF->GetInsidePoints(1);
                        // KIRI_LOG_INFO("change site pos={0},{1},{2}", points[0].x, points[0].y, points[0].z);
                        siteI->Set(points[0].x, points[0].y, points[0].z);
                    }
                    else
                    {
                        auto centroid = siteI->Polygon->GetCentroid();
                        if (!mCudaSDF->CheckPointsInside(make_float3(centroid.x, centroid.y, centroid.z)))
                        {
                            remove.emplace_back(siteI->id());
                            auto points = mCudaSDF->GetInsidePoints(1);
                            // KIRI_LOG_INFO("change site pos={0},{1},{2}", points[0].x, points[0].y, points[0].z);
                            siteI->Set(points[0].x, points[0].y, points[0].z);
                        }
                    }
                }
            }

            // if (!remove.empty())
            // {
            //     mPowerDiagram->RemoveVoroSitesByIndexArray(remove);
            //     return true;
            // }

            return false;
        }

        void Compute()
        {
            mCurIteration++;

            mPowerDiagram->Move2Centroid();
            this->ComputeWeightsError();
            this->AdaptWeights();

            // auto sites = mPowerDiagram->GetSites();
            // for (int i = 0; i < sites.size(); i++)
            // {

            //     auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
            //     if (siteI->isBoundaryVertex())
            //         continue;

            //     KIRI_LOG_DEBUG("id={0}:({1},{2},{3}); weight={4}", siteI->id(), siteI->X(), siteI->Y(), siteI->Z(), siteI->weight());
            // }

            mPowerDiagram->Compute();

            if (CheckVoroCell())
            {
                mPowerDiagram->Compute();
            }

            // mCurGlobalPorosity = this->ComputeMiniumPorosity();
            // mGlobalPorosityArray.emplace_back(mCurGlobalPorosity);
            // mGlobalErrorArray.emplace_back(mCurGlobalWeightError);
            mPowerDiagram->ExportObj();
            KIRI_LOG_DEBUG("error={0}", mCurGlobalWeightError);
        }

        void reset()
        {
            mCurIteration = 0;
            mCompleteArea = 0.0;
            mCurGlobalPorosity = 0.0;
        }

        void ComputeBoundaryArea()
        {
            auto boundary = mPowerDiagram->GetBoundary();
            if (!boundary)
            {
                KIRI_LOG_ERROR("Not Set Boundary!");
                return;
            }
            mCompleteArea = mPowerDiagram->GetBoundary()->GetVolume();
            KIRI_LOG_DEBUG("Boundary Volume={0}", mCompleteArea);
        }

        double GetGlobalAreaError()
        {
            auto error = 0.0;
            auto sites = mPowerDiagram->GetSites();

            for (int i = 0; i < sites.size(); i++)
            {
                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (siteI->isBoundaryVertex())
                    continue;

                auto n = siteI->neighbors().size();
                if (n > 0)
                {
                    auto currentArea = (!siteI->Polygon) ? 0.0 : siteI->Polygon->GetVolume();
                    auto targetArea = 4.0 / 3.0 * kiri_math_mini::pi<double>() * std::pow(siteI->radius(), 3.0);
                    error += std::abs(targetArea - currentArea) / (mCompleteArea * 2.0);
                    // KIRI_LOG_DEBUG("currentArea={0};targetArea={1},error={2}", currentArea, targetArea, error);
                }
                else
                {
                    KIRI_LOG_ERROR("No Neighbor Sites");
                }
            }
            return error;
        }

        double GetGlobalAvgDistance()
        {
            double sum = 0.0;
            int num = 0;
            auto site = mPowerDiagram->GetSites();

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
                KIRI_LOG_ERROR("GetGlobalAvgDistance:: no neighbor site!!");
                return 0.0;
            }

            // KIRI_LOG_DEBUG("avg distance sum={0}", sum);

            return sum / num;
        }

        double ComputeMiniumPorosity()
        {
            return 0.0;
        }

        void ComputeWeightsError()
        {
            mCurGlobalWeightError = 0.0;
            auto site = mPowerDiagram->GetSites();
            mWeightError.assign(site.size(), 0.0);
            mWeightAbsError.assign(site.size(), 0.0);

            for (int i = 0; i < site.size(); i++)
            {
                if (site[i]->isBoundaryVertex())
                    continue;

                auto total = 0.0;
                auto cnt = 0;
                auto siteI = site[i];

                if (!siteI->neighbors().empty())
                {
                    auto radiusI = siteI->radius();

                    for (auto neighbor : siteI->neighbors())
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex3>(neighbor);
                        total += sn->weight() - sn->radius() * sn->radius();
                        cnt++;
                    }

                    auto avg = total / cnt;

                    for (auto neighbor : siteI->neighbors())
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex3>(neighbor);
                        auto distance = site[i]->distanceTo(sn);
                        auto minW = std::abs(std::sqrt(sn->weight()) - std::sqrt(siteI->weight()));
                        auto maxW = std::sqrt(sn->weight()) + std::sqrt(siteI->weight());
                        if (distance < maxW && distance > minW)
                        {
                            auto sum = sn->weight() - sn->radius() * sn->radius();
                            mWeightError[i] += avg - sum;
                            mWeightAbsError[i] += std::abs(avg - sum);
                            mCurGlobalWeightError += std::abs(avg - sum);
                        }
                        else
                        {

                            auto pw = distance * distance - siteI->weight();
                            mWeightError[i] += pw;
                            mWeightAbsError[i] += std::abs(pw);
                            mCurGlobalWeightError += std::abs(pw);
                        }
                    }
                }
            }
        }

        void AdaptWeights()
        {

            auto gAreaError = GetGlobalAreaError();
            auto gAvgDistance = GetGlobalAvgDistance();

            KIRI_LOG_DEBUG("gAreaError={0}; gAvgDistance={1}; mCurGlobalWeightError={2}", gAreaError, gAvgDistance, mCurGlobalWeightError);

            auto gammaArea = 1e-2;
            auto gammaBC = 1e-2;

            auto sites = mPowerDiagram->GetSites();
            for (int i = 0; i < sites.size(); i++)
            {

                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (siteI->isBoundaryVertex())
                    continue;

                auto weight = siteI->weight();

                auto areaWeight = 0.0;
                auto bcWeight = 0.0;

                auto n = siteI->neighbors().size();
                if (n > 0)
                {
                    auto currentArea = (!siteI->Polygon) ? 0.0 : siteI->Polygon->GetVolume();
                    auto targetArea = 4.0 / 3.0 * kiri_math_mini::pi<double>() * std::pow(siteI->radius(), 3.0);

                    auto pArea = 2.0;
                    if (currentArea != 0.0)
                        pArea = targetArea / currentArea;

                    auto areaErrorTransform = (-(gAreaError - 1.0) * (gAreaError - 1.0) + 1.0);
                    auto areaStep = gAvgDistance * areaErrorTransform * gammaArea;
                    if (pArea < (1.0 - std::numeric_limits<double>::epsilon()) && weight > 0.0)
                        areaWeight -= areaStep;
                    else if (pArea > (1.0 + std::numeric_limits<double>::epsilon()))
                        areaWeight += areaStep;
                }

                auto error = mWeightAbsError[i] / (mCurGlobalWeightError + std::numeric_limits<double>::epsilon());
                auto errorTransform = (-(error - 1.0) * (error - 1.0) + 1.0);

                auto step = errorTransform * gammaBC;
                if (mWeightError[i] < 0.0)
                    bcWeight -= step;
                else if (mWeightError[i] > 0.0)
                    bcWeight += step;

                // KIRI_LOG_DEBUG("current site weight={0}; areaweight={1}, bcweight={2}", weight, areaWeight, bcWeight);
                siteI->setWeight(weight + areaWeight + bcWeight);
            }
        }

    private:
        int mSiteCounter = 0;
        int mCurIteration = 0;

        double mCompleteArea = 0.0;
        double mCurGlobalPorosity = 0.0;
        double mCurGlobalWeightError = 0.0;

        int mMaxiumNum = 1000;
        bool bReachMaxuimNum = false;
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