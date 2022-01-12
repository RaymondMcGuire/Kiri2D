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
        virtual ~MultiSizeSampler3D() noexcept {}

        std::vector<Primitives::Vertex3Ptr> GetSites() { return mPowerDiagram->GetSites(); }

        void AddSite(double x, double y, double z, double radius)
        {
            auto site = std::make_shared<Voronoi::VoronoiSite3>(x, y, z, mSiteCounter++);
            site->SetRadius(radius);
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
            Reset();

            ComputeBoundaryArea();

            mPowerDiagram->Init();

            CheckVoroCell();
        }

        std::vector<Vector4D> GetSampledSpheres()
        {
            std::vector<Vector4D> data;
            auto sites = mPowerDiagram->GetSites();
            for (int i = 0; i < sites.size(); i++)
            {
                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (siteI->GetIsBoundaryVertex())
                    continue;

                if (mCudaSDF->CheckPointsInside(make_float3(siteI->X(), siteI->Y(), siteI->Z())))
                {
                    if (siteI->Polygon)
                    {
                        auto radius = siteI->Polygon->ComputeMinDisInPoly(Vector3D(siteI->X(), siteI->Y(), siteI->Z()));
                        data.emplace_back(Vector4D(siteI->X(), siteI->Y(), siteI->Z(), radius));
                    }
                    else
                    {
                        KIRI_LOG_ERROR("no polygon");
                    }
                }
                else
                {
                    KIRI_LOG_ERROR("point not inside polygon");
                }
            }
            return data;
        }

        bool CheckVoroCell()
        {
            // std::vector<int> remove;

            auto sites = mPowerDiagram->GetSites();
            for (int i = 0; i < sites.size(); i++)
            {

                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                if (siteI->GetIsBoundaryVertex())
                    continue;

                if (mCudaSDF->CheckPointsInside(make_float3(siteI->X(), siteI->Y(), siteI->Z())))
                {
                    if (siteI->Polygon)
                    {
                    }
                    else
                    {
                        KIRI_LOG_ERROR("no polygon");
                    }
                }
                else
                {
                    KIRI_LOG_ERROR("point not inside polygon");
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
            mPowerDiagram->Compute();

            // auto needAddSites = this->DynamicAddSites();
            // if (needAddSites)
            // {
            //     mPowerDiagram->Compute();
            //     // KIRI_LOG_DEBUG("Iter={0} : Add Sites!!!!", mCurIteration);
            // }
            // else
            // {
            //     mPowerDiagram->Move2Centroid();
            //     this->ComputeWeightsError();
            //     this->AdaptWeights();
            //     mPowerDiagram->Compute();
            // }

            // if (CheckVoroCell())
            // {
            //     mPowerDiagram->Compute();
            // }

            // mCurGlobalPorosity = this->ComputeMiniumPorosity();
            // mGlobalPorosityArray.emplace_back(mCurGlobalPorosity);
            // mGlobalErrorArray.emplace_back(mCurGlobalWeightError);

            // KIRI_LOG_DEBUG("porosity={0}; error={1}", mCurGlobalPorosity, mCurGlobalWeightError);
        }

        void Reset()
        {
            mCurIteration = 0;
            mCompleteArea = 0.0;
            mCurGlobalPorosity = 0.0;
        }

        void ComputeBoundaryArea()
        {
        }

        double GetGlobalAreaError()
        {
            auto error = 0.0;
            /*       auto sites = mPowerDiagram->GetSites();

                   for (int i = 0; i < sites.size(); i++)
                   {
                       auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                       if (siteI->GetIsBoundaryVertex())
                           continue;

                       auto n = siteI->mNeighborSites.size();
                       if (n > 2)
                       {
                           auto currentArea = (!siteI->CellPolygon) ? 0.0 : siteI->CellPolygon->GetArea();
                           auto targetArea = n * siteI->GetRadius() * siteI->GetRadius() * std::tan(kiri_math_mini::pi<double>() / n);
                           error += std::abs(targetArea - currentArea) / (mCompleteArea * 2.0);
                       }
                   }*/
            return error;
        }

        double GetGlobalAvgDistance()
        {
            double sum = 0.0;
            int num = 0;
            auto site = mPowerDiagram->GetSites();

            for (int i = 0; i < site.size(); i++)
            {
                if (site[i]->GetIsBoundaryVertex())
                    continue;

                if (!site[i]->mNeighborSites.empty())
                {
                    for (auto neighbor : site[i]->mNeighborSites)
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex3>(neighbor);
                        auto distance = site[i]->Distance(sn);
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
                if (site[i]->GetIsBoundaryVertex())
                    continue;

                auto total = 0.0;
                auto cnt = 0;
                auto siteI = site[i];

                if (!siteI->mNeighborSites.empty())
                {
                    auto radiusI = siteI->GetRadius();

                    for (auto neighbor : siteI->mNeighborSites)
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex3>(neighbor);
                        total += sn->GetWeight() - sn->GetRadius() * sn->GetRadius();
                        cnt++;
                    }

                    auto avg = total / cnt;

                    for (auto neighbor : siteI->mNeighborSites)
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex3>(neighbor);
                        auto distance = siteI->Distance(sn);
                        auto minW = std::abs(std::sqrt(sn->GetWeight()) - std::sqrt(siteI->GetWeight()));
                        auto maxW = std::sqrt(sn->GetWeight()) + std::sqrt(siteI->GetWeight());
                        if (distance < maxW && distance > minW)
                        {
                            auto sum = sn->GetWeight() - sn->GetRadius() * sn->GetRadius();
                            mWeightError[i] += avg - sum;
                            mWeightAbsError[i] += std::abs(avg - sum);
                            mCurGlobalWeightError += std::abs(avg - sum);
                        }
                        else
                        {

                            auto pw = distance * distance - siteI->GetWeight();
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

            /* auto gAreaError = GetGlobalAreaError();
             auto gAvgDistance = GetGlobalAvgDistance();

             auto gammaArea = 1.0;
             auto gammaBC = 1.0;

             auto sites = mPowerDiagram->GetSites();
             for (int i = 0; i < sites.size(); i++)
             {

                 auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(sites[i]);
                 if (siteI->GetIsBoundaryVertex())
                     continue;

                 auto weight = siteI->GetWeight();

                 auto areaWeight = 0.0;
                 auto bcWeight = 0.0;

                 auto n = siteI->mNeighborSites.size();
                 if (n > 2)
                 {
                     auto currentArea = (!siteI->CellPolygon) ? 0.0 : siteI->CellPolygon->GetArea();
                     auto targetArea = n * siteI->GetRadius() * siteI->GetRadius() * std::tan(kiri_math_mini::pi<double>() / n);

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

                 siteI->SetWeight(weight + areaWeight + bcWeight);
             }*/
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