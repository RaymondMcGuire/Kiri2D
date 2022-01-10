/***
 * @Author: Xu.WANG
 * @Date: 2022-01-10 22:51:36
 * @LastEditTime: 2022-01-10 23:03:16
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_MS_SAMPLER_H_
#define _HDV_MS_SAMPLER_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/power_diagram.h>
#include <kiri2d/geo/convex_clip2.h>

namespace HDV::Sampler
{
    class MultiSizeSampler2D
    {
    public:
        explicit MultiSizeSampler2D() { mPowerDiagram = std::make_shared<Voronoi::PowerDiagram2D>(); }
        virtual ~MultiSizeSampler2D() noexcept {}

        std::vector<Primitives::Vertex2Ptr> GetSites() { return mPowerDiagram->GetSites(); }

        void AddSite(double x, double y, double radius)
        {
            auto site = std::make_shared<Voronoi::VoronoiSite2>(x, y, mSiteCounter++);
            site->SetRadius(radius);
            mPowerDiagram->AddSite(site);
        }

        void SetMaxiumNum(int num)
        {
            mMaxiumNum = num;
        }

        void SetBoundaryPolygon(const std::shared_ptr<Voronoi::VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> &boundary)
        {
            mPowerDiagram->SetBoundaryPolygon(boundary);
        }

        void Init()
        {
            Reset();

            ComputeBoundaryArea();

            //! TODO check site position whether is inside boundary or not
            mPowerDiagram->Compute();
        }

        void Compute()
        {
            mCurIteration++;
            this->DynamicAddSites();
            mPowerDiagram->Move2Centroid();
            this->ComputeWeightsError();
            this->AdaptWeights();
            mPowerDiagram->Compute();

            mCurGlobalPorosity = this->ComputeMiniumPorosity();
            mGlobalPorosityArray.emplace_back(mCurGlobalPorosity);
            mGlobalErrorArray.emplace_back(mCurGlobalWeightError);

            KIRI_LOG_DEBUG("porosity={0}; error={1}", mCurGlobalPorosity, mCurGlobalWeightError);
        }

        void Reset()
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
            mCompleteArea = mPowerDiagram->GetBoundary()->GetArea();
        }

        double GetGlobalAreaError()
        {
            auto error = 0.0;
            auto sites = mPowerDiagram->GetSites();

            for (int i = 0; i < sites.size(); i++)
            {
                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                if (siteI->GetIsBoundaryVertex())
                    continue;

                auto n = siteI->mNeighborSites.size();
                if (n > 2)
                {
                    auto currentArea = (!siteI->CellPolygon) ? 0.0 : siteI->CellPolygon->GetArea();
                    auto targetArea = n * siteI->GetRadius() * siteI->GetRadius() * std::tan(kiri_math_mini::pi<double>() / n);
                    error += std::abs(targetArea - currentArea) / (mCompleteArea * 2.0);
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
                if (site[i]->GetIsBoundaryVertex())
                    continue;

                if (!site[i]->mNeighborSites.empty())
                {
                    for (auto neighbor : site[i]->mNeighborSites)
                    {
                        auto sn = site[neighbor];
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

        Vector2D LineFitLeastSquares(std::vector<double> data)
        {
            double A = 0.0;
            double B = 0.0;
            double C = 0.0;
            double D = 0.0;

            for (int i = 0; i < data.size(); i++)
            {
                A += i * i;
                B += i;
                C += i * data[i];
                D += data[i];
            }

            double tmp = data.size() * A - B * B;

            double k = (data.size() * C - B * D) / (tmp + std::numeric_limits<double>::epsilon());
            double b = (A * D - B * C) / (tmp + std::numeric_limits<double>::epsilon());

            return Vector2D(k, b);
        }

        std::vector<Vector4D> GetMICBySSkel()
        {
            std::vector<Vector4D> circles;
            auto sites = mPowerDiagram->GetSites();

            for (int i = 0; i < sites.size(); i++)
            {
                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                if (siteI->GetIsBoundaryVertex())
                    continue;

                auto poly = siteI->CellPolygon;
                if (poly)
                {

                    if (poly->mSkeletons.empty())
                        poly->ComputeSSkel1998Convex();

                    auto mic = poly->ComputeMICByStraightSkeleton();
                    circles.emplace_back(Vector4D(mic, siteI->GetRadius()));
                    // KIRI_LOG_DEBUG("has poly: circle={0},{1},{2},{3}", mic.x, mic.y, mic.z, siteI->GetRadius());
                }
                else
                {
                    KIRI_LOG_ERROR("GetMICBySSkel: No Polygon Data!!!");
                }
            }

            return circles;
        }

        double ComputeMiniumPorosity()
        {
            auto maxCircleArray = this->GetMICBySSkel();

            auto sum = 0.0;
            for (int i = 0; i < maxCircleArray.size(); i++)
            {
                auto circle = maxCircleArray[i];
                sum += circle.z * circle.z * kiri_math_mini::pi<double>();
            }

            auto boundary = mPowerDiagram->GetBoundary();

            // KIRI_LOG_DEBUG("boundary area ={0}, sum={1}, max circle size={2}", boundary->GetArea(), sum, maxCircleArray.size());
            return (boundary->GetArea() - sum) / boundary->GetArea();
        }

        void DynamicAddSites()
        {
            if (mPowerDiagram->GetSites().size() - 4 >= mMaxiumNum && bReachMaxuimNum == false)
                bReachMaxuimNum = true;

            auto entityNum = 20;
            auto kThreshold = 200;

            if (mGlobalPorosityArray.size() > entityNum)
            {
                bool bAddVoroSite = false;
                std::vector<Voronoi::VoronoiSite2Ptr> newVoroArrays;
                std::vector<double> errorArray(mGlobalPorosityArray.end() - entityNum, mGlobalPorosityArray.end());
                auto line = LineFitLeastSquares(errorArray);
                // KIRI_LOG_DEBUG("line k={0}", std::abs(line.x));
                if (std::abs(line.x) < 1e-6)
                {
                    // KIRI_LOG_DEBUG("reach line res={0}", std::abs(line.x) < 1e-6f);
                    std::vector<double> radiusRange;
                    radiusRange.push_back(20.0);
                    radiusRange.push_back(30.0);
                    radiusRange.push_back(80.0);
                    radiusRange.push_back(150.0);

                    std::vector<double> radiusRangeProb;
                    radiusRangeProb.push_back(0.5);
                    radiusRangeProb.push_back(0.4);
                    radiusRangeProb.push_back(0.1);

                    std::random_device engine;
                    std::mt19937 gen(engine());
                    std::piecewise_constant_distribution<double> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

                    if (bReachMaxuimNum)
                    {
                        auto pos = mPowerDiagram->GetBoundary()->GetRndInnerPoint();
                        auto site = std::make_shared<Voronoi::VoronoiSite2>(pos.x, pos.y, mSiteCounter++);
                        site->SetRadius(pcdis(gen));
                        newVoroArrays.emplace_back(site);
                    }
                    else
                    {
                        auto voroSite = mPowerDiagram->GetSites();
                        for (int i = 0; i < voroSite.size() - 4; i++)
                        {
                            auto pos = mPowerDiagram->GetBoundary()->GetRndInnerPoint();
                            auto nSite = std::make_shared<Voronoi::VoronoiSite2>(pos.x, pos.y, mSiteCounter++);

                            nSite->SetRadius(pcdis(gen));
                            newVoroArrays.emplace_back(nSite);
                        }
                    }
                }

                auto new_vorosite_num = newVoroArrays.size();
                auto cur_vorosite_num = mPowerDiagram->GetSites().size() - 4;
                auto need_append_vorosite_num = new_vorosite_num;

                if (!bReachMaxuimNum)
                {
                    if ((new_vorosite_num + cur_vorosite_num) > mMaxiumNum)
                        need_append_vorosite_num = mMaxiumNum - cur_vorosite_num;

                    for (int i = 0; i < need_append_vorosite_num; i++)
                        mPowerDiagram->AddSite(newVoroArrays[i]);
                }
                else
                {
                    auto current_mp = ComputeMiniumPorosity();
                    if (mLastMP > current_mp)
                    {
                        // KIRI_LOG_DEBUG("Add P");
                        for (int i = 0; i < need_append_vorosite_num; i++)
                            mPowerDiagram->AddSite(newVoroArrays[i]);

                        mLastMP = current_mp;
                    }
                }

                // if (bAddVoroSite)
                //     mPowerDiagram->ResetVoroSitesWeight();
            }
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
                        auto sn = site[neighbor];
                        total += sn->GetWeight() - sn->GetRadius() * sn->GetRadius();
                        cnt++;
                    }

                    auto avg = total / cnt;

                    for (auto neighbor : siteI->mNeighborSites)
                    {
                        auto sn = site[neighbor];
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

            auto gAreaError = GetGlobalAreaError();
            auto gAvgDistance = GetGlobalAvgDistance();

            auto gammaArea = 1.0;
            auto gammaBC = 1.0;

            auto sites = mPowerDiagram->GetSites();
            for (int i = 0; i < sites.size(); i++)
            {

                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
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

        std::vector<double>
            mWeightError,
            mWeightAbsError,
            mGlobalErrorArray,
            mGlobalPorosityArray;

        Voronoi::PowerDiagram2Ptr mPowerDiagram;
    };

} // namespace HDV::Sampler

#endif /* _HDV_MS_SAMPLER_H_ */