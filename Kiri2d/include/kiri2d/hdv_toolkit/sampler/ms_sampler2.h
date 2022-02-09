/***
 * @Author: Xu.WANG
 * @Date: 2022-01-10 22:51:36
 * @LastEditTime: 2022-01-10 23:03:16
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_MS_SAMPLER2_H_
#define _HDV_MS_SAMPLER2_H_

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

            //! TODO check site position whether is inside boundary or not
            mPowerDiagram->Compute();
        }

        bool CheckVoroCell()
        {
            std::vector<int> remove;

            auto sites = mPowerDiagram->GetSites();

            for (auto i = 0; i < sites.size(); i++)
            {

                auto siteI = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                if (siteI->GetIsBoundaryVertex())
                    continue;

                if (!siteI->CellPolygon)
                    remove.emplace_back(siteI->GetId());
                else
                {
                    auto centroid = siteI->CellPolygon->GetCentroid();
                    if (!mPowerDiagram->GetBoundary()->Contains(centroid))
                        remove.emplace_back(siteI->GetId());
                }
            }

            if (!remove.empty())
            {
                mPowerDiagram->RemoveVoroSitesByIndexArray(remove);
                return true;
            }

            return false;
        }

        void Compute()
        {
            mCurIteration++;

            auto needAddSites = this->DynamicAddSites();
            if (needAddSites)
            {
                mPowerDiagram->Compute();
                // KIRI_LOG_DEBUG("Iter={0} : Add Sites!!!!", mCurIteration);
            }
            else
            {
                mPowerDiagram->Move2Centroid();
                this->ComputeWeightsError();
                this->AdaptWeights();
                mPowerDiagram->Compute();
            }

            if (CheckVoroCell())
            {
                mPowerDiagram->Compute();
            }

            if (RemoveNoiseVoroSites())
            {
                mPowerDiagram->Compute();
            }

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

#pragma omp parallel for reduction(+ \
                                   : error)
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
                    // auto targetArea = siteI->GetRadius() * siteI->GetRadius() * kiri_math_mini::pi<double>();
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

#pragma omp parallel for reduction(+ \
                                   : sum, num)
            for (int i = 0; i < site.size(); i++)
            {
                if (site[i]->GetIsBoundaryVertex())
                    continue;

                if (!site[i]->mNeighborSites.empty())
                {
                    for (auto neighbor : site[i]->mNeighborSites)
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex2>(neighbor);
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
                    // remove.emplace_back(siteI->GetRadius());
                }
            }

            return circles;
        }

        double ComputeMiniumPorosity()
        {
            auto maxCircleArray = this->GetMICBySSkel();

            auto sum = 0.0;

#pragma omp parallel for reduction(+ \
                                   : sum)
            for (int i = 0; i < maxCircleArray.size(); i++)
            {
                auto circle = maxCircleArray[i];
                sum += circle.z * circle.z * kiri_math_mini::pi<double>();
            }

            auto boundary = mPowerDiagram->GetBoundary();

            // KIRI_LOG_DEBUG("boundary area ={0}, sum={1}, max circle size={2}", boundary->GetArea(), sum, maxCircleArray.size());
            return (boundary->GetArea() - sum) / boundary->GetArea();
        }

        bool RemoveNoiseVoroSites()
        {
            std::vector<int> removeVoroIdxs;
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

                    auto micI = poly->ComputeMICByStraightSkeleton();
                    for (auto neighbor : siteI->mNeighborSites)
                    {
                        auto siteJ = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(neighbor);
                        if (siteJ->GetIsBoundaryVertex())
                            continue;

                        auto polyJ = siteJ->CellPolygon;
                        if (polyJ)
                        {

                            if (polyJ->mSkeletons.empty())
                                polyJ->ComputeSSkel1998Convex();

                            auto micJ = polyJ->ComputeMICByStraightSkeleton();

                            auto disIJ = (Vector2F(micI.x, micI.y) - Vector2F(micJ.x, micJ.y)).length();
                            // if ((disIJ < ((micI.z + micJ.z) / 2.f)) &&
                            //     !std::binary_search(removeVoroIdxs.begin(), removeVoroIdxs.end(), siteI->GetId()) &&
                            //     !std::binary_search(removeVoroIdxs.begin(), removeVoroIdxs.end(), siteJ->GetId()))
                            //     removeVoroIdxs.emplace_back(siteJ->GetId());

                            if ((disIJ < ((micI.z + micJ.z) / 2.f)))
                                removeVoroIdxs.emplace_back(siteJ->GetId());
                        }
                        else
                        {
                            KIRI_LOG_ERROR("GetMICBySSkel: No Polygon Data!!!");
                            // remove.emplace_back(siteI->GetRadius());
                        }
                    }
                }
                else
                {
                    KIRI_LOG_ERROR("GetMICBySSkel: No Polygon Data!!!");
                    // remove.emplace_back(siteI->GetRadius());
                }
            }

            if (!removeVoroIdxs.empty())
            {
                KIRI_LOG_DEBUG("Remove overlapping cell, size={0}", removeVoroIdxs.size());
                mPowerDiagram->RemoveVoroSitesByIndexArray(removeVoroIdxs);
                return true;
            }

            return false;
        }

        bool DynamicAddSites()
        {
            if (mPowerDiagram->GetSites().size() - 4 >= mMaxiumNum && bReachMaxuimNum == false)
                bReachMaxuimNum = true;

            if (bReachMaxuimNum)
                return false;

            auto entityNum = 20;
            auto kThreshold = 200;

            std::random_device engine;
            std::mt19937 gen(engine());
            std::piecewise_constant_distribution<double> dist{std::begin(mUsrDefinedRadiusDist), std::end(mUsrDefinedRadiusDist), std::begin(mUsrDefinedRadiusDistProb)};

            if (mGlobalPorosityArray.size() > entityNum)
            {
                std::vector<Voronoi::VoronoiSite2Ptr> newVoroArrays;
                std::vector<double> errorArray(mGlobalPorosityArray.end() - entityNum, mGlobalPorosityArray.end());
                auto line = LineFitLeastSquares(errorArray);
                // KIRI_LOG_DEBUG("line k={0}", std::abs(line.x));
                if (std::abs(line.x) < 1e-6)
                {
                    // KIRI_LOG_DEBUG("reach line res={0}", std::abs(line.x) < 1e-6f);

                    if (!bReachMaxuimNum)
                    {
                        auto voroSite = mPowerDiagram->GetSites();
                        for (int i = 0; i < voroSite.size() - 4; i++)
                        {
                            auto pos = mPowerDiagram->GetBoundary()->GetRndInnerPoint();
                            auto nSite = std::make_shared<Voronoi::VoronoiSite2>(pos.x, pos.y, mSiteCounter++);

                            nSite->SetRadius(dist(gen));
                            newVoroArrays.emplace_back(nSite);
                            // KIRI_LOG_DEBUG("new site={0},{1};radius={2};id={3}", pos.x, pos.y, nSite->GetRadius(), mSiteCounter - 1);
                        }
                    }
                }

                auto new_vorosite_num = newVoroArrays.size();
                auto cur_vorosite_num = mPowerDiagram->GetSites().size() - 4;
                auto need_append_vorosite_num = new_vorosite_num;

                if (!bReachMaxuimNum)
                {
                    if ((new_vorosite_num + cur_vorosite_num) > mMaxiumNum)
                    {
                        bReachMaxuimNum = true;
                        need_append_vorosite_num = mMaxiumNum - cur_vorosite_num;
                    }

                    for (int i = 0; i < need_append_vorosite_num; i++)
                        mPowerDiagram->AddSite(newVoroArrays[i]);
                }

                if (!newVoroArrays.empty())
                    return true;
            }

            return false;
        }

        void ComputeWeightsError()
        {
            mCurGlobalWeightError = 0.0;
            auto site = mPowerDiagram->GetSites();
            mWeightError.assign(site.size(), 0.0);
            mWeightAbsError.assign(site.size(), 0.0);

#pragma omp parallel for
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
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex2>(neighbor);
                        total += sn->GetWeight() - sn->GetRadius() * sn->GetRadius();
                        cnt++;
                    }

                    auto avg = total / cnt;

                    for (auto neighbor : siteI->mNeighborSites)
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex2>(neighbor);
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

#pragma omp parallel for
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
                    // auto targetArea = kiri_math_mini::pi<double>() * siteI->GetRadius() * siteI->GetRadius();

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

        std::vector<double> mUsrDefinedRadiusDist;
        std::vector<double> mUsrDefinedRadiusDistProb;

        std::vector<double>
            mWeightError,
            mWeightAbsError,
            mGlobalErrorArray,
            mGlobalPorosityArray;

        Voronoi::PowerDiagram2Ptr mPowerDiagram;
    };

} // namespace HDV::Sampler

#endif /* _HDV_MS_SAMPLER2_H_ */