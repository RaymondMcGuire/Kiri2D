/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-10-20 00:47:43
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_core.cpp
 */

#include <kiri2d/voronoi/voro_ns_optimize.h>
#include <random>
#include <omp.h>

namespace KIRI
{
    void KiriVoroNSOptimize::Init()
    {
        Reset();

        ComputeBoundaryPolygonArea();

        CorrectVoroSitePos();

        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroNSOptimize::Reset()
    {
        mCurGlobalPorosity = 0.f;
        mCurIteration = 0;
        mCompleteArea = 0.f;
        mVoroSitesWeightError.clear();
        mVoroSitesWeightAbsError.clear();
        mGlobalErrorArray.clear();
        mGlobalPorosityArray.clear();
    }

    void KiriVoroNSOptimize::CorrectVoroSitePos()
    {
        if (mPowerDiagram->GetBoundaryPolygon2() == NULL)
        {
            KIRI_LOG_ERROR("CorrectVoroSitePos::Please set boundary polygon!!");
            return;
        }

        auto voroSite = mPowerDiagram->GetVoroSites();
        if (!voroSite.empty())
        {
            auto boundary = mPowerDiagram->GetBoundaryPolygon2();
#pragma omp parallel for
            for (int i = 0; i < voroSite.size(); i++)
            {
                auto pos = voroSite[i]->GetValue();
                if (!boundary->Contains(Vector2F(pos.x, pos.y)))
                    voroSite[i]->ResetValue(boundary->GetRndInnerPoint());
            }
        }
        else
            KIRI_LOG_ERROR("CorrectVoroSitePos::No voro site!!");
    }

    void KiriVoroNSOptimize::ComputeBoundaryPolygonArea()
    {
        if (mPowerDiagram->GetBoundaryPolygon2() == NULL)
        {
            KIRI_LOG_ERROR("GetBoundaryPolygonArea::Please set boundary polygon!!");
            mCompleteArea = 0.f;
            return;
        }

        mCompleteArea = mPowerDiagram->GetBoundaryPolygon2()->GetPolygonArea();
    }

    void KiriVoroNSOptimize::SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary)
    {
        mCompleteArea = boundary->GetPolygonArea();
        mPowerDiagram->SetBoundaryPolygon2(boundary);
    }

    void KiriVoroNSOptimize::CorrectWeights()
    {
        auto voroSite = mPowerDiagram->GetVoroSites();
#pragma omp parallel for
        for (int i = 0; i < voroSite.size(); i++)
        {
            for (size_t j = 0; j < voroSite.size(); j++)
            {
                if (voroSite[i]->GetIdx() != voroSite[j]->GetIdx())
                {
                    auto distance = voroSite[i]->GetDistance2(voroSite[j]);
                    if (std::sqrt(voroSite[i]->GetWeight()) >= distance)
                        voroSite[j]->SetWeight(distance * distance);
                }
            }
        }
    }

    void KiriVoroNSOptimize::AdaptPositionsWeights()
    {
        auto outside = mPowerDiagram->Move2CentroidDisableSite();
        // if (outside)
        //     CorrectWeights();
    }

    float KiriVoroNSOptimize::GetGlobalAreaError()
    {
        auto error = 0.f;
        auto voroSite = mPowerDiagram->GetVoroSites();

#pragma omp parallel for reduction(+ \
                                   : error)
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto n = voroSite[i]->GetNeighborSites().size();
            if (n > 2)
            {
                auto currentArea = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
                auto targetArea = n * voroSite[i]->GetRadius() * voroSite[i]->GetRadius() * std::tanf(KIRI_PI<float>() / n);
                error += std::abs(targetArea - currentArea) / (mCompleteArea * 2.f);
            }
        }
        return error;
    }

    void KiriVoroNSOptimize::RemoveNoiseVoroSites()
    {
        Vector<UInt> removeVoroIdxs;
        auto voroSite = mPowerDiagram->GetVoroSites();
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto siteI = voroSite[i];
            auto data = siteI->GetValue();

            auto polyI = siteI->GetCellPolygon();
            if (polyI != NULL)
            {
                // check boundary
                auto boundary_poly = mPowerDiagram->GetBoundaryPolygon2();
                auto boundary_contain = boundary_poly->Contains(polyI->GetPolygonCentroid());
                if (!boundary_contain)
                {
                    removeVoroIdxs.emplace_back(siteI->GetIdx());
                    continue;
                }

                if (polyI->GetSkeletons().empty())
                    polyI->ComputeSSkel1998Convex();

                auto micI = polyI->ComputeMICByStraightSkeleton();

                auto neighbors = siteI->GetNeighborSites();
                for (size_t j = 0; j < neighbors.size(); j++)
                {
                    auto siteJ = neighbors[j];
                    if (siteJ->GetIdx() == siteI->GetIdx())
                        continue;

                    auto polyJ = siteJ->GetCellPolygon();
                    if (polyJ != NULL)
                    {
                        if (polyJ->GetSkeletons().empty())
                            polyJ->ComputeSSkel1998Convex();

                        auto micJ = polyJ->ComputeMICByStraightSkeleton();
                        auto disIJ = (Vector2F(micI.x, micI.y) - Vector2F(micJ.x, micJ.y)).length();
                        if ((disIJ < ((micI.z + micJ.z) / 2.f)) &&
                            !std::binary_search(removeVoroIdxs.begin(), removeVoroIdxs.end(), siteI->GetIdx()) &&
                            !std::binary_search(removeVoroIdxs.begin(), removeVoroIdxs.end(), siteJ->GetIdx()))
                            removeVoroIdxs.emplace_back(siteJ->GetIdx());
                    }
                }
            }
        }

        if (!removeVoroIdxs.empty())
        {
            // KIRI_LOG_DEBUG("Remove overlapping cell, size={0}", removeVoroIdxs.size());
            mPowerDiagram->RemoveVoroSitesByIndexArray(removeVoroIdxs);
            // AdaptPositionsWeights();
            mPowerDiagram->ComputeDiagram();
        }
    }

    void KiriVoroNSOptimize::AdaptWeights()
    {
        ComputeVoroSiteWeightError();

        auto gAreaError = GetGlobalAreaError();
        auto gAvgDistance = GetGlobalAvgDistance();

        auto gammaArea = 1.f;
        auto gammaBC = 100.f;

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto weight = voroSite[i]->GetWeight();

            auto areaWeight = 0.f;
            auto bcWeight = 0.f;

            auto n = voroSite[i]->GetNeighborSites().size();
            if (n > 2)
            {
                auto currentArea = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
                auto targetArea = n * voroSite[i]->GetRadius() * voroSite[i]->GetRadius() * std::tanf(KIRI_PI<float>() / n);

                auto pArea = 2.f;
                if (currentArea != 0.f)
                    pArea = targetArea / currentArea;

                auto areaErrorTransform = (-(gAreaError - 1.f) * (gAreaError - 1.f) + 1.f);
                auto areaStep = gAvgDistance * areaErrorTransform * gammaArea;
                if (pArea < (1.f - MEpsilon<float>()) && weight > 0.f)
                    areaWeight -= areaStep;
                else if (pArea > (1.f + MEpsilon<float>()))
                    areaWeight += areaStep;
            }

            auto error = mVoroSitesWeightAbsError[i] / (mCurGlobalWeightError + MEpsilon<float>());
            auto errorTransform = (-(error - 1.f) * (error - 1.f) + 1.f);

            auto step = errorTransform * gammaBC;
            if (mVoroSitesWeightError[i] < 0.f)
                bcWeight -= step;
            else if (mVoroSitesWeightError[i] > 0.f)
                bcWeight += step;

            // KIRI_LOG_DEBUG("AdaptWeights: error={0}, step={1}, weight={2}", error, step, weight);
            //  // KIRI_LOG_DEBUG("AdaptWeights: mVoroSitesDisError={0}, mVoroSitesDisErrorAbs={1}, mCurGlobalDisError={2}", mVoroSitesDisError[i], mVoroSitesDisErrorAbs[i], mCurGlobalDisError);

            // KIRI_LOG_DEBUG("AdaptWeights: idx={0}, aw={1}, bw={2}", i, areaWeight, bcWeight);

            voroSite[i]->SetWeight(weight + areaWeight + bcWeight);
        }

        // KIRI_LOG_DEBUG("AdaptWeights: mCurGlobalWeightError={0}", mCurGlobalWeightError);
    }

    void KiriVoroNSOptimize::ComputeVoroSiteWeightError()
    {
        mCurGlobalWeightError = 0.f;
        auto voroSite = mPowerDiagram->GetVoroSites();
        mVoroSitesWeightError.assign(voroSite.size(), 0.f);
        mVoroSitesWeightAbsError.assign(voroSite.size(), 0.f);

        for (int i = 0; i < voroSite.size(); i++)
        {
            auto total = 0.f;
            auto cnt = 0;
            auto siteI = voroSite[i];

            if (!siteI->GetNeighborSites().empty())
            {

                auto radiusI = siteI->GetRadius();
                for (size_t j = 0; j < siteI->GetNeighborSites().size(); j++)
                {
                    auto siteJ = siteI->GetNeighborSites()[j];
                    total += siteJ->GetWeight() - siteJ->GetRadius() * siteJ->GetRadius();
                    cnt++;
                }

                auto avg = total / cnt;

                for (size_t j = 0; j < siteI->GetNeighborSites().size(); j++)
                {
                    auto siteJ = siteI->GetNeighborSites()[j];

                    auto distance = siteI->GetDistance2(siteJ);
                    auto minW = std::abs(std::sqrt(siteJ->GetWeight()) - std::sqrt(siteI->GetWeight()));
                    auto maxW = std::sqrt(siteJ->GetWeight()) + std::sqrt(siteI->GetWeight());
                    if (distance < maxW && distance > minW)
                    {
                        auto sum = siteJ->GetWeight() - siteJ->GetRadius() * siteJ->GetRadius();
                        mVoroSitesWeightError[i] += 1.f / (avg - sum + 1e-9f);
                        mVoroSitesWeightAbsError[i] += 1.f / std::abs(avg - sum + 1e-9f);
                        mCurGlobalWeightError += 1.f / std::abs(avg - sum + 1e-9f);
                    }
                    else
                    {

                        auto pw = distance * distance - siteI->GetWeight();
                        mVoroSitesWeightError[i] += pw;
                        mVoroSitesWeightAbsError[i] += std::abs(pw);
                        mCurGlobalWeightError += std::abs(pw);
                    }
                }
            }
        }
    }

    float KiriVoroNSOptimize::GetGlobalAvgDistance()
    {
        float sum = 0.f;
        UInt num = 0;
        auto voroSite = mPowerDiagram->GetVoroSites();

        for (int i = 0; i < voroSite.size(); i++)
        {
            if (!voroSite[i]->GetNeighborSites().empty())
            {
                for (size_t j = 0; j < voroSite[i]->GetNeighborSites().size(); j++)
                {
                    auto distance = voroSite[i]->GetDistance2(voroSite[i]->GetNeighborSites()[j]);
                    sum += distance;
                    num++;
                }
            }
        }

        if (num == 0)
        {
            KIRI_LOG_ERROR("GetGlobalAvgDistance:: no neighbor site!!");
            return 0.f;
        }

        return sum / num;
    }

    void KiriVoroNSOptimize::SplitEventProcess()
    {
        auto coef = 0.5f;
        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        Vector<KiriVoroGroupSitePtr> new_sites;
        Vector<UInt> removeVoroIdxs;

        auto voroSite = mPowerDiagram->GetVoroSites();

        // record avg radius
        auto total_rad = 0.f;
        for (int i = 0; i < voroSite.size(); i++)
        {
            total_rad += voroSite[i]->GetRadius();
            // KIRI_LOG_DEBUG(" rad={0}", voroSite[i]->GetRadius());
        }
        total_rad /= voroSite.size();

        // check if need split
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto total_area = 0.f;
            auto siteI = std::dynamic_pointer_cast<KiriVoroGroupSite>(voroSite[i]);
            auto polyi = siteI->GetCellPolygon();
            if (polyi == nullptr)
                continue;

            if (!siteI->GetNeighborSites().empty())
            {
                for (size_t j = 0; j < siteI->GetNeighborSites().size(); j++)
                {
                    auto siteJ = siteI->GetNeighborSites()[j];
                    auto polyj = siteJ->GetCellPolygon();
                    total_area += polyj != nullptr ? polyj->GetPolygonArea() : 0.f;
                }
            }

            // judgement 1
            // if (polyi->GetPolygonArea() < total_area * coef)
            //     continue;

            if (siteI->GetRadius() < total_rad * 2.5f)
                continue;

            // split event
            auto centroid = polyi->GetPolygonCentroid();

            // rnd direction
            auto rnd_norm_dir = Vector2F(dist(rndEngine), dist(rndEngine)).normalized();

            auto new_vs1 = centroid + rnd_norm_dir;
            auto new_vs2 = centroid - rnd_norm_dir;

            // KIRI_LOG_DEBUG("cen={0},{1}, new1={2},{3}, new2={4},{5}", centroid.x, centroid.y, new_vs1.x, new_vs1.y, new_vs2.x, new_vs2.y);

            if (!polyi->Contains(new_vs1) || !polyi->Contains(new_vs2))
            {
                KIRI_LOG_ERROR("New Voronoi Site Pos is ERROR!!!");
                continue;
            }

            auto nvs1 = std::make_shared<KiriVoroGroupSite>(new_vs1);
            nvs1->SetWeight(siteI->GetWeight() / 1.2f);
            nvs1->SetRadius(siteI->GetRadius() / std::sqrtf(2.f));
            nvs1->SetGroupColor(siteI->GetGroupColor());
            nvs1->SetGroupId(siteI->GetGroupId());
            auto nvs2 = std::make_shared<KiriVoroGroupSite>(new_vs2);
            nvs2->SetWeight(siteI->GetWeight() / 1.2f);
            nvs2->SetRadius(siteI->GetRadius() / std::sqrtf(2.f));
            nvs2->SetGroupColor(siteI->GetGroupColor());
            nvs2->SetGroupId(siteI->GetGroupId());
            new_sites.emplace_back(nvs1);
            new_sites.emplace_back(nvs2);

            removeVoroIdxs.emplace_back(siteI->GetIdx());
        }

        if (!removeVoroIdxs.empty())
        {
            mPowerDiagram->RemoveVoroSitesByIndexArray(removeVoroIdxs);
            KIRI_LOG_DEBUG("remove sites num={0}, total num={1}", removeVoroIdxs.size(), mPowerDiagram->GetVoroSites().size());

            for (size_t idx = 0; idx < new_sites.size(); idx++)
                mPowerDiagram->AddVoroSite(new_sites[idx]);

            KIRI_LOG_DEBUG("add new sites num={0}, total num={1}", new_sites.size(), mPowerDiagram->GetVoroSites().size());

            mPowerDiagram->ComputeDiagram();
        }
    }

    void KiriVoroNSOptimize::Iterate()
    {
        mCurIteration++;
        SplitEventProcess();
        AdaptPositionsWeights();
        AdaptWeights();

        mPowerDiagram->ComputeDiagram();
        RemoveNoiseVoroSites();
    }

    float KiriVoroNSOptimize::ComputeIterate()
    {
        Iterate();
        mGlobalErrorArray.emplace_back(mCurGlobalWeightError);
        return mCurGlobalWeightError;
    }

    void KiriVoroNSOptimize::ComputeLloyd(UInt num)
    {
        mPowerDiagram->SetRelaxIterNumber(num);
        mPowerDiagram->LloydRelaxation();
    }
}