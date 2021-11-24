/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-10-20 00:47:43
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_core.cpp
 */

#include <kiri2d/voronoi/voro_ns_optimize.h>
#include <kiri2d/poly/PolygonClipping.h>
#include <random>
#include <omp.h>
#include <kiri2d/bop12/booleanop.h>

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

        mCurGlobalPosConstainWeightError = 0.f;
        mVoroPosConstainWeightError.clear();
        mVoroPosConstainWeightAbsError.clear();
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
                if (voroSite[i]->GetIsFrozen())
                    continue;

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

    void KiriVoroNSOptimize::AdaptPositionsWeights()
    {
        auto outside = mPowerDiagram->Move2CentroidDisableSite();
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
            AdaptPositionsWeights();
            // mPowerDiagram->ComputeDiagram();
        }
    }

    void KiriVoroNSOptimize::AdaptWeights()
    {
        ComputeVoroSiteWeightError();
        ComputeVoroSitePosConstrains();

        auto gAreaError = GetGlobalAreaError();
        auto gAvgDistance = GetGlobalAvgDistance();

        auto gammaArea = 1.f;
        auto gammaBC = 1.f;
        auto gammaPS = 1000.f;

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (int i = 0; i < voroSite.size(); i++)
        {
            if (voroSite[i]->GetIsFrozen())
                continue;

            auto weight = voroSite[i]->GetWeight();

            auto areaWeight = 0.f;
            auto bcWeight = 0.f;
            auto psWeight = 0.f;

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

            auto poserror = mVoroSitesWeightAbsError[i] / (mCurGlobalWeightError + MEpsilon<float>());
            auto poserrorTransform = (-(error - 1.f) * (error - 1.f) + 1.f);

            auto posstep = poserrorTransform * gammaPS;
            if (mVoroPosConstainWeightError[i] < 0.f)
                psWeight -= posstep;
            else if (mVoroPosConstainWeightError[i] > 0.f)
                psWeight += posstep;

            // KIRI_LOG_DEBUG("AdaptWeights: error={0}, step={1}, weight={2}", error, step, weight);
            //  // KIRI_LOG_DEBUG("AdaptWeights: mVoroSitesDisError={0}, mVoroSitesDisErrorAbs={1}, mCurGlobalDisError={2}", mVoroSitesDisError[i], mVoroSitesDisErrorAbs[i], mCurGlobalDisError);

            // KIRI_LOG_DEBUG("AdaptWeights: idx={0}, aw={1}, bw={2}", i, areaWeight, bcWeight);

            voroSite[i]->SetWeight(weight + areaWeight + bcWeight + psWeight);
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

            if (siteI->GetIsFrozen())
                continue;

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
                        mVoroSitesWeightError[i] += avg - sum;
                        mVoroSitesWeightAbsError[i] += std::abs(avg - sum);
                        mCurGlobalWeightError += std::abs(avg - sum);
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

    Vector<KiriVoroCellPolygon2Ptr> KiriVoroNSOptimize::GetFrozenPolygon()
    {
        mUnionPolygonArray.clear();
        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto siteI = voroSite[i];
            if (!siteI->GetIsFrozen())
                continue;

            for (size_t j = 0; j < voroSite.size(); j++)
            {
                auto siteJ = voroSite[j];
                if (!siteJ->GetIsFrozen() || siteI->GetIdx() == siteJ->GetIdx())
                    continue;

                auto gsite_i = std::dynamic_pointer_cast<KiriVoroGroupSite>(siteI);
                auto gsite_j = std::dynamic_pointer_cast<KiriVoroGroupSite>(siteJ);

                if (gsite_i->GetGroupId() != gsite_j->GetGroupId())
                    continue;

                if ((!gsite_i->GetCellPolygon()->GetBBox().overlaps(gsite_j->GetCellPolygon()->GetBBox())) || (gsite_i->GetCellPolygon()->GetBBox().contains(gsite_j->GetCellPolygon()->GetBBox())))
                {

                    gsite_i->SetFreeze(false);
                    gsite_j->SetFreeze(false);

                    gsite_j->SetGroupColor(Vector3F(dist(rndEngine), dist(rndEngine), dist(rndEngine)));
                    gsite_j->SetGroupId(mMaxGroupNum++);
                    break;
                }

                // union
                gsite_i->GetCellPolygon()->ComputeVoroSitesList();
                gsite_j->GetCellPolygon()->ComputeVoroSitesList();

                auto A = gsite_i->GetCellPolygon()->GetPolygonVertices();
                auto B = gsite_j->GetCellPolygon()->GetPolygonVertices();

                cbop::Polygon polyA, polyB;

                cbop::Contour vert1, vert2;
                for (size_t ai = 0; ai < A.size(); ai++)
                    vert1.add(cbop::Point_2(A[ai].x, A[ai].y));
                polyA.push_back(vert1);

                for (size_t bi = 0; bi < B.size(); bi++)
                    vert2.add(cbop::Point_2(B[bi].x, B[bi].y));
                polyB.push_back(vert2);

                cbop::BooleanOpType op = cbop::UNION;

                cbop::Polygon result;
                auto compute_result = cbop::compute(polyA, polyB, result, op);

                if (!result.getContours().empty() && compute_result == true)
                {
                    auto p = result.getContours()[0].getPoints();

                    auto unionPolygon = std::make_shared<KiriVoroCellPolygon2>();
                    for (int pp = 0; pp < p.size(); ++pp)
                    {
                        unionPolygon->AddPolygonVertex2(Vector2F(p[pp].x(), p[pp].y()));
                    }

                    unionPolygon->SetColor(gsite_i->GetGroupColor());
                    unionPolygon->PopPolygonVertex2();
                    unionPolygon->ReversePolygonVertex2();

                    unionPolygon->UpdateBBox();
                    unionPolygon->ComputeVoroSitesList();

                    mUnionPolygonArray.emplace_back(unionPolygon);
                    //   KIRI_LOG_DEBUG("Union Success!!");
                }
                else
                {
                    // KIRI_LOG_ERROR("Union Failed!!");

                    gsite_i->SetFreeze(false);
                    gsite_j->SetFreeze(false);

                    gsite_j->SetGroupColor(Vector3F(dist(rndEngine), dist(rndEngine), dist(rndEngine)));
                    gsite_j->SetGroupId(mMaxGroupNum++);
                }

                break;
            }
        }

        // KIRI_LOG_DEBUG("mUnionPolygonArray size={0}", mUnionPolygonArray.size());

        return mUnionPolygonArray;
    }

    void KiriVoroNSOptimize::ComputeVoroSitePosConstrains()
    {
        auto voroSite = mPowerDiagram->GetVoroSites();

        mCurGlobalPosConstainWeightError = 0.f;
        mVoroPosConstainWeightError.assign(voroSite.size(), 0.f);
        mVoroPosConstainWeightAbsError.assign(voroSite.size(), 0.f);

        for (int i = 0; i < voroSite.size(); i++)
        {
            auto siteI = voroSite[i];
            if (siteI->GetIsFrozen())
                continue;

            if (!siteI->GetNeighborSites().empty())
            {
                auto radiusI = siteI->GetRadius();
                for (size_t j = 0; j < siteI->GetNeighborSites().size(); j++)
                {
                    auto siteJ = siteI->GetNeighborSites()[j];
                    if (siteJ->GetIsFrozen())
                        continue;

                    auto real_disij = siteI->GetDistance2(siteJ);
                    auto idea_disij = (siteI->GetWeight() - siteJ->GetWeight()) / (siteI->GetRadius() - siteJ->GetRadius() + 1e-9f);
                    auto dis_weight = idea_disij != 0.f ? (real_disij / idea_disij - 1.f) : 0.f;

                    if (std::abs(dis_weight) < 1e-3f && dis_weight != 0.f)
                    {
                        KIRI_LOG_DEBUG("match");
                        auto gsite_i = std::dynamic_pointer_cast<KiriVoroGroupSite>(siteI);
                        auto gsite_j = std::dynamic_pointer_cast<KiriVoroGroupSite>(siteJ);

                        gsite_j->SetGroupId(gsite_i->GetGroupId());
                        gsite_j->SetGroupColor(gsite_i->GetGroupColor());

                        gsite_i->SetFreeze(true);
                        gsite_j->SetFreeze(true);
                        break;
                    }

                    mVoroPosConstainWeightError[i] += dis_weight;
                    mVoroPosConstainWeightAbsError[i] += std::abs(dis_weight);
                    mCurGlobalPosConstainWeightError += std::abs(dis_weight);
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

            // if (polyi->GetPolygonArea() < total_area * coef && siteI->GetRadius() < total_rad * 2.3f)
            //     continue;

            if (siteI->GetRadius() < total_rad * 2.3f)
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
            nvs2->SetGroupColor(Vector3F(dist(rndEngine), dist(rndEngine), dist(rndEngine)));
            nvs2->SetGroupId(mMaxGroupNum++);
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