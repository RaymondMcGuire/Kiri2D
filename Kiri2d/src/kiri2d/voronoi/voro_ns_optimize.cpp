/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-10-20 00:47:43
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_core.cpp
 */

#include <kiri2d/voronoi/voro_ns_optimize.h>
#include <polyclipper2d/PolygonClipping.h>
#include <random>
#include <bop12/booleanop.h>

#ifdef KIRI_WINDOWS
#include <omp.h>
#endif

namespace KIRI
{
    void KiriVoroNSOptimize::init()
    {
        reset();

        ComputeBoundaryPolygonArea();

        CorrectVoroSitePos();

        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroNSOptimize::reset()
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
                // if (voroSite[i]->GetIsFrozen())
                //     continue;

                auto pos = voroSite[i]->GetValue();
                if (!boundary->contains(Vector2F(pos.x, pos.y)))
                    voroSite[i]->ResetValue(boundary->rndInnerPoint());
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

    void KiriVoroNSOptimize::adaptPositionsWeights()
    {
        auto outside = mPowerDiagram->Move2CentroidDisableSite();
    }

    float KiriVoroNSOptimize::globalAreaError()
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
                auto current_area = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
                auto target_area = n * voroSite[i]->radius() * voroSite[i]->radius() * std::tanf(KIRI_PI<float>() / n);
                error += std::abs(target_area - current_area) / (mCompleteArea * 2.f);
            }
        }
        return error;
    }

    void KiriVoroNSOptimize::removeNoiseVoroSites()
    {
        Vector<UInt> remove_voro_idxs;
        auto voroSite = mPowerDiagram->GetVoroSites();
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto site_i = voroSite[i];
            auto data = site_i->GetValue();

            auto polyI = site_i->GetCellPolygon();
            if (polyI != NULL)
            {
                // check boundary
                auto boundary_poly = mPowerDiagram->GetBoundaryPolygon2();
                auto boundary_contain = boundary_poly->contains(polyI->GetPolygonCentroid());
                if (!boundary_contain)
                {
                    remove_voro_idxs.emplace_back(site_i->GetIdx());
                    continue;
                }

                if (polyI->GetSkeletons().empty())
                    polyI->computeSSkel1998Convex();

                auto mic_i = polyI->computeMICByStraightSkeleton();

                auto neighbors = site_i->GetNeighborSites();
                for (size_t j = 0; j < neighbors.size(); j++)
                {
                    auto site_j = neighbors[j];
                    if (site_j->GetIdx() == site_i->GetIdx())
                        continue;

                    auto poly_j = site_j->GetCellPolygon();
                    if (poly_j != NULL)
                    {
                        if (poly_j->GetSkeletons().empty())
                            poly_j->computeSSkel1998Convex();

                        auto micJ = poly_j->computeMICByStraightSkeleton();
                        auto dist_ij = (Vector2F(mic_i.x, mic_i.y) - Vector2F(micJ.x, micJ.y)).length();
                        if ((dist_ij < ((mic_i.z + micJ.z) / 2.f)) &&
                            !std::binary_search(remove_voro_idxs.begin(), remove_voro_idxs.end(), site_i->GetIdx()) &&
                            !std::binary_search(remove_voro_idxs.begin(), remove_voro_idxs.end(), site_j->GetIdx()))
                            remove_voro_idxs.emplace_back(site_j->GetIdx());
                    }
                }
            }
        }

        if (!remove_voro_idxs.empty())
        {
            // KIRI_LOG_DEBUG("remove overlapping cell, size={0}", remove_voro_idxs.size());
            mPowerDiagram->removeVoroSitesByIndexArray(remove_voro_idxs);
            adaptPositionsWeights();
            // mPowerDiagram->ComputeDiagram();
        }
    }

    void KiriVoroNSOptimize::adaptWeights()
    {
        ComputeVoroSiteWeightError();
        ComputeVoroSitePosConstrains();

        auto g_area_error = globalAreaError();
        auto g_avg_distance = globalAvgDistance();

        auto gamma_area = 1.f;
        auto gamma_bc = 1.f;
        auto gammaPS = 1000.f;

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (int i = 0; i < voroSite.size(); i++)
        {
            // if (voroSite[i]->GetIsFrozen())
            //     continue;

            auto weight = voroSite[i]->weight();

            auto weight_area = 0.f;
            auto weight_bc = 0.f;
            auto psWeight = 0.f;

            auto n = voroSite[i]->GetNeighborSites().size();
            if (n > 2)
            {
                auto current_area = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
                auto target_area = n * voroSite[i]->radius() * voroSite[i]->radius() * std::tanf(KIRI_PI<float>() / n);

                auto area_percentage = 2.f;
                if (current_area != 0.f)
                    area_percentage = target_area / current_area;

                auto area_error_transform = (-(g_area_error - 1.f) * (g_area_error - 1.f) + 1.f);
                auto area_step = g_avg_distance * area_error_transform * gamma_area;
                if (area_percentage < (1.f - std::numeric_limits<float>::epsilon()) && weight > 0.f)
                    weight_area -= area_step;
                else if (area_percentage > (1.f + std::numeric_limits<float>::epsilon()))
                    weight_area += area_step;
            }

            auto error = mVoroSitesWeightAbsError[i] / (mCurGlobalWeightError + std::numeric_limits<float>::epsilon());
            auto error_transform = (-(error - 1.f) * (error - 1.f) + 1.f);

            auto step = error_transform * gamma_bc;
            if (mVoroSitesWeightError[i] < 0.f)
                weight_bc -= step;
            else if (mVoroSitesWeightError[i] > 0.f)
                weight_bc += step;

            auto poserror = mVoroSitesWeightAbsError[i] / (mCurGlobalWeightError + std::numeric_limits<float>::epsilon());
            auto poserrorTransform = (-(error - 1.f) * (error - 1.f) + 1.f);

            auto posstep = poserrorTransform * gammaPS;
            if (mVoroPosConstainWeightError[i] < 0.f)
                psWeight -= posstep;
            else if (mVoroPosConstainWeightError[i] > 0.f)
                psWeight += posstep;

            // KIRI_LOG_DEBUG("adaptWeights: error={0}, step={1}, weight={2}", error, step, weight);
            //  // KIRI_LOG_DEBUG("adaptWeights: mVoroSitesDisError={0}, mVoroSitesDisErrorAbs={1}, mCurGlobalDisError={2}", mVoroSitesDisError[i], mVoroSitesDisErrorAbs[i], mCurGlobalDisError);

            // KIRI_LOG_DEBUG("adaptWeights: idx={0}, aw={1}, bw={2}", i, weight_area, weight_bc);

            voroSite[i]->setWeight(weight + weight_area + weight_bc + psWeight);
        }

        // KIRI_LOG_DEBUG("adaptWeights: mCurGlobalWeightError={0}", mCurGlobalWeightError);
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
            auto site_i = voroSite[i];

            // if (site_i->GetIsFrozen())
            //     continue;

            if (!site_i->GetNeighborSites().empty())
            {

                auto radius_i = site_i->radius();
                for (size_t j = 0; j < site_i->GetNeighborSites().size(); j++)
                {
                    auto site_j = site_i->GetNeighborSites()[j];
                    total += site_j->weight() - site_j->radius() * site_j->radius();
                    cnt++;
                }

                auto avg = total / cnt;

                for (size_t j = 0; j < site_i->GetNeighborSites().size(); j++)
                {
                    auto site_j = site_i->GetNeighborSites()[j];

                    auto distance = site_i->GetDistance2(site_j);
                    auto minW = std::abs(std::sqrt(site_j->weight()) - std::sqrt(site_i->weight()));
                    auto maxW = std::sqrt(site_j->weight()) + std::sqrt(site_i->weight());
                    if (distance < maxW && distance > minW)
                    {
                        auto sum = site_j->weight() - site_j->radius() * site_j->radius();
                        mVoroSitesWeightError[i] += avg - sum;
                        mVoroSitesWeightAbsError[i] += std::abs(avg - sum);
                        mCurGlobalWeightError += std::abs(avg - sum);
                    }
                    else
                    {

                        auto pw = distance * distance - site_i->weight();
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
        std::random_device seed;
        std::default_random_engine engine(seed());
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto site_i = voroSite[i];
            if (!site_i->GetIsFrozen())
                continue;

            for (size_t j = 0; j < voroSite.size(); j++)
            {
                auto site_j = voroSite[j];
                if (!site_j->GetIsFrozen() || site_i->GetIdx() == site_j->GetIdx())
                    continue;

                auto gsite_i = std::dynamic_pointer_cast<KiriVoroGroupSite>(site_i);
                auto gsite_j = std::dynamic_pointer_cast<KiriVoroGroupSite>(site_j);

                if (gsite_i->GetGroupId() != gsite_j->GetGroupId())
                    continue;

                if ((!gsite_i->GetCellPolygon()->bbox().overlaps(gsite_j->GetCellPolygon()->bbox())) || (gsite_i->GetCellPolygon()->bbox().contains(gsite_j->GetCellPolygon()->bbox())))
                {

                    gsite_i->SetFreeze(false);
                    gsite_j->SetFreeze(false);

                    gsite_j->SetGroupColor(Vector3F(dist(engine), dist(engine), dist(engine)));
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

                    unionPolygon->updateBBox();
                    unionPolygon->ComputeVoroSitesList();

                    mUnionPolygonArray.emplace_back(unionPolygon);
                    //   KIRI_LOG_DEBUG("Union Success!!");
                }
                else
                {
                    // KIRI_LOG_ERROR("Union Failed!!");

                    gsite_i->SetFreeze(false);
                    gsite_j->SetFreeze(false);

                    gsite_j->SetGroupColor(Vector3F(dist(engine), dist(engine), dist(engine)));
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
            auto site_i = voroSite[i];
            if (site_i->GetIsFrozen())
                continue;

            if (!site_i->GetNeighborSites().empty())
            {
                auto radius_i = site_i->radius();
                for (size_t j = 0; j < site_i->GetNeighborSites().size(); j++)
                {
                    auto site_j = site_i->GetNeighborSites()[j];
                    if (site_j->GetIsFrozen())
                        continue;

                    auto real_disij = site_i->GetDistance2(site_j);
                    auto idea_disij = 0.5f * (site_i->radius() + site_j->radius());
                    auto dis_weight = idea_disij != 0.f ? (real_disij / idea_disij - 1.f) : 0.f;

                    // if (dis_weight != 0.f && !site_i->GetIsFrozen() && !site_j->GetIsFrozen())
                    if (std::abs(dis_weight) < 1e-3f && dis_weight != 0.f && !site_i->GetIsFrozen() && !site_j->GetIsFrozen())
                    {
                        // KIRI_LOG_DEBUG("match");
                        auto gsite_i = std::dynamic_pointer_cast<KiriVoroGroupSite>(site_i);
                        auto gsite_j = std::dynamic_pointer_cast<KiriVoroGroupSite>(site_j);

                        gsite_j->SetGroupId(gsite_i->GetGroupId());
                        gsite_j->SetGroupColor(gsite_i->GetGroupColor());

                        gsite_i->SetFreeze(true);
                        gsite_j->SetFreeze(true);
                        // gsite_j->setRadius(gsite_i->radius());
                        break;
                    }

                    mVoroPosConstainWeightError[i] += dis_weight;
                    mVoroPosConstainWeightAbsError[i] += std::abs(dis_weight);
                    mCurGlobalPosConstainWeightError += std::abs(dis_weight);
                }
            }
        }
    }

    float KiriVoroNSOptimize::globalAvgDistance()
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
            KIRI_LOG_ERROR("globalAvgDistance:: no neighbor site!!");
            return 0.f;
        }

        return sum / num;
    }

    void KiriVoroNSOptimize::SplitEventProcess()
    {
        auto coef = 0.5f;
        std::random_device seed;
        std::default_random_engine engine(seed());
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        Vector<KiriVoroGroupSitePtr> new_sites;
        Vector<UInt> remove_voro_idxs;

        auto voroSite = mPowerDiagram->GetVoroSites();

        // record avg radius
        auto total_rad = 0.f;
        for (int i = 0; i < voroSite.size(); i++)
        {
            total_rad += voroSite[i]->radius();
            // KIRI_LOG_DEBUG(" rad={0}", voroSite[i]->radius());
        }
        total_rad /= voroSite.size();

        // check if need split
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto total_area = 0.f;
            auto site_i = std::dynamic_pointer_cast<KiriVoroGroupSite>(voroSite[i]);
            auto polyi = site_i->GetCellPolygon();
            if (polyi == nullptr)
                continue;

            if (!site_i->GetNeighborSites().empty())
            {
                for (size_t j = 0; j < site_i->GetNeighborSites().size(); j++)
                {
                    auto site_j = site_i->GetNeighborSites()[j];
                    auto polyj = site_j->GetCellPolygon();
                    total_area += polyj != nullptr ? polyj->GetPolygonArea() : 0.f;
                }
            }

            // if (polyi->GetPolygonArea() < total_area * coef && site_i->radius() < total_rad * 2.3f)
            //     continue;

            if (site_i->radius() < total_rad * 2.3f)
                continue;

            // split event
            auto centroid = polyi->GetPolygonCentroid();

            // rnd direction
            auto rnd_norm_dir = Vector2F(dist(engine), dist(engine)).normalized();

            auto new_vs1 = centroid + rnd_norm_dir;
            auto new_vs2 = centroid - rnd_norm_dir;

            // KIRI_LOG_DEBUG("cen={0},{1}, new1={2},{3}, new2={4},{5}", centroid.x, centroid.y, new_vs1.x, new_vs1.y, new_vs2.x, new_vs2.y);

            if (!polyi->contains(new_vs1) || !polyi->contains(new_vs2))
            {
                KIRI_LOG_ERROR("New Voronoi Site Pos is ERROR!!!");
                continue;
            }

            auto nvs1 = std::make_shared<KiriVoroGroupSite>(new_vs1);
            nvs1->setWeight(site_i->weight() / 1.2f);
            nvs1->setRadius(site_i->radius() / std::sqrtf(2.f));
            nvs1->SetGroupColor(site_i->GetGroupColor());
            nvs1->SetGroupId(site_i->GetGroupId());

            auto nvs2 = std::make_shared<KiriVoroGroupSite>(new_vs2);
            nvs2->setWeight(site_i->weight() / 1.2f);
            nvs2->setRadius(site_i->radius() / std::sqrtf(2.f));
            nvs2->SetGroupColor(Vector3F(dist(engine), dist(engine), dist(engine)));
            nvs2->SetGroupId(mMaxGroupNum++);
            new_sites.emplace_back(nvs1);
            new_sites.emplace_back(nvs2);

            remove_voro_idxs.emplace_back(site_i->GetIdx());
        }

        if (!remove_voro_idxs.empty())
        {
            mPowerDiagram->removeVoroSitesByIndexArray(remove_voro_idxs);
            KIRI_LOG_DEBUG("remove sites num={0}, total num={1}", remove_voro_idxs.size(), mPowerDiagram->GetVoroSites().size());

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
        adaptPositionsWeights();
        adaptWeights();

        mPowerDiagram->ComputeDiagram();
        removeNoiseVoroSites();
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