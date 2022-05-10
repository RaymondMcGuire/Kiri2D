/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-10-20 00:47:43
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_core.cpp
 */

#include <kiri2d/voronoi/voro_poropti_core.h>
#include <random>
#ifdef KIRI_WINDOWS
#include <omp.h>
#endif

namespace KIRI
{
    Vector2F lineFitLeastSquares(Vector<float> data)
    {
        float A = 0.f;
        float B = 0.f;
        float C = 0.f;
        float D = 0.f;

        for (int i = 0; i < data.size(); i++)
        {
            A += i * i;
            B += i;
            C += i * data[i];
            D += data[i];
        }

        float tmp = data.size() * A - B * B;

        float k = (data.size() * C - B * D) / (tmp + std::numeric_limits<float>::epsilon());
        float b = (A * D - B * C) / (tmp + std::numeric_limits<float>::epsilon());

        return Vector2F(k, b);
    }

    Vector<Vector4F> KiriVoroPoroOptiCore::GetCellSSkel()
    {
        Vector<Vector4F> skeletons;
        auto sites = this->sites();

#pragma omp parallel for
        for (int i = 0; i < sites.size(); i++)
        {
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                if (poly->GetSkeletons().empty())
                {
                    poly->computeSSkel1998Convex();
                }
                auto sk = poly->GetSkeletons();
                skeletons.insert(skeletons.end(), sk.begin(), sk.end());
            }
        }

        return skeletons;
    }

    Vector<Vector4F> KiriVoroPoroOptiCore::computeMICBySSkel()
    {
        Vector<Vector4F> circles;
        auto sites = this->sites();

        Vector<UInt> remove_voro_idxs;

        for (int i = 0; i < sites.size(); i++)
        {
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                if (poly->GetSkeletons().empty())
                    poly->computeSSkel1998Convex();

                auto mic = poly->computeMICByStraightSkeleton();
                circles.emplace_back(Vector4F(mic, sites[i]->radius()));
            }
            else
            {
                remove_voro_idxs.emplace_back(sites[i]->GetIdx());
            }
        }

        // KIRI_LOG_INFO("computeMICBySSkel:EndPoint1");

        if (!remove_voro_idxs.empty())
            this->removeVoroSitesByIndexArray(remove_voro_idxs);

        // KIRI_LOG_INFO("computeMICBySSkel:EndPoint2");

        return circles;
    }

    float KiriVoroPoroOptiCore::computeMiniumPorosity()
    {
        // KIRI_LOG_INFO("Start computeMICBySSkel");
        auto maximum_circles = this->computeMICBySSkel();
        // KIRI_LOG_INFO("Compeleted computeMICBySSkel");

        auto sum = 0.f;
#pragma omp parallel for reduction(+ \
                                   : sum)
        for (int i = 0; i < maximum_circles.size(); i++)
        {
            auto circle = maximum_circles[i];
            sum += circle.z * circle.z * KIRI_PI<float>();
        }

        auto boundary = mPowerDiagram->GetBoundaryPolygon2();
        return (boundary->GetPolygonArea() - sum) / boundary->GetPolygonArea();
    }

    void KiriVoroPoroOptiCore::init()
    {
        reset();

        ComputeBoundaryPolygonArea();

        CorrectVoroSitePos();

        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroPoroOptiCore::reset()
    {
        mCurGlobalPorosity = 0.f;
        mCurIteration = 0;
        mCompleteArea = 0.f;
        mVoroSitesWeightError.clear();
        mVoroSitesWeightAbsError.clear();
        mGlobalErrorArray.clear();
        mGlobalPorosityArray.clear();
    }

    void KiriVoroPoroOptiCore::CorrectVoroSitePos()
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
                if (!boundary->contains(Vector2F(pos.x, pos.y)))
                    voroSite[i]->ResetValue(boundary->rndInnerPoint());
            }
        }
        else
            KIRI_LOG_ERROR("CorrectVoroSitePos::No voro site!!");
    }

    void KiriVoroPoroOptiCore::ComputeBoundaryPolygonArea()
    {
        if (mPowerDiagram->GetBoundaryPolygon2() == NULL)
        {
            KIRI_LOG_ERROR("GetBoundaryPolygonArea::Please set boundary polygon!!");
            mCompleteArea = 0.f;
            return;
        }

        mCompleteArea = mPowerDiagram->GetBoundaryPolygon2()->GetPolygonArea();
    }

    void KiriVoroPoroOptiCore::SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary)
    {
        mCompleteArea = boundary->GetPolygonArea();
        mPowerDiagram->SetBoundaryPolygon2(boundary);
    }

    void KiriVoroPoroOptiCore::CorrectWeights()
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
                    if (std::sqrt(voroSite[i]->weight()) >= distance)
                        voroSite[j]->setWeight(distance * distance);
                }
            }
        }
    }

    void KiriVoroPoroOptiCore::adaptPositionsWeights()
    {
        auto outside = mPowerDiagram->Move2CentroidDisableSite();
        // if (outside)
        //     CorrectWeights();
    }

    float KiriVoroPoroOptiCore::globalAreaError()
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

    void KiriVoroPoroOptiCore::removeNoiseVoroSites()
    {
        Vector<UInt> remove_voro_idxs;
        auto voroSite = mPowerDiagram->GetVoroSites();
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto site_i = voroSite[i];
            auto polyI = site_i->GetCellPolygon();
            if (polyI != NULL)
            {
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
        }
    }

    void KiriVoroPoroOptiCore::dynamicAddSites()
    {
        if (mPowerDiagram->GetVoroSites().size() >= mMaxiumNum && bReachMaximumNum == false)
            bReachMaximumNum = true;

        auto entity_num = 20;
        auto k_threshold = 200;

        if (mGlobalPorosityArray.size() > entity_num)
        {
            bool bAddVoroSite = false;
            Vector<KiriVoroSitePtr> new_sites;
            Vector<float> errorArray(mGlobalPorosityArray.end() - entity_num, mGlobalPorosityArray.end());
            auto line = lineFitLeastSquares(errorArray);
            // KIRI_LOG_DEBUG("line k={0}", std::abs(line.x));
            if (std::abs(line.x) < 1e-6f)
            {
                // KIRI_LOG_DEBUG("reach line res={0}", std::abs(line.x) < 1e-6f);
                std::vector<float> radiusRange;
                radiusRange.push_back(20.f);
                radiusRange.push_back(30.f);
                radiusRange.push_back(80.f);
                radiusRange.push_back(150.f);

                std::vector<float> radiusRangeProb;
                radiusRangeProb.push_back(0.5f);
                radiusRangeProb.push_back(0.4f);
                radiusRangeProb.push_back(0.1f);

                std::random_device engine;
                std::mt19937 gen(engine());
                std::piecewise_constant_distribution<float> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

                if (bReachMaximumNum)
                {
                    auto pos = mPowerDiagram->GetBoundaryPolygon2()->rndInnerPoint();
                    auto site = std::make_shared<KiriVoroSite>(pos.x, pos.y);
                    site->setRadius(pcdis(gen));
                    new_sites.emplace_back(site);
                }
                else
                {
                    auto voroSite = mPowerDiagram->GetVoroSites();
                    for (int i = 0; i < voroSite.size(); i++)
                    {
                        auto pos = mPowerDiagram->GetBoundaryPolygon2()->rndInnerPoint();
                        auto new_site = std::make_shared<KiriVoroSite>(pos.x, pos.y);

                        new_site->setRadius(pcdis(gen));
                        new_sites.emplace_back(new_site);
                    }
                }
            }

            auto new_vorosite_num = new_sites.size();
            auto cur_vorosite_num = mPowerDiagram->GetVoroSites().size();
            auto need_append_vorosite_num = new_vorosite_num;

            if (!bReachMaximumNum)
            {
                if ((new_vorosite_num + cur_vorosite_num) > mMaxiumNum)
                    need_append_vorosite_num = mMaxiumNum - cur_vorosite_num;

                for (int i = 0; i < need_append_vorosite_num; i++)
                    addSite(new_sites[i]);
            }
            else
            {
                auto current_mp = computeMiniumPorosity();
                if (mLastMP > current_mp)
                {
                    // KIRI_LOG_DEBUG("add P");
                    for (int i = 0; i < need_append_vorosite_num; i++)
                        addSite(new_sites[i]);

                    mLastMP = current_mp;
                }
            }

            // if (bAddVoroSite)
            //     mPowerDiagram->ResetVoroSitesWeight();
        }
    }

    void KiriVoroPoroOptiCore::adaptWeights()
    {
        ComputeVoroSiteWeightError();

        auto g_area_error = globalAreaError();
        auto g_avg_distance = globalAvgDistance();

        auto gamma_area = 1.f;
        auto gamma_bc = 1.f;

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (int i = 0; i < voroSite.size(); i++)
        {
            auto weight = voroSite[i]->weight();

            auto weight_area = 0.f;
            auto weight_bc = 0.f;

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

            // KIRI_LOG_DEBUG("adaptWeights: error={0}, step={1}, weight={2}", error, step, weight);
            //  // KIRI_LOG_DEBUG("adaptWeights: mVoroSitesDisError={0}, mVoroSitesDisErrorAbs={1}, mCurGlobalDisError={2}", mVoroSitesDisError[i], mVoroSitesDisErrorAbs[i], mCurGlobalDisError);

            // KIRI_LOG_DEBUG("adaptWeights: idx={0}, aw={1}, bw={2}", i, weight_area, weight_bc);

            voroSite[i]->setWeight(weight + weight_area + weight_bc);
        }

        // KIRI_LOG_DEBUG("adaptWeights: mCurGlobalWeightError={0}", mCurGlobalWeightError);
    }

    void KiriVoroPoroOptiCore::ComputeVoroSiteWeightError()
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

    float KiriVoroPoroOptiCore::globalAvgDistance()
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

    float KiriVoroPoroOptiCore::Iterate()
    {
        mCurIteration++;
        dynamicAddSites();
        adaptPositionsWeights();
        adaptWeights();

        // KIRI_LOG_DEBUG("---compute power---");
        mPowerDiagram->ComputeDiagram();
        //  if (!mPowerDiagram->ComputeDiagram())
        //     mPowerDiagram->ReGenVoroSites();
        removeNoiseVoroSites();

        return computeMiniumPorosity();
    }

    float KiriVoroPoroOptiCore::ComputeIterate()
    {
        mCurGlobalPorosity = Iterate();
        mGlobalPorosityArray.emplace_back(mCurGlobalPorosity);
        mGlobalErrorArray.emplace_back(mCurGlobalWeightError);
        return mCurGlobalWeightError;
    }

    void KiriVoroPoroOptiCore::ComputeLloyd(UInt num)
    {
        mPowerDiagram->SetRelaxIterNumber(num);
        mPowerDiagram->LloydRelaxation();
    }
}