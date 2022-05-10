/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-24 16:01:43
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_treemap_core.cpp
 */

#include <kiri2d/voronoi/voro_poropti_treemap_core.h>
namespace KIRI
{

    void KiriVoroPoroOptiTreeMapCore::init()
    {

        reset();

        ComputeBoundaryPolygonArea();

        CorrectVoroSitePos();

        ReComputePercentage();

        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroPoroOptiTreeMapCore::reset()
    {
        mCompleteArea = 0.f;
        mVoroSitesWeightError.clear();
        mVoroSitesWeightAbsError.clear();
        mGlobalErrorArray.clear();
    }

    void KiriVoroPoroOptiTreeMapCore::CorrectVoroSitePos()
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
            for (size_t i = 0; i < voroSite.size(); i++)
            {
                auto pos = voroSite[i]->GetValue();
                if (!boundary->contains(Vector2F(pos.x, pos.y)))
                    voroSite[i]->ResetValue(boundary->rndInnerPoint());
            }
        }
        else
            KIRI_LOG_ERROR("CorrectVoroSitePos::No voro site!!");
    }

    void KiriVoroPoroOptiTreeMapCore::ReComputePercentage()
    {
        auto voroSite = mPowerDiagram->GetVoroSites();
        if (!voroSite.empty())
        {
            auto sum = 0.f;
            for (size_t i = 0; i < voroSite.size(); i++)
                sum += voroSite[i]->GetPercentage();

            for (size_t i = 0; i < voroSite.size(); i++)
                voroSite[i]->SetPercentage(voroSite[i]->GetPercentage() / sum);
        }
        else
            KIRI_LOG_ERROR("ReComputePercentage::No voro site!!");
    }

    void KiriVoroPoroOptiTreeMapCore::ComputeBoundaryPolygonArea()
    {
        if (mPowerDiagram->GetBoundaryPolygon2() == NULL)
        {
            KIRI_LOG_ERROR("GetBoundaryPolygonArea::Please set boundary polygon!!");
            mCompleteArea = 0.f;
            return;
        }

        mCompleteArea = mPowerDiagram->GetBoundaryPolygon2()->GetPolygonArea();
    }

    void KiriVoroPoroOptiTreeMapCore::SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary)
    {
        mCompleteArea = boundary->GetPolygonArea();
        mPowerDiagram->SetBoundaryPolygon2(boundary);
    }

    void KiriVoroPoroOptiTreeMapCore::CorrectWeights()
    {
        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
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

    void KiriVoroPoroOptiTreeMapCore::adaptPositionsWeights()
    {

        auto outside = mPowerDiagram->move2Centroid();

        if (outside)
            CorrectWeights();
    }

    void KiriVoroPoroOptiTreeMapCore::ComputeVoroSiteWeightError()
    {
        mCurGlobalWeightError = 0.f;
        auto voroSite = mPowerDiagram->GetVoroSites();
        mVoroSitesWeightError.assign(voroSite.size(), 0.f);
        mVoroSitesWeightAbsError.assign(voroSite.size(), 0.f);

        for (size_t i = 0; i < voroSite.size(); i++)
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
                        mVoroSitesWeightError[i] += distance * distance - site_i->weight();
                        mVoroSitesWeightAbsError[i] += std::abs(distance * distance - site_i->weight());
                        mCurGlobalWeightError += std::abs(distance * distance - site_i->weight());
                    }
                }
            }
        }
    }

    void KiriVoroPoroOptiTreeMapCore::adaptWeights()
    {
        ComputeVoroSiteWeightError();

        auto g_area_error = globalAreaError();
        // auto g_avg_distance = globalAvgDistance();

        auto gamma_area = 1000.f;
        auto gamma_bc = 1.f;

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto weight = voroSite[i]->weight();

            auto weight_area = 0.f;
            auto weight_bc = 0.f;

            // auto n = voroSite[i]->GetNeighborSites().size();
            // if (n > 2)
            //{
            auto current_area = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
            auto target_area = mCompleteArea * voroSite[i]->GetPercentage();

            auto area_percentage = 2.f;
            if (current_area != 0.f)
                area_percentage = target_area / current_area;

            auto area_error_transform = (-(g_area_error - 1.f) * (g_area_error - 1.f) + 1.f);
            auto area_step = area_error_transform * gamma_area;
            if (area_percentage < (1.f - std::numeric_limits<float>::epsilon()) && weight > 0.f)
                weight_area -= area_step;
            else if (area_percentage > (1.f + std::numeric_limits<float>::epsilon()))
                weight_area += area_step;
            //}

            // auto error = mVoroSitesWeightAbsError[i] / (mCurGlobalWeightError + std::numeric_limits<float>::epsilon());
            // auto error_transform = (-(error - 1.f) * (error - 1.f) + 1.f);

            // auto step = error_transform * gamma_bc;
            // if (mVoroSitesWeightError[i] < 0.f)
            //     weight_bc -= step;
            // else if (mVoroSitesWeightError[i] > 0.f)
            //     weight_bc += step;

            voroSite[i]->setWeight(weight + weight_area + weight_bc);
        }

        // KIRI_LOG_DEBUG("adaptWeights: mCurGlobalWeightError={0}", mCurGlobalWeightError);
    }

    float KiriVoroPoroOptiTreeMapCore::globalAreaError()
    {

        auto error = 0.f;
        if (mCompleteArea == 0.f)
        {
            KIRI_LOG_ERROR("globalAreaError::Please set boundary polygon!!");
            return error;
        }

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto current_area = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
            auto target_area = mCompleteArea * voroSite[i]->GetPercentage();
            error += std::abs(target_area - current_area) / (mCompleteArea * 2.f);
        }
        return error;
    }

    float KiriVoroPoroOptiTreeMapCore::globalAvgDistance()
    {
        float sum = 0.f;
        UInt num = 0;
        auto voroSite = mPowerDiagram->GetVoroSites();

        for (size_t i = 0; i < voroSite.size(); i++)
        {
            // KIRI_LOG_DEBUG("voro site pos={0},{1}", voroSite[i]->GetValue().x, voroSite[i]->GetValue().y);
            //  KIRI_LOG_ERROR("globalAvgDistance:: voro n num={0}", voroSite[i]->GetNeighborSites().size());
            if (!voroSite[i]->GetNeighborSites().empty())
            {
                for (size_t j = 0; j < voroSite[i]->GetNeighborSites().size(); j++)
                {
                    auto distance = voroSite[i]->GetDistance2(voroSite[i]->GetNeighborSites()[j]);
                    // KIRI_LOG_DEBUG("distance = {0}", distance);
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

    float KiriVoroPoroOptiTreeMapCore::Iterate()
    {

        adaptPositionsWeights();
        adaptWeights();
        mPowerDiagram->ComputeDiagram();

        return mCurGlobalWeightError;
    }

    float KiriVoroPoroOptiTreeMapCore::ComputeIterate()
    {
        auto error = Iterate();
        mGlobalErrorArray.emplace_back(error);
        return error;
    }

}