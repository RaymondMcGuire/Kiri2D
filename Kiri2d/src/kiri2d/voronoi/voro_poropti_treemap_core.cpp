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

    void KiriVoroPoroOptiTreeMapCore::Init()
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
                if (!boundary->Contains(Vector2F(pos.x, pos.y)))
                    voroSite[i]->ResetValue(boundary->GetRndInnerPoint());
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

    void KiriVoroPoroOptiTreeMapCore::AdaptPositionsWeights()
    {

        auto outside = mPowerDiagram->Move2Centroid();

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
            auto siteI = voroSite[i];

            if (!siteI->GetNeighborSites().empty())
            {

                auto radiusI = siteI->radius();
                for (size_t j = 0; j < siteI->GetNeighborSites().size(); j++)
                {
                    auto siteJ = siteI->GetNeighborSites()[j];
                    total += siteJ->weight() - siteJ->radius() * siteJ->radius();
                    cnt++;
                }

                auto avg = total / cnt;

                for (size_t j = 0; j < siteI->GetNeighborSites().size(); j++)
                {
                    auto siteJ = siteI->GetNeighborSites()[j];

                    auto distance = siteI->GetDistance2(siteJ);
                    auto minW = std::abs(std::sqrt(siteJ->weight()) - std::sqrt(siteI->weight()));
                    auto maxW = std::sqrt(siteJ->weight()) + std::sqrt(siteI->weight());
                    if (distance < maxW && distance > minW)
                    {
                        auto sum = siteJ->weight() - siteJ->radius() * siteJ->radius();
                        mVoroSitesWeightError[i] += avg - sum;
                        mVoroSitesWeightAbsError[i] += std::abs(avg - sum);
                        mCurGlobalWeightError += std::abs(avg - sum);
                    }
                    else
                    {
                        mVoroSitesWeightError[i] += distance * distance - siteI->weight();
                        mVoroSitesWeightAbsError[i] += std::abs(distance * distance - siteI->weight());
                        mCurGlobalWeightError += std::abs(distance * distance - siteI->weight());
                    }
                }
            }
        }
    }

    void KiriVoroPoroOptiTreeMapCore::AdaptWeights()
    {
        ComputeVoroSiteWeightError();

        auto gAreaError = GetGlobalAreaError();
        // auto gAvgDistance = GetGlobalAvgDistance();

        auto gammaArea = 1000.f;
        auto gammaBC = 1.f;

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto weight = voroSite[i]->weight();

            auto areaWeight = 0.f;
            auto bcWeight = 0.f;

            // auto n = voroSite[i]->GetNeighborSites().size();
            // if (n > 2)
            //{
            auto currentArea = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
            auto targetArea = mCompleteArea * voroSite[i]->GetPercentage();

            auto pArea = 2.f;
            if (currentArea != 0.f)
                pArea = targetArea / currentArea;

            auto areaErrorTransform = (-(gAreaError - 1.f) * (gAreaError - 1.f) + 1.f);
            auto areaStep = areaErrorTransform * gammaArea;
            if (pArea < (1.f - std::numeric_limits<float>::epsilon()) && weight > 0.f)
                areaWeight -= areaStep;
            else if (pArea > (1.f + std::numeric_limits<float>::epsilon()))
                areaWeight += areaStep;
            //}

            // auto error = mVoroSitesWeightAbsError[i] / (mCurGlobalWeightError + std::numeric_limits<float>::epsilon());
            // auto errorTransform = (-(error - 1.f) * (error - 1.f) + 1.f);

            // auto step = errorTransform * gammaBC;
            // if (mVoroSitesWeightError[i] < 0.f)
            //     bcWeight -= step;
            // else if (mVoroSitesWeightError[i] > 0.f)
            //     bcWeight += step;

            voroSite[i]->setWeight(weight + areaWeight + bcWeight);
        }

        // KIRI_LOG_DEBUG("AdaptWeights: mCurGlobalWeightError={0}", mCurGlobalWeightError);
    }

    float KiriVoroPoroOptiTreeMapCore::GetGlobalAreaError()
    {

        auto error = 0.f;
        if (mCompleteArea == 0.f)
        {
            KIRI_LOG_ERROR("GetGlobalAreaError::Please set boundary polygon!!");
            return error;
        }

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto currentArea = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
            auto targetArea = mCompleteArea * voroSite[i]->GetPercentage();
            error += std::abs(targetArea - currentArea) / (mCompleteArea * 2.f);
        }
        return error;
    }

    float KiriVoroPoroOptiTreeMapCore::GetGlobalAvgDistance()
    {
        float sum = 0.f;
        UInt num = 0;
        auto voroSite = mPowerDiagram->GetVoroSites();

        for (size_t i = 0; i < voroSite.size(); i++)
        {
            // KIRI_LOG_DEBUG("voro site pos={0},{1}", voroSite[i]->GetValue().x, voroSite[i]->GetValue().y);
            //  KIRI_LOG_ERROR("GetGlobalAvgDistance:: voro n num={0}", voroSite[i]->GetNeighborSites().size());
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
            KIRI_LOG_ERROR("GetGlobalAvgDistance:: no neighbor site!!");
            return 0.f;
        }

        return sum / num;
    }

    float KiriVoroPoroOptiTreeMapCore::Iterate()
    {

        AdaptPositionsWeights();
        AdaptWeights();
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