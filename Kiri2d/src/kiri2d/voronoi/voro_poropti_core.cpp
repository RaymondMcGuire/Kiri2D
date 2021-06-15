/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-15 13:23:12
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_core.cpp
 */

#include <kiri2d/voronoi/voro_poropti_core.h>
namespace KIRI
{

    void KiriVoroPoroOptiCore::Init()
    {

        Reset();

        ComputeBoundaryPolygonArea();

        CorrectVoroSitePos();

        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroPoroOptiCore::InitWeight()
    {
        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto neighborSize = voroSite[i]->GetNeighborSites().size();
            if (neighborSize > 2)
            {
                auto sqrtWeight = voroSite[i]->GetRadius() / std::cosf(KIRI_PI<float>() / neighborSize);
                voroSite[i]->SetWeight(sqrtWeight * sqrtWeight);
            }
        }
    }

    void KiriVoroPoroOptiCore::Reset()
    {
        mCompleteArea = 0.f;
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
        for (size_t i = 0; i < voroSite.size(); i++)
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

    void KiriVoroPoroOptiCore::AdaptPositionsWeights()
    {
        //mPowerDiagram->Move2Centroid();
        //ComputeVoroSiteMovement();
        //auto outside = mPowerDiagram->MoveVoroSites(mVoroSitesMovemnet);

        auto outside = mPowerDiagram->Move2Centroid();
        if (outside)
            CorrectWeights();
    }

    float KiriVoroPoroOptiCore::GetGlobalAreaError()
    {
        auto error = 0.f;
        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto currentArea = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
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

    void KiriVoroPoroOptiCore::AdaptWeights()
    {
        ComputeVoroSiteWeightError();

        auto gAreaError = GetGlobalAreaError();
        auto gAvgDistance = GetGlobalAvgDistance();

        auto gammaArea = 1.f;
        auto gammaBC = 0.1f;

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
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
                if (pArea < (1.f - MEpsilon<float>()))
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

            //KIRI_LOG_DEBUG("AdaptWeights: error={0}, step={1}, weight={2}", error, step, weight);
            // // KIRI_LOG_DEBUG("AdaptWeights: mVoroSitesDisError={0}, mVoroSitesDisErrorAbs={1}, mCurGlobalDisError={2}", mVoroSitesDisError[i], mVoroSitesDisErrorAbs[i], mCurGlobalDisError);

            //KIRI_LOG_DEBUG("AdaptWeights: idx={0}, weight={1}", i, weight + areaWeight + bcWeight);
            voroSite[i]->SetWeight(weight + areaWeight + bcWeight);
        }

        KIRI_LOG_DEBUG("AdaptWeights: mCurGlobalWeightError={0}", mCurGlobalWeightError);
    }

    void KiriVoroPoroOptiCore::ComputeVoroSiteWeightError()
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

                auto radiusI = siteI->GetRadius();
                for (size_t j = 0; j < siteI->GetNeighborSites().size(); j++)
                {
                    auto siteJ = siteI->GetNeighborSites()[j];
                    //total += siteJ->GetWeight() + siteJ->GetRadius() * siteJ->GetRadius();
                    total += siteJ->GetWeight() - siteJ->GetRadius() * siteJ->GetRadius();
                    cnt++;
                }

                auto avg = total / cnt;

                for (size_t j = 0; j < siteI->GetNeighborSites().size(); j++)
                {
                    auto siteJ = siteI->GetNeighborSites()[j];
                    // auto sum = siteJ->GetWeight() + siteJ->GetRadius() * siteJ->GetRadius();
                    // mVoroSitesWeightError[siteJ->GetIdx()] += avg - sum;
                    // mVoroSitesWeightAbsError[siteJ->GetIdx()] += std::abs(avg - sum);
                    // mCurGlobalWeightError += std::abs(avg - sum);

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
                        mVoroSitesWeightError[i] += distance * distance - siteI->GetWeight();
                        mVoroSitesWeightAbsError[i] += std::abs(distance * distance - siteI->GetWeight());
                        mCurGlobalWeightError += std::abs(distance * distance - siteI->GetWeight());
                    }
                }
            }
        }
    }

    float KiriVoroPoroOptiCore::GetGlobalAvgDistance()
    {
        float sum = 0.f;
        UInt num = 0;
        auto voroSite = mPowerDiagram->GetVoroSites();

        for (size_t i = 0; i < voroSite.size(); i++)
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

    float KiriVoroPoroOptiCore::Iterate()
    {

        AdaptPositionsWeights();
        AdaptWeights();
        if (!mPowerDiagram->ComputeDiagram())
            mPowerDiagram->ReGenVoroSites();

        return mCurGlobalWeightError;
    }

    float KiriVoroPoroOptiCore::ComputeIterate()
    {
        return Iterate();
    }

    void KiriVoroPoroOptiCore::ComputeDiagram()
    {
        if (!mPowerDiagram->ComputeDiagram())
            mPowerDiagram->ReGenVoroSites();
    }

    void KiriVoroPoroOptiCore::ComputeLloyd(UInt num)
    {
        mPowerDiagram->SetRelaxIterNumber(num);
        mPowerDiagram->LloydRelaxation();
    }

    void KiriVoroPoroOptiCore::Compute()
    {
        Init();

        while (mCurIteration < mMaxIterationNum)
        {
            Iterate();

            mCurIteration++;
        }
    }
}