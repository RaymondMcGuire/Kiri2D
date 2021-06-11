/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-11 11:05:49
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

        // ComputeBoundaryPolygonArea();

        CorrectVoroSitePos();

        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroPoroOptiCore::Reset()
    {
    }

    void KiriVoroPoroOptiCore::ComputeVoroSiteMovement()
    {
        auto epsilon = 0.001f;
        auto voroSite = mPowerDiagram->GetVoroSites();
        mVoroSitesMovemnet.assign(voroSite.size(), Vector2F(0.f));
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            // TODO change param name
            auto radiusI = voroSite[i]->GetPercentage();
            if (!voroSite[i]->GetNeighborSites().empty())
            {
                for (size_t j = 0; j < voroSite[i]->GetNeighborSites().size(); j++)
                {
                    auto siteJ = voroSite[i]->GetNeighborSites()[j];
                    auto distance = voroSite[i]->GetDistance2(siteJ);
                    auto targetDis = distance - 2.f * radiusI;
                    auto moveDir = Vector2F(voroSite[i]->GetValue().x, voroSite[i]->GetValue().y) - Vector2F(siteJ->GetValue().x, siteJ->GetValue().y);
                    auto moveDis = moveDir.normalized() * targetDis * epsilon;
                    mVoroSitesMovemnet[siteJ->GetIdx()] += moveDis;
                }
            }
        }
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
        // if (mPowerDiagram->GetBoundaryPolygon2() == NULL)
        // {
        //     KIRI_LOG_ERROR("GetBoundaryPolygonArea::Please set boundary polygon!!");
        //     mCompleteArea = 0.f;
        //     return;
        // }

        // mCompleteArea = mPowerDiagram->GetBoundaryPolygon2()->GetPolygonArea();
    }

    void KiriVoroPoroOptiCore::SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary)
    {
        //mCompleteArea = boundary->GetPolygonArea();
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
        // if (outside)
        //     CorrectWeights();
    }

    void KiriVoroPoroOptiCore::AdaptWeights()
    {
        ComputeVoroSiteWeightError();
        //auto gAvg = GetGlobalAvgDistance();

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto error = mVoroSitesWeightAbsError[i] / (mCurGlobalWeightError + MEpsilon<float>());
            auto errorTransform = (-(error - 1.f) * (error - 1.f) + 1.f);
            auto scale = 1000.f;
            auto step = errorTransform * scale;
            auto weight = voroSite[i]->GetWeight();

            if (mVoroSitesWeightError[i] < 0.f)
                weight -= step;
            else if (mVoroSitesWeightError[i] > 0.f)
                weight += step;

            //KIRI_LOG_DEBUG("AdaptWeights: error={0}, step={1}, weight={2}", error, step, weight);
            // // KIRI_LOG_DEBUG("AdaptWeights: mVoroSitesDisError={0}, mVoroSitesDisErrorAbs={1}, mCurGlobalDisError={2}", mVoroSitesDisError[i], mVoroSitesDisErrorAbs[i], mCurGlobalDisError);

            voroSite[i]->SetWeight(weight);
        }

        // KIRI_LOG_DEBUG("AdaptWeights: mCurGlobalWeightError={0}", mCurGlobalWeightError);
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
            if (!voroSite[i]->GetNeighborSites().empty())
            {
                // TODO change param name
                auto radiusI = voroSite[i]->GetPercentage();
                for (size_t j = 0; j < voroSite[i]->GetNeighborSites().size(); j++)
                {
                    auto siteJ = voroSite[i]->GetNeighborSites()[j];
                    total += siteJ->GetWeight() + siteJ->GetPercentage() * siteJ->GetPercentage();
                    cnt++;
                }

                auto avg = total / cnt;

                for (size_t j = 0; j < voroSite[i]->GetNeighborSites().size(); j++)
                {
                    auto siteJ = voroSite[i]->GetNeighborSites()[j];
                    auto sum = siteJ->GetWeight() + siteJ->GetPercentage() * siteJ->GetPercentage();
                    mVoroSitesWeightError[siteJ->GetIdx()] += avg - sum;
                    mVoroSitesWeightAbsError[siteJ->GetIdx()] += std::abs(avg - sum);
                    mCurGlobalWeightError += std::abs(avg - sum);
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
            //KIRI_LOG_DEBUG("voro site pos={0},{1}", voroSite[i]->GetValue().x, voroSite[i]->GetValue().y);
            // KIRI_LOG_ERROR("GetGlobalAvgDistance:: voro n num={0}", voroSite[i]->GetNeighborSites().size());
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