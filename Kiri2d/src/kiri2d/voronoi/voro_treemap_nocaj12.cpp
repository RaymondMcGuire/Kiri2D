/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-05-28 16:59:09
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_treemap_nocaj12.cpp
 */

#include <kiri2d/voronoi/voro_treemap_nocaj12.h>
namespace KIRI
{
    void KiriVoroTreemapNocaj12::Reset()
    {
        mCompleteArea = 0.f;
        mCurGlobalAreaError = 1.f;
        mCurMaxAreaError = 1.f;
        mCurIteration = 0;

        mErrorThreshold = 0.3f;
        mMaxIterationNum = 800;
    }

    void KiriVoroTreemapNocaj12::ComputeBoundaryPolygonArea()
    {
        if (mPowerDiagram->GetBoundaryPolygon2() == NULL)
        {
            KIRI_LOG_ERROR("GetBoundaryPolygonArea::Please set boundary polygon!!");
            mCompleteArea = 0.f;
            return;
        }

        mCompleteArea = mPowerDiagram->GetBoundaryPolygon2()->GetPolygonArea();
    }

    void KiriVoroTreemapNocaj12::SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary)
    {
        mCompleteArea = boundary->GetPolygonArea();
        mPowerDiagram->SetBoundaryPolygon2(boundary);
    }

    void KiriVoroTreemapNocaj12::CorrectWeights()
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

    void KiriVoroTreemapNocaj12::AdaptPositionsWeights()
    {
        auto outside = mPowerDiagram->Move2Centroid();
        if (outside)
            CorrectWeights();
    }

    void KiriVoroTreemapNocaj12::AdaptWeights()
    {
        auto gAvg = GetGlobalAvgDistance();
        auto error = GetGlobalAreaError();

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto currentArea = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
            auto targetArea = mCompleteArea * voroSite[i]->GetPercentage();

            auto increase = 2.f;
            if (currentArea == 0.f)
                increase = targetArea / currentArea;

            auto errorTransform = (-(error - 1.f) * (error - 1.f) + 1.f);
            auto step = 1.f * gAvg * errorTransform;
            auto weight = voroSite[i]->GetWeight();

            if (increase < (1.f - MEpsilon<float>()))
                weight -= step;
            else if (increase > (1.f + MEpsilon<float>()))
                weight += step;

            voroSite[i]->SetWeight(weight);
        }
    }

    float KiriVoroTreemapNocaj12::GetGlobalAreaError()
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

    float KiriVoroTreemapNocaj12::GetMaxAreaError()
    {
        auto maxError = 0.f;
        if (mCompleteArea == 0.f)
        {
            KIRI_LOG_ERROR("GetMaxAreaError::Please set boundary polygon!!");
            return maxError;
        }

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto currentArea = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
            auto targetArea = mCompleteArea * voroSite[i]->GetPercentage();
            auto error = std::abs(targetArea - currentArea) / (mCompleteArea * 2.f);
            maxError = std::max(maxError, error);
        }
        return maxError;
    }

    float KiriVoroTreemapNocaj12::GetGlobalAvgDistance()
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
                    sum += voroSite[i]->GetDistance2(voroSite[i]->GetNeighborSites()[j]);
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

    void KiriVoroTreemapNocaj12::Iterate()
    {
        AdaptPositionsWeights();

        AdaptWeights();

        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroTreemapNocaj12::Compute()
    {
        Reset();
        ComputeBoundaryPolygonArea();

        mPowerDiagram->ComputeDiagram();

        while (mCurIteration < mMaxIterationNum)
        {
            Iterate();

            mCurGlobalAreaError = GetGlobalAreaError();
            mCurMaxAreaError = GetMaxAreaError();
            mCurIteration++;

            if (mCurGlobalAreaError < mErrorThreshold)
                break;
        }

        KIRI_LOG_INFO("Iteration:{0}, Local max area error={1}, Global area error={2}", mCurIteration, mCurMaxAreaError, mCurGlobalAreaError);
    }
}