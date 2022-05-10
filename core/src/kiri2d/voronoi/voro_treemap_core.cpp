/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-04 15:38:37
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_treemap_core.cpp
 */

#include <kiri2d/voronoi/voro_treemap_core.h>
namespace KIRI
{

    void KiriVoroTreeMapCore::init()
    {

        reset();

        ComputeBoundaryPolygonArea();

        CorrectVoroSitePos();

        ReComputePercentage();

        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroTreeMapCore::reset()
    {
        mCompleteArea = 0.f;
        mCurGlobalAreaError = 1.f;
        mCurMaxAreaError = 1.f;
        mCurIteration = 0;

        mErrorThreshold = 0.3f;
        mMaxIterationNum = 35;
    }

    void KiriVoroTreeMapCore::CorrectVoroSitePos()
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

    void KiriVoroTreeMapCore::ReComputePercentage()
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

    void KiriVoroTreeMapCore::ComputeBoundaryPolygonArea()
    {
        if (mPowerDiagram->GetBoundaryPolygon2() == NULL)
        {
            KIRI_LOG_ERROR("GetBoundaryPolygonArea::Please set boundary polygon!!");
            mCompleteArea = 0.f;
            return;
        }

        mCompleteArea = mPowerDiagram->GetBoundaryPolygon2()->GetPolygonArea();
    }

    void KiriVoroTreeMapCore::SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary)
    {
        mCompleteArea = boundary->GetPolygonArea();
        mPowerDiagram->SetBoundaryPolygon2(boundary);
    }

    void KiriVoroTreeMapCore::CorrectWeights()
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

    void KiriVoroTreeMapCore::adaptPositionsWeights()
    {

        auto outside = mPowerDiagram->move2Centroid();

        if (outside)
            CorrectWeights();
    }

    void KiriVoroTreeMapCore::adaptWeights()
    {
        auto gAvg = globalAvgDistance();
        auto error = globalAreaError();

        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto current_area = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
            auto target_area = mCompleteArea * voroSite[i]->GetPercentage();

            auto increase = 2.f;
            if (current_area != 0.f)
                increase = target_area / current_area;

            auto error_transform = (-(error - 1.f) * (error - 1.f) + 1.f);
            auto step = 1.f * gAvg * error_transform;
            auto weight = voroSite[i]->weight();

            auto epsilon = 0.01f;
            if (increase < (1.f - epsilon))
                weight -= step;
            else if (increase > (1.f + epsilon))
                weight += step;

            voroSite[i]->setWeight(weight);
        }
    }

    float KiriVoroTreeMapCore::globalAreaError()
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

    float KiriVoroTreeMapCore::GetMaxAreaError()
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
            auto current_area = (voroSite[i]->GetCellPolygon() == NULL) ? 0.f : voroSite[i]->GetCellPolygon()->GetPolygonArea();
            auto target_area = mCompleteArea * voroSite[i]->GetPercentage();
            auto error = std::abs(target_area - current_area) / (mCompleteArea * 2.f);
            maxError = std::max(maxError, error);
        }
        return maxError;
    }

    float KiriVoroTreeMapCore::globalAvgDistance()
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

    void KiriVoroTreeMapCore::Iterate()
    {
        adaptPositionsWeights();
        adaptWeights();
        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroTreeMapCore::ComputeIterate()
    {
        Iterate();

        // mCurGlobalAreaError = globalAreaError();
        // mCurMaxAreaError = GetMaxAreaError();
        // KIRI_LOG_INFO("Iteration:{0}, Local max area error={1}, Global area error={2}", mCurIteration, mCurMaxAreaError, mCurGlobalAreaError);
    }

    void KiriVoroTreeMapCore::compute()
    {
        init();

        while (mCurIteration < mMaxIterationNum)
        {
            Iterate();

            mCurGlobalAreaError = globalAreaError();
            mCurMaxAreaError = GetMaxAreaError();
            mCurIteration++;

            if (mCurGlobalAreaError < mErrorThreshold)
                break;
        }

        KIRI_LOG_INFO("Iteration:{0}, Local max area error={1}, Global area error={2}", mCurIteration, mCurMaxAreaError, mCurGlobalAreaError);
    }
}