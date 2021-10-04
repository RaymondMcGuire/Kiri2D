/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-10-05 01:41:40
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_core.cpp
 */

#include <kiri2d/voronoi/voro_poropti_core.h>
#include <random>
namespace KIRI
{
    Vector2F LineFitLeastSquares(Vector<float> data)
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

        float k = (data.size() * C - B * D) / (tmp + MEpsilon<float>());
        float b = (A * D - B * C) / (tmp + MEpsilon<float>());

        return Vector2F(k, b);
    }

    Vector<Vector4F> KiriVoroPoroOptiCore::GetCellSSkel()
    {
        Vector<Vector4F> skeletons;
        auto sites = this->GetSites();

        for (size_t i = 0; i < sites.size(); i++)
        {
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                if (poly->GetSkeletons().empty())
                {
                    poly->ComputeSSkel1998Convex();
                }
                auto sk = poly->GetSkeletons();
                skeletons.insert(skeletons.end(), sk.begin(), sk.end());
            }
        }

        return skeletons;
    }

    Vector<Vector4F> KiriVoroPoroOptiCore::GetMICBySSkel()
    {
        Vector<Vector4F> circles;
        auto sites = this->GetSites();

        Vector<UInt> removeVoroIdxs;
        for (size_t i = 0; i < sites.size(); i++)
        {
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                if (poly->GetSkeletons().empty())
                    poly->ComputeSSkel1998Convex();

                auto mic = poly->ComputeMICByStraightSkeleton();
                circles.emplace_back(Vector4F(mic, sites[i]->GetRadius()));
            }
            else
            {
                removeVoroIdxs.emplace_back(sites[i]->GetIdx());
            }
        }

        //KIRI_LOG_INFO("GetMICBySSkel:EndPoint1");

        if (!removeVoroIdxs.empty())
            this->RemoveVoroSitesByIndexArray(removeVoroIdxs);

        //KIRI_LOG_INFO("GetMICBySSkel:EndPoint2");

        return circles;
    }

    float KiriVoroPoroOptiCore::ComputeMiniumPorosity()
    {
        //KIRI_LOG_INFO("Start GetMICBySSkel");
        auto maxCircleArray = this->GetMICBySSkel();
        //KIRI_LOG_INFO("Compeleted GetMICBySSkel");

        auto sum = 0.f;

        for (size_t i = 0; i < maxCircleArray.size(); i++)
        {
            auto circle = maxCircleArray[i];
            sum += circle.z * circle.z * KIRI_PI<float>();
        }

        auto boundary = mPowerDiagram->GetBoundaryPolygon2();
        return (boundary->GetPolygonArea() - sum) / boundary->GetPolygonArea();
    }

    void KiriVoroPoroOptiCore::Init()
    {
        Reset();

        ComputeBoundaryPolygonArea();

        CorrectVoroSitePos();

        mPowerDiagram->ComputeDiagram();
    }

    void KiriVoroPoroOptiCore::Reset()
    {
        mCurIteration = 0;
        mCompleteArea = 0.f;
        mVoroSitesWeightError.clear();
        mVoroSitesWeightAbsError.clear();
        mGlobalErrorArray.clear();
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
        auto outside = mPowerDiagram->Move2CentroidDisableSite();
        if (outside)
            CorrectWeights();
    }

    float KiriVoroPoroOptiCore::GetGlobalAreaError()
    {
        auto error = 0.f;
        auto voroSite = mPowerDiagram->GetVoroSites();
        for (size_t i = 0; i < voroSite.size(); i++)
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

    void KiriVoroPoroOptiCore::DynamicAddSites()
    {
        auto entityNum = 20;
        auto kThreshold = 200;

        if (mGlobalErrorArray.size() > entityNum)
        {
            bool bAddVoroSite = false;
            Vector<KiriVoroSitePtr> newVoroArrays;
            Vector<float> errorArray(mGlobalErrorArray.end() - entityNum, mGlobalErrorArray.end());
            auto line = LineFitLeastSquares(errorArray);
            //KIRI_LOG_DEBUG("line k ={0}", line.x);

            Vector<UInt> removeVoroIdxs;
            auto voroSite = mPowerDiagram->GetVoroSites();
            for (size_t i = 0; i < voroSite.size(); i++)
            {
                bool bNoPoly = false;
                if (std::abs(line.x) < kThreshold)
                {
                    auto poly = voroSite[i]->GetCellPolygon();
                    if (poly != NULL)
                    {
                        if (poly->GetSkeletons().empty())
                            poly->ComputeSSkel1998Convex();

                        auto mic = poly->ComputeMICByStraightSkeleton();
                        auto maxRadius = mic.z;
                        auto targetRadius = voroSite[i]->GetRadius();
                        // KIRI_LOG_DEBUG("maxRadius ={0},targetRadius={1}", maxRadius, targetRadius);
                        if (maxRadius > targetRadius)
                        {
                            auto pos = mPowerDiagram->GetBoundaryPolygon2()->GetRndInnerPoint();
                            auto nSite = std::make_shared<KiriVoroSite>(pos.x, pos.y);

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

                            nSite->SetRadius(pcdis(gen));
                            newVoroArrays.emplace_back(nSite);

                            bAddVoroSite = true;
                        }
                    }
                    else
                        bNoPoly = true;
                }

                // if (voroSite[i]->GetWeight() < 0.f || bNoPoly)
                //     removeVoroIdxs.emplace_back(voroSite[i]->GetIdx());

                if (bNoPoly)
                    removeVoroIdxs.emplace_back(voroSite[i]->GetIdx());
            }

            if (!removeVoroIdxs.empty())
                mPowerDiagram->RemoveVoroSitesByIndexArray(removeVoroIdxs);

            for (size_t i = 0; i < newVoroArrays.size(); i++)
                AddSite(newVoroArrays[i]);

            if (bAddVoroSite)
                mPowerDiagram->ResetVoroSitesWeight();
        }
    }

    void KiriVoroPoroOptiCore::AdaptWeights()
    {
        ComputeVoroSiteWeightError();

        auto gAreaError = GetGlobalAreaError();
        auto gAvgDistance = GetGlobalAvgDistance();

        auto gammaArea = 1.f;
        auto gammaBC = 1.f;

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

            //KIRI_LOG_DEBUG("AdaptWeights: error={0}, step={1}, weight={2}", error, step, weight);
            // // KIRI_LOG_DEBUG("AdaptWeights: mVoroSitesDisError={0}, mVoroSitesDisErrorAbs={1}, mCurGlobalDisError={2}", mVoroSitesDisError[i], mVoroSitesDisErrorAbs[i], mCurGlobalDisError);

            //KIRI_LOG_DEBUG("AdaptWeights: idx={0}, aw={1}, bw={2}", i, areaWeight, bcWeight);

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
                        auto pw = (distance * distance - siteI->GetWeight());
                        mVoroSitesWeightError[i] += pw;
                        mVoroSitesWeightAbsError[i] += std::abs(pw);
                        mCurGlobalWeightError += std::abs(pw);
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
        mCurIteration++;
        DynamicAddSites();
        AdaptPositionsWeights();
        AdaptWeights();

        //KIRI_LOG_DEBUG("---compute power---");
        mPowerDiagram->ComputeDiagram();
        //  if (!mPowerDiagram->ComputeDiagram())
        //     mPowerDiagram->ReGenVoroSites();

        return mCurGlobalWeightError;
    }

    float KiriVoroPoroOptiCore::ComputeIterate()
    {
        auto error = Iterate();
        mGlobalErrorArray.emplace_back(error);
        return error;
    }

    void KiriVoroPoroOptiCore::ComputeLloyd(UInt num)
    {
        mPowerDiagram->SetRelaxIterNumber(num);
        mPowerDiagram->LloydRelaxation();
    }
}