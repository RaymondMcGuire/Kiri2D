/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-07 17:29:38
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 18:59:11
 * @FilePath: \Kiri2D\core\include\kiri2d\hdv_toolkit\voronoi\voronoi_nocaj_core.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _HDV_VORONOI_NOCAJ_CORE_H_
#define _HDV_VORONOI_NOCAJ_CORE_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/power_diagram.h>

namespace HDV::VoronoiTreeMap
{

    class VoronoiNocajCore
    {
    public:
        explicit VoronoiNocajCore()
        {
            reset();
            mPowerDiagram = std::make_shared<Voronoi::PowerDiagram2D>();
        }

        virtual ~VoronoiNocajCore() {}

        void setBoundary(const std::shared_ptr<Voronoi::VoronoiPolygon2> &boundary)
        {
            mPowerDiagram->setBoundary(boundary);
        }

        void setMaxIterationNum(UInt num) { mMaxIterationNum = num; }
        void setErrorThreshold(float error) { mErrorThreshold = error; }

        std::vector<Primitives::Vertex2Ptr> sites()
        {
            return mPowerDiagram->sites();
        }

        const std::shared_ptr<Voronoi::VoronoiPolygon2> &boundary()
        {
            return mPowerDiagram->boundary();
        }

        constexpr UInt maxIterationNum() const { return mMaxIterationNum; }
        constexpr float errorThreshold() const { return mErrorThreshold; }

        void init()
        {

            reset();

            computeBoundaryArea();

            computePercentage();

            mPowerDiagram->compute();
        }

        void reset()
        {
            mCompleteArea = 0.f;
            mCurGlobalAreaError = 1.f;
            mCurIteration = 0;

            mErrorThreshold = 0.3f;
            mMaxIterationNum = 35;
        }

        void computeIterate()
        {
            iterate();
        }

        void addSite(HDV::Primitives::Vertex2Ptr site)
        {
            mPowerDiagram->addSite(site);
        }

    private:
        float mCompleteArea = 0.f, mCurGlobalAreaError = 1.f, mErrorThreshold = 0.3f;
        UInt mCurIteration = 0, mMaxIterationNum = 35;

        Voronoi::PowerDiagram2Ptr mPowerDiagram;

        void iterate()
        {
            adaptPositionsWeights();
            adaptWeights();
            mPowerDiagram->compute();
        }

        void computeBoundaryArea()
        {
            auto boundary = mPowerDiagram->boundary();
            if (!boundary)
            {
                KIRI_LOG_ERROR("Not Set Boundary!");
                return;
            }
            mCompleteArea = mPowerDiagram->boundary()->area();

            // KIRI_LOG_DEBUG("boundary area={0}", mCompleteArea);
        }

        void computePercentage()
        {
            auto sites = mPowerDiagram->sites();
            if (!sites.empty())
            {

                auto sum = 0.f;
                for (size_t i = 0; i < sites.size(); i++)
                {

                    auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                    if (site_i->isBoundaryVertex())
                        continue;

                    sum += sites[i]->percentage();
                }

                for (size_t i = 0; i < sites.size(); i++)
                {
                    auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                    if (site_i->isBoundaryVertex())
                        continue;

                    sites[i]->setPercentage(sites[i]->percentage() / sum);
                }

                // KIRI_LOG_DEBUG("computePercentage sum={0}; site size={1}", sum, sites.size());
            }
            else
                KIRI_LOG_ERROR("computePercentage::No voro site!!");
        }

        void adaptPositionsWeights()
        {
            mPowerDiagram->move2Centroid();
        }

        void adaptWeights()
        {
            auto gAvg = globalAvgDistance();
            auto error = globalAreaError();

            auto sites = mPowerDiagram->sites();
            for (size_t i = 0; i < sites.size(); i++)
            {
                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                auto current_area = (!site_i->polygon()) ? 0.0 : site_i->polygon()->area();
                auto target_area = mCompleteArea * sites[i]->percentage();

                auto increase = 2.f;
                if (current_area != 0.f)
                    increase = target_area / current_area;

                auto error_transform = (-(error - 1.f) * (error - 1.f) + 1.f);
                auto step = 1.f * gAvg * error_transform;
                auto weight = sites[i]->weight();

                auto epsilon = 0.01f;
                if (increase < (1.f - epsilon))
                    weight -= step;
                else if (increase > (1.f + epsilon))
                    weight += step;

                sites[i]->setWeight(weight);
            }
        }

        float globalAreaError()
        {

            auto error = 0.0;
            auto sites = mPowerDiagram->sites();

            for (int i = 0; i < sites.size(); i++)
            {
                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                auto n = site_i->neighbors().size();
                if (n > 2)
                {
                    auto current_area = (!site_i->polygon()) ? 0.0 : site_i->polygon()->area();
                    auto target_area = mCompleteArea * sites[i]->percentage();

                    error += std::abs(target_area - current_area) / (mCompleteArea * 2.0);
                }
            }
            return error;
        }
        float globalAvgDistance()
        {
            auto sum = 0.0;
            auto num = 0;
            auto sites = mPowerDiagram->sites();

            for (int i = 0; i < sites.size(); i++)
            {
                if (sites[i]->isBoundaryVertex())
                    continue;

                if (!sites[i]->neighbors().empty())
                {
                    for (auto neighbor : sites[i]->neighbors())
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex2>(neighbor);
                        auto distance = sites[i]->distanceTo(sn);
                        sum += distance;
                        num++;
                    }
                }

                // KIRI_LOG_DEBUG("sites size={0}; neighbors num={1}", sites.size(), sites[i]->neighbors().size());
            }

            // for (int i = 0; i < sites.size(); i++)
            //     KIRI_LOG_DEBUG("site id={0}; isB={1}; neighbor size={2}", sites[i]->id(), sites[i]->isBoundaryVertex(), sites[i]->neighbors().size());

            if (num == 0)
            {
                KIRI_LOG_WARN("globalAvgDistance:: no neighbor sites!!");
                return 0.0;
            }

            return sum / num;
        }
    };

    typedef SharedPtr<VoronoiNocajCore> VoronoiNocajCorePtr;
}
#endif