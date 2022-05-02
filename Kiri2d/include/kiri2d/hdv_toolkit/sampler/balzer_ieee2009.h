/***
 * @Author: Xu.WANG
 * @Date: 2022-01-10 22:51:36
 * @LastEditTime: 2022-01-10 23:03:16
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_BALZER_IEEE2009_H_
#define _HDV_BALZER_IEEE2009_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/power_diagram.h>
#include <kiri2d/geo/convex_clip2.h>

namespace HDV::Sampler
{
    class BalzerIEEE2009
    {
    public:
        explicit BalzerIEEE2009() { mPowerDiagram = std::make_shared<Voronoi::PowerDiagram2D>(); }
        virtual ~BalzerIEEE2009() {}

        std::vector<Primitives::Vertex2Ptr> GetSites() { return mPowerDiagram->GetSites(); }

        void AddSite(double x, double y, double capacity)
        {
            auto site = std::make_shared<Voronoi::CapacityVoronoiSite2>(x, y, mSiteCounter++);
            site->capacity = capacity;

            mPowerDiagram->AddSite(site);
        }

        void SetBoundaryPolygon(const std::shared_ptr<Voronoi::VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> &boundary)
        {
            mPowerDiagram->SetBoundaryPolygon(boundary);
        }

        void Init()
        {
            reset();

            ComputeBoundaryArea();

            mPowerDiagram->Compute();
        }

        bool Compute()
        {
            mCurIteration++;
            mPowerDiagram->LloydIteration();
            // ComputeFalsePosition();

            // KIRI_LOG_DEBUG("------");
            // auto sites = mPowerDiagram->GetSites();
            // for (int i = 0; i < sites.size(); i++)
            // {
            //     if (sites[i]->isBoundaryVertex())
            //         continue;

            //     auto siteI = std::dynamic_pointer_cast<Voronoi::CapacityVoronoiSite2>(sites[i]);

            //     auto area = 0.0;
            //     if (siteI->CellPolygon)
            //         area = siteI->CellPolygon->GetArea();
            //     KIRI_LOG_DEBUG("site id={0}, area={1}", siteI->id(), area);
            // }

            return ComputeFalsePosition();
        }

        void reset()
        {
            mStable = true;
            mCurIteration = 0;
            mCompleteArea = 0.0;
            mCurGlobalPorosity = 0.0;
        }

        void ComputeBoundaryArea()
        {
            auto boundary = mPowerDiagram->GetBoundary();
            if (!boundary)
            {
                KIRI_LOG_ERROR("Not Set Boundary!");
                return;
            }
            mCompleteArea = mPowerDiagram->GetBoundary()->GetArea();
        }

        double DeltaC(const Voronoi::CapacityVoronoiSite2Ptr &site, double weight)
        {
            site->setWeight(weight);
            mPowerDiagram->Compute();

            auto area = 0.0;
            if (site->CellPolygon)
                area = site->CellPolygon->GetArea();

            return area - kiri_math_mini::pi<double>() * site->capacity * site->capacity;
        }

        bool ComputeFalsePosition()
        {
            auto sites = mPowerDiagram->GetSites();

            for (int i = 0; i < sites.size(); i++)
            {
                if (sites[i]->isBoundaryVertex())
                    continue;

                auto siteI = std::dynamic_pointer_cast<Voronoi::CapacityVoronoiSite2>(sites[i]);
                double a = mFPMA, b = mFPMB;
                auto fa = DeltaC(siteI, a), fb = DeltaC(siteI, b);
                if (fa * fb >= 0.0)
                {
                    KIRI_LOG_ERROR("Assumed A and B is Not Right!: f(a)={0}, f(b)={1}", fa, fb);
                    return false;
                }

                double c = a; // Initialize result

                for (int i = 0; i < mMaximumIteration; i++)
                {
                    // Find the point that touches x axis
                    c = (a * DeltaC(siteI, b) - b * DeltaC(siteI, a)) / (DeltaC(siteI, b) - DeltaC(siteI, a));

                    // Check if the above found point is root
                    if (DeltaC(siteI, c) < 1e-6)
                        break;

                    // Decide the side to repeat the steps
                    else if (DeltaC(siteI, c) * DeltaC(siteI, a) < 0.0)
                        b = c;
                    else
                        a = c;
                }

                if (std::abs(DeltaC(siteI, c)) > 1e-6)
                    mStable = false;
                siteI->setWeight(c);
            }

            return mStable;
        }

        std::vector<Vector4D> GetMICBySSkel()
        {
            std::vector<Vector4D> circles;
            auto sites = mPowerDiagram->GetSites();

            for (int i = 0; i < sites.size(); i++)
            {
                auto siteI = std::dynamic_pointer_cast<Voronoi::CapacityVoronoiSite2>(sites[i]);
                if (siteI->isBoundaryVertex())
                    continue;

                auto poly = siteI->CellPolygon;
                if (poly)
                {

                    if (poly->mSkeletons.empty())
                        poly->ComputeSSkel1998Convex();

                    auto mic = poly->ComputeMICByStraightSkeleton();
                    circles.emplace_back(Vector4D(mic, siteI->radius()));
                    // KIRI_LOG_DEBUG("has poly: circle={0},{1},{2},{3}", mic.x, mic.y, mic.z, siteI->radius());
                }
                else
                {
                    KIRI_LOG_ERROR("GetMICBySSkel: No Polygon Data!!!");
                    // remove.emplace_back(siteI->radius());
                }
            }

            return circles;
        }

    private:
        bool mStable = true;
        int mSiteCounter = 0;
        int mCurIteration = 0;
        int mMaximumIteration = 100000;
        double mFPMA = -1000.0;
        double mFPMB = 1000.0;

        double mCompleteArea = 0.0;
        double mCurGlobalPorosity = 0.0;
        double mCurGlobalWeightError = 0.0;

        Voronoi::PowerDiagram2Ptr mPowerDiagram;
    };

} // namespace HDV::Sampler

#endif /* _HDV_BALZER_IEEE2009_H_ */