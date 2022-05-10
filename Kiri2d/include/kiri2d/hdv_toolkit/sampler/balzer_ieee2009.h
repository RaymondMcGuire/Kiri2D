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

        std::vector<Primitives::Vertex2Ptr> sites() { return mPowerDiagram->sites(); }

        void addSite(double x, double y, double capacity)
        {
            auto site = std::make_shared<Voronoi::CapacityVoronoiSite2>(x, y, mSiteCounter++);
            site->capacity = capacity;

            mPowerDiagram->addSite(site);
        }

        void setBoundaryPolygon(const std::shared_ptr<Voronoi::VoronoiPolygon2> &boundary)
        {
            mPowerDiagram->setBoundaryPolygon(boundary);
        }

        void init()
        {
            reset();

            computeBoundaryArea();

            mPowerDiagram->compute();
        }

        bool compute()
        {
            mCurIteration++;
            mPowerDiagram->lloydIteration();

            return computeFalsePosition();
        }

        void reset()
        {
            mStable = true;
            mCurIteration = 0;
            mCompleteArea = 0.0;
            mCurGlobalPorosity = 0.0;
        }

        void computeBoundaryArea()
        {
            auto boundary = mPowerDiagram->GetBoundary();
            if (!boundary)
            {
                KIRI_LOG_ERROR("Not Set Boundary!");
                return;
            }
            mCompleteArea = mPowerDiagram->GetBoundary()->area();
        }

        double deltaC(const Voronoi::CapacityVoronoiSite2Ptr &site, double weight)
        {
            site->setWeight(weight);
            mPowerDiagram->compute();

            auto area = 0.0;
            if (site->cellPolygon())
                area = site->cellPolygon()->area();

            return area - kiri_math_mini::pi<double>() * site->capacity * site->capacity;
        }

        bool computeFalsePosition()
        {
            auto sites = mPowerDiagram->sites();

            for (int i = 0; i < sites.size(); i++)
            {
                if (sites[i]->isBoundaryVertex())
                    continue;

                auto site_i = std::dynamic_pointer_cast<Voronoi::CapacityVoronoiSite2>(sites[i]);
                double a = mFPMA, b = mFPMB;
                auto fa = deltaC(site_i, a), fb = deltaC(site_i, b);
                if (fa * fb >= 0.0)
                {
                    KIRI_LOG_ERROR("Assumed A and B is Not Right!: f(a)={0}, f(b)={1}", fa, fb);
                    return false;
                }

                double c = a; // Initialize result

                for (int i = 0; i < mMaximumIteration; i++)
                {
                    // Find the point that touches x axis
                    c = (a * deltaC(site_i, b) - b * deltaC(site_i, a)) / (deltaC(site_i, b) - deltaC(site_i, a));

                    // Check if the above found point is root
                    if (deltaC(site_i, c) < 1e-6)
                        break;

                    // Decide the side to repeat the steps
                    else if (deltaC(site_i, c) * deltaC(site_i, a) < 0.0)
                        b = c;
                    else
                        a = c;
                }

                if (std::abs(deltaC(site_i, c)) > 1e-6)
                    mStable = false;
                site_i->setWeight(c);
            }

            return mStable;
        }

        std::vector<Vector4D> computeMICBySSkel()
        {
            std::vector<Vector4D> circles;
            auto sites = mPowerDiagram->sites();

            for (int i = 0; i < sites.size(); i++)
            {
                auto site_i = std::dynamic_pointer_cast<Voronoi::CapacityVoronoiSite2>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                auto poly = site_i->cellPolygon();
                if (poly)
                {

                    if (poly->skeletons().empty())
                        poly->computeSSkel1998Convex();

                    auto mic = poly->computeMICByStraightSkeleton();
                    circles.emplace_back(Vector4D(mic, site_i->radius()));
                    // KIRI_LOG_DEBUG("has poly: circle={0},{1},{2},{3}", mic.x, mic.y, mic.z, site_i->radius());
                }
                else
                {
                    KIRI_LOG_ERROR("computeMICBySSkel: No Polygon Data!!!");
                    // remove.emplace_back(site_i->radius());
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