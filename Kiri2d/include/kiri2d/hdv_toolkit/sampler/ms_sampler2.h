/***
 * @Author: Xu.WANG
 * @Date: 2022-01-10 22:51:36
 * @LastEditTime: 2022-01-10 23:03:16
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_MS_SAMPLER2_H_
#define _HDV_MS_SAMPLER2_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/power_diagram.h>
#include <kiri2d/geo/convex_clip2.h>

namespace HDV::Sampler
{
    class MultiSizedSampler2D
    {
    public:
        explicit MultiSizedSampler2D()
        {
            mPowerDiagram = std::make_shared<Voronoi::PowerDiagram2D>();
        }

        virtual ~MultiSizedSampler2D()
        {
        }

        std::vector<Primitives::Vertex2Ptr> sites()
        {
            return mPowerDiagram->sites();
        }

        void addSite(double x, double y, double radius)
        {
            auto site = std::make_shared<Voronoi::VoronoiSite2>(x, y, mSiteCounter++);
            site->setRadius(radius);
            mPowerDiagram->addSite(site);
        }

        void setMaxiumNum(int num)
        {
            mMaxiumNum = num;
        }

        void setBoundaryPolygon(const std::shared_ptr<Voronoi::VoronoiPolygon2> &boundary)
        {
            mPowerDiagram->setBoundaryPolygon(boundary);
        }

        void setRadiusDist(std::vector<double> dist)
        {
            mUsrDefinedRadiusDist = dist;
        }

        void setRadiusDistProb(std::vector<double> prob)
        {
            mUsrDefinedRadiusDistProb = prob;
        }

        void init()
        {
            reset();

            computeBoundaryArea();

            //! TODO check site position whether is inside boundary or not
            mPowerDiagram->compute();
        }

        bool checkVoroCell()
        {
            std::vector<int> remove;

            auto sites = mPowerDiagram->sites();

            for (auto i = 0; i < sites.size(); i++)
            {

                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                if (!site_i->cellPolygon())
                    remove.emplace_back(site_i->id());
                else
                {
                    auto centroid = site_i->cellPolygon()->centroid();
                    if (!mPowerDiagram->GetBoundary()->contains(centroid))
                        remove.emplace_back(site_i->id());
                }
            }

            if (!remove.empty())
            {
                mPowerDiagram->removeVoroSitesByIndexArray(remove);
                return true;
            }

            return false;
        }

        float compute()
        {
            mCurIteration++;

            auto b_add_sites = this->dynamicAddSites();

            if (b_add_sites)
            {
                mPowerDiagram->compute();
                // KIRI_LOG_DEBUG("Iter={0} : add Sites!!!!", mCurIteration);
            }
            else
            {
                mPowerDiagram->move2Centroid();
                this->computeWeightsError();
                this->adaptWeights();
                mPowerDiagram->compute();
            }

            if (checkVoroCell())
            {
                mPowerDiagram->compute();
            }

            mCurGlobalPorosity = this->computeMiniumPorosity();
            mGlobalPorosityArray.emplace_back(mCurGlobalPorosity);
            mGlobalErrorArray.emplace_back(mCurGlobalWeightError);

            // KIRI_LOG_DEBUG("porosity={0}; error={1}", mCurGlobalPorosity, mCurGlobalWeightError);
            return mCurGlobalPorosity;
        }

        void reset()
        {
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

        double globalAreaError()
        {
            auto error = 0.0;
            auto sites = mPowerDiagram->sites();

#pragma omp parallel for reduction(+ \
                                   : error)
            for (int i = 0; i < sites.size(); i++)
            {
                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                auto n = site_i->neighbors().size();
                if (n > 2)
                {
                    auto current_area = (!site_i->cellPolygon()) ? 0.0 : site_i->cellPolygon()->area();
                    auto target_area = n * site_i->radius() * site_i->radius() * std::tan(kiri_math_mini::pi<double>() / n);
                    // auto target_area = site_i->radius() * site_i->radius() * kiri_math_mini::pi<double>();
                    error += std::abs(target_area - current_area) / (mCompleteArea * 2.0);
                }
            }
            return error;
        }

        double globalAvgDistance()
        {
            double sum = 0.0;
            int num = 0;
            auto site = mPowerDiagram->sites();

#pragma omp parallel for reduction(+ \
                                   : sum, num)
            for (int i = 0; i < site.size(); i++)
            {
                if (site[i]->isBoundaryVertex())
                    continue;

                if (!site[i]->neighbors().empty())
                {
                    for (auto neighbor : site[i]->neighbors())
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex2>(neighbor);
                        auto distance = site[i]->distanceTo(sn);
                        sum += distance;
                        num++;
                    }
                }
            }

            if (num == 0)
            {
                KIRI_LOG_ERROR("globalAvgDistance:: no neighbor site!!");
                return 0.0;
            }

            return sum / num;
        }

        Vector2D lineFitLeastSquares(std::vector<double> data)
        {
            double A = 0.0;
            double B = 0.0;
            double C = 0.0;
            double D = 0.0;

            for (int i = 0; i < data.size(); i++)
            {
                A += i * i;
                B += i;
                C += i * data[i];
                D += data[i];
            }

            double tmp = data.size() * A - B * B;

            double k = (data.size() * C - B * D) / (tmp + std::numeric_limits<double>::epsilon());
            double b = (A * D - B * C) / (tmp + std::numeric_limits<double>::epsilon());

            return Vector2D(k, b);
        }

        std::vector<Vector4D> computeMICBySSkel()
        {
            std::vector<Vector4D> circles;
            auto sites = mPowerDiagram->sites();

            for (int i = 0; i < sites.size(); i++)
            {
                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
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

        double computeMiniumPorosity()
        {
            auto maximum_circles = this->computeMICBySSkel();

            auto sum = 0.0;

#pragma omp parallel for reduction(+ \
                                   : sum)
            for (int i = 0; i < maximum_circles.size(); i++)
            {
                auto circle = maximum_circles[i];
                sum += circle.z * circle.z * kiri_math_mini::pi<double>();
            }

            auto boundary = mPowerDiagram->GetBoundary();

            // KIRI_LOG_DEBUG("boundary area ={0}, sum={1}, max circle size={2}", boundary->area(), sum, maximum_circles.size());
            return (boundary->area() - sum) / boundary->area();
        }

        bool removeNoiseVoroSites()
        {
            std::vector<int> remove_voro_idxs;
            auto sites = mPowerDiagram->sites();

            for (int i = 0; i < sites.size(); i++)
            {
                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                auto poly = site_i->cellPolygon();
                if (poly)
                {

                    if (poly->skeletons().empty())
                        poly->computeSSkel1998Convex();

                    auto mic_i = poly->computeMICByStraightSkeleton();
                    for (auto neighbor : site_i->neighbors())
                    {
                        auto site_j = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(neighbor);
                        if (site_j->isBoundaryVertex())
                            continue;

                        auto poly_j = site_j->cellPolygon();
                        if (poly_j)
                        {

                            if (poly_j->skeletons().empty())
                                poly_j->computeSSkel1998Convex();

                            auto micJ = poly_j->computeMICByStraightSkeleton();

                            auto dist_ij = (Vector2F(mic_i.x, mic_i.y) - Vector2F(micJ.x, micJ.y)).length();
                            // if ((dist_ij < ((mic_i.z + micJ.z) / 2.f)) &&
                            //     !std::binary_search(remove_voro_idxs.begin(), remove_voro_idxs.end(), site_i->id()) &&
                            //     !std::binary_search(remove_voro_idxs.begin(), remove_voro_idxs.end(), site_j->id()))
                            //     remove_voro_idxs.emplace_back(site_j->id());

                            if ((dist_ij < ((mic_i.z + micJ.z) / 2.f)))
                                remove_voro_idxs.emplace_back(site_j->id());
                        }
                        else
                        {
                            KIRI_LOG_ERROR("computeMICBySSkel: No Polygon Data!!!");
                            // remove.emplace_back(site_i->radius());
                        }
                    }
                }
                else
                {
                    KIRI_LOG_ERROR("computeMICBySSkel: No Polygon Data!!!");
                    // remove.emplace_back(site_i->radius());
                }
            }

            if (!remove_voro_idxs.empty())
            {
                KIRI_LOG_DEBUG("remove overlapping cell, size={0}", remove_voro_idxs.size());
                mPowerDiagram->removeVoroSitesByIndexArray(remove_voro_idxs);
                return true;
            }

            return false;
        }

        bool dynamicAddSites()
        {
            if (mPowerDiagram->sites().size() - 4 >= mMaxiumNum && bReachMaximumNum == false)
                bReachMaximumNum = true;

            if (bReachMaximumNum)
                return false;

            auto entity_num = 20;
            auto k_threshold = 200;

            std::random_device engine;
            std::mt19937 gen(engine());
            std::piecewise_constant_distribution<double> dist{std::begin(mUsrDefinedRadiusDist), std::end(mUsrDefinedRadiusDist), std::begin(mUsrDefinedRadiusDistProb)};

            if (mGlobalPorosityArray.size() > entity_num)
            {
                std::vector<Voronoi::VoronoiSite2Ptr> new_sites;
                std::vector<double> errorArray(mGlobalPorosityArray.end() - entity_num, mGlobalPorosityArray.end());
                auto line = lineFitLeastSquares(errorArray);
                // KIRI_LOG_DEBUG("line k={0}", std::abs(line.x));
                if (std::abs(line.x) < 1e-6)
                {
                    // KIRI_LOG_DEBUG("reach line res={0}", std::abs(line.x) < 1e-6f);

                    if (!bReachMaximumNum)
                    {
                        auto voroSite = mPowerDiagram->sites();
                        for (int i = 0; i < voroSite.size() - 4; i++)
                        {
                            auto pos = mPowerDiagram->GetBoundary()->rndInnerPoint();
                            auto new_site = std::make_shared<Voronoi::VoronoiSite2>(pos.x, pos.y, mSiteCounter++);

                            new_site->setRadius(dist(gen));
                            new_sites.emplace_back(new_site);
                            // KIRI_LOG_DEBUG("new site={0},{1};radius={2};id={3}", pos.x, pos.y, new_site->radius(), mSiteCounter - 1);
                        }
                    }
                }

                auto new_vorosite_num = new_sites.size();
                auto cur_vorosite_num = mPowerDiagram->sites().size() - 4;
                auto need_append_vorosite_num = new_vorosite_num;

                if (!bReachMaximumNum)
                {
                    if ((new_vorosite_num + cur_vorosite_num) > mMaxiumNum)
                    {
                        bReachMaximumNum = true;
                        need_append_vorosite_num = mMaxiumNum - cur_vorosite_num;
                    }

                    for (int i = 0; i < need_append_vorosite_num; i++)
                        mPowerDiagram->addSite(new_sites[i]);
                }

                if (!new_sites.empty())
                    return true;
            }

            return false;
        }

        void computeWeightsError()
        {
            mCurGlobalWeightError = 0.0;
            auto site = mPowerDiagram->sites();
            mWeightError.assign(site.size(), 0.0);
            mWeightAbsError.assign(site.size(), 0.0);

#pragma omp parallel for
            for (int i = 0; i < site.size(); i++)
            {
                if (site[i]->isBoundaryVertex())
                    continue;

                auto total = 0.0;
                auto cnt = 0;
                auto site_i = site[i];

                if (!site_i->neighbors().empty())
                {
                    auto radius_i = site_i->radius();

                    for (auto neighbor : site_i->neighbors())
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex2>(neighbor);
                        total += sn->weight() - sn->radius() * sn->radius();
                        cnt++;
                    }

                    auto avg = total / cnt;

                    for (auto neighbor : site_i->neighbors())
                    {
                        auto sn = std::dynamic_pointer_cast<Primitives::Vertex2>(neighbor);
                        auto distance = site[i]->distanceTo(sn);
                        auto minW = std::abs(std::sqrt(sn->weight()) - std::sqrt(site_i->weight()));
                        auto maxW = std::sqrt(sn->weight()) + std::sqrt(site_i->weight());
                        if (distance < maxW && distance > minW)
                        {
                            auto sum = sn->weight() - sn->radius() * sn->radius();
                            mWeightError[i] += avg - sum;
                            mWeightAbsError[i] += std::abs(avg - sum);
                            mCurGlobalWeightError += std::abs(avg - sum);
                        }
                        else
                        {
                            auto pw = distance * distance - site_i->weight();
                            mWeightError[i] += pw;
                            mWeightAbsError[i] += std::abs(pw);
                            mCurGlobalWeightError += std::abs(pw);
                        }
                    }
                }
            }
        }

        void adaptWeights()
        {

            auto g_area_error = globalAreaError();
            auto g_avg_distance = globalAvgDistance();

            auto gamma_area = 1.0;
            auto gamma_bc = 1.0;

            auto sites = mPowerDiagram->sites();

#pragma omp parallel for
            for (int i = 0; i < sites.size(); i++)
            {

                auto site_i = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
                if (site_i->isBoundaryVertex())
                    continue;

                auto weight = site_i->weight();

                auto weight_area = 0.0;
                auto weight_bc = 0.0;

                auto n = site_i->neighbors().size();
                if (n > 2)
                {
                    auto current_area = (!site_i->cellPolygon()) ? 0.0 : site_i->cellPolygon()->area();
                    auto target_area = n * site_i->radius() * site_i->radius() * std::tan(kiri_math_mini::pi<double>() / n);
                    // auto target_area = kiri_math_mini::pi<double>() * site_i->radius() * site_i->radius();

                    auto area_percentage = 2.0;
                    if (current_area != 0.0)
                        area_percentage = target_area / current_area;

                    auto area_error_transform = (-(g_area_error - 1.0) * (g_area_error - 1.0) + 1.0);
                    auto area_step = g_avg_distance * area_error_transform * gamma_area;
                    if (area_percentage < (1.0 - std::numeric_limits<double>::epsilon()) && weight > 0.0)
                        weight_area -= area_step;
                    else if (area_percentage > (1.0 + std::numeric_limits<double>::epsilon()))
                        weight_area += area_step;
                }

                auto error = mWeightAbsError[i] / (mCurGlobalWeightError + std::numeric_limits<double>::epsilon());
                auto error_transform = (-(error - 1.0) * (error - 1.0) + 1.0);

                auto step = error_transform * gamma_bc;
                if (mWeightError[i] < 0.0)
                    weight_bc -= step;
                else if (mWeightError[i] > 0.0)
                    weight_bc += step;

                site_i->setWeight(weight + weight_area + weight_bc);
            }
        }

    private:
        int mSiteCounter = 0;
        int mCurIteration = 0;

        double mCompleteArea = 0.0;
        double mCurGlobalPorosity = 0.0;
        double mCurGlobalWeightError = 0.0;

        int mMaxiumNum = 1000;
        bool bReachMaximumNum = false;
        double mLastMP = 1.0;

        std::vector<double> mUsrDefinedRadiusDist;
        std::vector<double> mUsrDefinedRadiusDistProb;

        std::vector<double>
            mWeightError,
            mWeightAbsError,
            mGlobalErrorArray,
            mGlobalPorosityArray;

        Voronoi::PowerDiagram2Ptr mPowerDiagram;
    };

} // namespace HDV::Sampler

#endif /* _HDV_MS_SAMPLER2_H_ */