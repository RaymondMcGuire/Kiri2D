/***
 * @Author: Xu.WANG
 * @Date: 2021-10-05 00:08:58
 * @LastEditTime: 2021-11-18 19:39:06
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _KIRI_VORO_POROPTI_H_
#define _KIRI_VORO_POROPTI_H_

#pragma once

#include <kiri2d/voronoi/voro_poropti_node.h>

namespace KIRI
{

    class KiriVoroPoroOpti
    {
    public:
        explicit KiriVoroPoroOpti()
        {
            mRootCore = std::make_shared<KiriVoroPoroOptiCore>();
            mRootBoundary = std::make_shared<KiriVoroCellPolygon2>();
        }

        ~KiriVoroPoroOpti() {}

        void SetRootBoundary2(const Vector<Vector2F> &boundary);

        void GenExample(float width, float height);

        float ComputeIterate();
        void ComputeChildIterate();

        void SetMaxIterationNum(UInt num) { mRootCore->SetMaxIterationNum(num); }

        const KiriVoroPoroOptiCorePtr &GetRootCore() const { return mRootCore; }
        const KiriVoroCellPolygon2Ptr &GetRootBoundary() const { return mRootBoundary; }
        float GetMiniumPorosity()
        {
            auto porosity = mRootCore->GetCurGlobalPorosity();
            if (porosity != 0.f)
                return porosity;
            else
                return mRootCore->computeMiniumPorosity();
        }
        Vector<Vector4F> GetCellSSkel() { return mRootCore->GetCellSSkel(); }
        Vector<Vector4F> computeMICBySSkel() { return mRootCore->computeMICBySSkel(); }

        Vector<KiriVoroSitePtr> GetLeafNodeSites()
        {
            Vector<KiriVoroSitePtr> sites;
            for (size_t i = 0; i < mRootCore->sites().size(); i++)
                sites.emplace_back(mRootCore->sites()[i]);

            return sites;
        }

        Vector<Vector4F> GetLeafNodeMaxInscribedCircle()
        {
            Vector<Vector4F> circles;
            for (size_t i = 0; i < mNodes.size(); i++)
                circles.emplace_back(mNodes[i]->ComputeMaxInscribedCircle());

            return circles;
        }

        Vector<Vector4F> GetVoronoiSitesData()
        {
            Vector<Vector4F> sites_data;
            auto sites = mRootCore->sites();
            for (size_t i = 0; i < sites.size(); i++)
                sites_data.emplace_back(Vector4F(sites[i]->GetValue().x, sites[i]->GetValue().y, sites[i]->weight(), sites[i]->radius()));

            return sites_data;
        }

    private:
        UInt mMaxDepth = 0;
        KiriVoroCellPolygon2Ptr mRootBoundary;
        KiriVoroPoroOptiCorePtr mRootCore;
        Vector<KiriVoroPoroOptiNodePtr> mNodes;
    };

    typedef SharedPtr<KiriVoroPoroOpti> KiriVoroPoroOptiPtr;
}
#endif