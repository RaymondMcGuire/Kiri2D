/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-08-02 13:35:45
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_poropti.h
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
        explicit KiriVoroPoroOpti(
            float targetPorosity = 0.f)
            : mTargetPorosity(targetPorosity)
        {
            mRootCore = std::make_shared<KiriVoroPoroOptiCore>(targetPorosity);
            mRootBoundary = std::make_shared<KiriVoroCellPolygon2>();
        }

        ~KiriVoroPoroOpti() noexcept {}

        void SetRootBoundary2(const Vector<Vector2F> &boundary);

        void GenExample(float width, float height);

        float ComputeIterate();
        void ComputeChildIterate();

        const KiriVoroPoroOptiCorePtr &GetRootCore() const { return mRootCore; }
        const KiriVoroCellPolygon2Ptr &GetRootBoundary() const { return mRootBoundary; }
        float GetMiniumPorosity()
        {
            if (mTargetPorosity == 0.f)
                return mRootCore->ComputeMiniumPorosity();
            else
                return mRootCore->GetMiniumPorosity();
        }
        Vector<Vector4F> GetCellSSkel() { return mRootCore->GetCellSSkel(); }
        Vector<Vector4F> GetMICBySSkel() { return mRootCore->GetMICBySSkel(); }

        Vector<KiriVoroSitePtr> GetLeafNodeSites()
        {
            Vector<KiriVoroSitePtr> sites;
            for (size_t i = 0; i < mRootCore->GetSites().size(); i++)
                sites.emplace_back(mRootCore->GetSites()[i]);

            return sites;
        }

        Vector<Vector4F> GetLeafNodeMaxInscribedCircle()
        {
            Vector<Vector4F> circles;
            for (size_t i = 0; i < mNodes.size(); i++)
                circles.emplace_back(mNodes[i]->ComputeMaxInscribedCircle());

            return circles;
        }

        Vector<Vector3F> GetVoronoiSitesData()
        {
            Vector<Vector3F> sites_data;
            auto sites = mRootCore->GetSites();
            for (size_t i = 0; i < sites.size(); i++)
                sites_data.emplace_back(Vector3F(sites[i]->GetValue().x, sites[i]->GetValue().y, sites[i]->GetWeight()));

            return sites_data;
        }

    private:
        float mTargetPorosity;
        UInt mMaxDepth = 0;
        KiriVoroCellPolygon2Ptr mRootBoundary;
        KiriVoroPoroOptiCorePtr mRootCore;
        Vector<KiriVoroPoroOptiNodePtr> mNodes;
    };

    typedef SharedPtr<KiriVoroPoroOpti> KiriVoroPoroOptiPtr;
}
#endif