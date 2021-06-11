/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-10 22:49:05
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
        explicit KiriVoroPoroOpti()
        {
            mRootCore = std::make_shared<KiriVoroPoroOptiCore>();
            mRootBoundary = std::make_shared<KiriVoroCellPolygon2>();
        }

        ~KiriVoroPoroOpti() noexcept {}

        void SetRootBoundary2(const Vector<Vector2F> &boundary);

        void GenExample(float width, float height);

        float ComputeIterate();
        void ComputeChildIterate();

        const KiriVoroPoroOptiCorePtr &GetRootCore() const { return mRootCore; }
        const KiriVoroCellPolygon2Ptr &GetRootBoundary() const { return mRootBoundary; }

        Vector<KiriVoroSitePtr> GetLeafNodeSites()
        {
            Vector<KiriVoroSitePtr> sites;
            for (size_t i = 0; i < mRootCore->GetSites().size(); i++)
                sites.emplace_back(mRootCore->GetSites()[i]);

            // for (size_t i = 0; i < mNodes.size(); i++)
            // {
            //     auto nodeCore = mNodes[i]->GetCore();
            //     for (size_t j = 0; j < nodeCore->GetSites().size(); j++)
            //         sites.emplace_back(nodeCore->GetSites()[j]);
            // }

            return sites;
        }

        Vector<Vector4F> GetLeafNodeMaxInscribedCircle()
        {
            Vector<Vector4F> circles;
            for (size_t i = 0; i < mNodes.size(); i++)
                circles.emplace_back(mNodes[i]->ComputeMaxInscribedCircle());

            return circles;
        }

        float ComputeMiniumPorosity();

    private:
        UInt mMaxDepth = 0;
        KiriVoroCellPolygon2Ptr mRootBoundary;
        KiriVoroPoroOptiCorePtr mRootCore;
        Vector<KiriVoroPoroOptiNodePtr> mNodes;
    };

    typedef SharedPtr<KiriVoroPoroOpti> KiriVoroPoroOptiPtr;
}
#endif