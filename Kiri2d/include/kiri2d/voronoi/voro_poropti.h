/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-07-23 17:09:45
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

        Vector<Vector4F> GetMICByStraightSkeleton(float lambda)
        {
            Vector<Vector4F> circles;
            auto sites = mRootCore->GetSites();

            Vector<UInt> removeVoroIdxs;
            for (size_t i = 0; i < sites.size(); i++)
            {
                auto poly = sites[i]->GetCellPolygon();
                if (poly != NULL)
                {
                    if (poly->GetSkeletons().empty())
                    {

                        poly->ComputeSSkel1998Convex();
                    }

                    auto mic = poly->ComputeMICByStraightSkeleton();
                    circles.emplace_back(Vector4F(mic, sites[i]->GetRadius()));
                }
                else
                {
                    removeVoroIdxs.emplace_back(sites[i]->GetIdx());
                }
            }

            if (!removeVoroIdxs.empty())
                mRootCore->RemoveVoroSitesByIndexArray(removeVoroIdxs);

            return circles;
        }

        Vector<Vector4F> GetCellSkeletons(float lambda)
        {
            Vector<Vector4F> skeletons;
            auto sites = mRootCore->GetSites();

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

        float ComputeMiniumPorosity(float lambda);

    private:
        UInt mMaxDepth = 0;
        KiriVoroCellPolygon2Ptr mRootBoundary;
        KiriVoroPoroOptiCorePtr mRootCore;
        Vector<KiriVoroPoroOptiNodePtr> mNodes;
    };

    typedef SharedPtr<KiriVoroPoroOpti> KiriVoroPoroOptiPtr;
}
#endif