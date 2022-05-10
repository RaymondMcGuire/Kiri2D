/***
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-02 21:13:29
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_treemap_nocaj12.h
 */

#ifndef _KIRI_VORO_TREEMAP_NOCAJ12_H_
#define _KIRI_VORO_TREEMAP_NOCAJ12_H_

#pragma once

#include <kiri2d/voronoi/voro_treemap_node.h>

namespace KIRI
{

    class KiriVoroTreeMapNocaj12
    {
    public:
        explicit KiriVoroTreeMapNocaj12()
        {
            mRootCore = std::make_shared<KiriVoroTreeMapCore>();
            mRootBoundary = std::make_shared<KiriVoroCellPolygon2>();
        }

        ~KiriVoroTreeMapNocaj12() {}

        void SetRootBoundary2(const Vector<Vector2F> &boundary);

        void GenExample(float width, float height);

        void ComputeIterate();

        const KiriVoroTreeMapCorePtr &GetRootCore() const { return mRootCore; }
        const KiriVoroCellPolygon2Ptr &GetRootBoundary() const { return mRootBoundary; }

        void MergeLeafNodeVoroSites(Vector<KiriVoroSitePtr> &sites, const Vector<KiriVoroTreeMapNodePtr> &child)
        {
            if (!child.empty())
            {
                for (size_t i = 0; i < child.size(); i++)
                {
                    auto csites = child[i]->GetChildSites();
                    for (size_t j = 0; j < csites.size(); j++)
                    {
                        if (child[i]->GetChildNodes()[j]->IsNoChildNode())
                            sites.emplace_back(csites[j]);
                    }
                    MergeLeafNodeVoroSites(sites, child[i]->GetChildNodes());
                }
            }
        }

        Vector<KiriVoroSitePtr> GetLeafNodeSites()
        {
            Vector<KiriVoroSitePtr> sites;
            for (size_t i = 0; i < mRootCore->sites().size(); i++)
                if (mNodes[i]->IsNoChildNode())
                    sites.emplace_back(mRootCore->sites()[i]);

            MergeLeafNodeVoroSites(sites, mNodes);
            return sites;
        }

    private:
        UInt mMaxDepth = 0;
        KiriVoroCellPolygon2Ptr mRootBoundary;
        KiriVoroTreeMapCorePtr mRootCore;
        Vector<KiriVoroTreeMapNodePtr> mNodes;

        void ComputeIterate(const Vector<KiriVoroTreeMapNodePtr> &child);
    };

    typedef SharedPtr<KiriVoroTreeMapNocaj12> KiriVoroTreeMapNocaj12Ptr;
}
#endif