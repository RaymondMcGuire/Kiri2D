/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-24 18:19:24
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_poropti_treemap.h
 */

#ifndef _KIRI_VORO_PORO_OPTI_TREEMAP_H_
#define _KIRI_VORO_PORO_OPTI_TREEMAP_H_

#pragma once

#include <kiri2d/voronoi/voro_poropti_treemap_node.h>

namespace KIRI
{

    class KiriVoroPoroOptiTreeMap
    {
    public:
        explicit KiriVoroPoroOptiTreeMap(
            const KiriVoroTreeMapDataPtr &data,
            const KiriVoroCellPolygon2Ptr &boundary)
        {
            mTreeMapData = data;
            mBoundary = boundary;
        }

        ~KiriVoroPoroOptiTreeMap() noexcept {}

        void InitTreeMapNodes();

        void ComputeIterate();

        Vector<KiriVoroSitePtr> GetLeafNodeSites()
        {
            Vector<KiriVoroSitePtr> sites;
            for (size_t i = 0; i < mNodes.size(); i++)
                if (mNodes[i]->IsNoChildNode())
                    sites.emplace_back(mNodes[i]->GetSite());

            return sites;
        }

        Vector<KiriVoroSitePtr> GetNodeSitesByConstrain()
        {
            Vector<KiriVoroSitePtr> sites;
            Vector<UInt> sitesId;

            for (size_t i = 0; i <= mMaxDepth; i++)
            {
                auto nodes = mTreeMapData->GetNodesByDepth(i);
                for (size_t j = 0; j < nodes.size(); j++)
                {
                    auto nodeId = nodes[j].id;
                    auto cnodeNum = nodes[j].child_num;

                    if (cnodeNum < 5 && !(std::find(sitesId.begin(), sitesId.end(), nodes[j].id) != sitesId.end()))
                    {

                        sites.emplace_back(mNodes[nodeId]->GetSite());

                        auto addId = mNodes[nodeId]->GetAllNodesId();
                        sitesId.insert(sitesId.end(), addId.begin(), addId.end());
                    }
                }
            }

            return sites;
        }

    private:
        UInt mMaxDepth = 0;

        KiriVoroTreeMapDataPtr mTreeMapData;
        KiriVoroCellPolygon2Ptr mBoundary;

        Vector<KiriVoroPoroOptiTreeMapNodePtr> mNodes;
    };

    typedef SharedPtr<KiriVoroPoroOptiTreeMap> KiriVoroPoroOptiTreeMapPtr;
}
#endif