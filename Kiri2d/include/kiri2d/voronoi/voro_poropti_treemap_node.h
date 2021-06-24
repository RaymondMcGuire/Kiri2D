/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-24 17:58:58
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_poropti_treemap_node.h
 */

#ifndef _KIRI_VORO_PORO_OPTI_TREEMAP_NODE_H_
#define _KIRI_VORO_PORO_OPTI_TREEMAP_NODE_H_

#pragma once

#include <kiri2d/voronoi/voro_treemap_data.h>
#include <kiri2d/voronoi/voro_poropti_treemap_core.h>

namespace KIRI
{

    class KiriVoroPoroOptiTreeMapNode
    {
    public:
        explicit KiriVoroPoroOptiTreeMapNode(UInt id, UInt depth, const KiriVoroSitePtr &site)
            : mId(id), mDepth(depth)
        {
            mCore = std::make_shared<KiriVoroPoroOptiTreeMapCore>();
            mSite = site;
        }

        ~KiriVoroPoroOptiTreeMapNode() noexcept {}

        void InitCore();
        const KiriVoroSitePtr &GetSite() const { return mSite; }

        const Vector<KiriVoroSitePtr> &GetChildSites() const { return mCore->GetSites(); }
        const Vector<SharedPtr<KiriVoroPoroOptiTreeMapNode>> &GetChildNodes() const { return mChildNodes; }

        void SetSiteBoundary(const KiriVoroCellPolygon2Ptr &boundary)
        {
            mSite->SetCellPolygon(boundary);
        }

        void SetSitePos(const Vector2F &pos)
        {
            mSite->ResetValue(pos);
        }

        const KiriVoroCellPolygon2Ptr &GetSiteBoundary() const { return mSite->GetCellPolygon(); }

        bool IsNoChildNode() const { return (mChildNodes.empty() ? true : false); }

        void ComputeIterate();
        void AddChildNodes(const Vector<VoroTreeMapNode> &childs);
        void AddChildNode(const SharedPtr<KiriVoroPoroOptiTreeMapNode> &child) { mChildNodes.emplace_back(child); };

        Vector<UInt> GetChildNodesId(Vector<SharedPtr<KiriVoroPoroOptiTreeMapNode>> childNodes)
        {
            Vector<UInt> id;
            for (size_t i = 0; i < childNodes.size(); i++)
            {
                id.emplace_back(childNodes[i]->GetNodeId());
                auto nid = GetChildNodesId(childNodes[i]->GetChildNodes());
                id.insert(id.end(), nid.begin(), nid.end());
            }
            return id;
        }

        Vector<UInt> GetAllNodesId()
        {
            return GetChildNodesId(mChildNodes);
        }

        constexpr UInt GetNodeId() const { return mId; }

    private:
        UInt mId, mDepth;
        KiriVoroSitePtr mSite;
        KiriVoroPoroOptiTreeMapCorePtr mCore;
        Vector<SharedPtr<KiriVoroPoroOptiTreeMapNode>> mChildNodes;
    };

    typedef SharedPtr<KiriVoroPoroOptiTreeMapNode> KiriVoroPoroOptiTreeMapNodePtr;
}
#endif