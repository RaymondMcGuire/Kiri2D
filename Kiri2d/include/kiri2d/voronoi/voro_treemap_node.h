/***
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-02 21:06:22
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_treemap_node.h
 */

#ifndef _KIRI_VORO_TREEMAP_NODE_H_
#define _KIRI_VORO_TREEMAP_NODE_H_

#pragma once

#include <kiri2d/voronoi/voro_treemap_core.h>

namespace KIRI
{

    class KiriVoroTreeMapNode
    {
    public:
        explicit KiriVoroTreeMapNode(UInt id, UInt depth, Vector2F pos, float percent)
            : mId(id), mDepth(depth)
        {
            mCore = std::make_shared<KiriVoroTreeMapCore>();
            mSite = std::make_shared<KiriVoroSite>(pos, percent);
        }

        ~KiriVoroTreeMapNode() {}

        void InitCore();
        const KiriVoroSitePtr &GetSite() const { return mSite; }

        const Vector<KiriVoroSitePtr> &GetChildSites() const { return mCore->GetSites(); }
        const Vector<SharedPtr<KiriVoroTreeMapNode>> &GetChildNodes() const { return mChildNodes; }

        bool IsNoChildNode() const { return (mChildNodes.empty() ? true : false); }

        void ComputeIterate();
        void AddChildNodes();

    private:
        UInt mId, mDepth;
        KiriVoroSitePtr mSite;
        KiriVoroTreeMapCorePtr mCore;
        Vector<SharedPtr<KiriVoroTreeMapNode>> mChildNodes;
    };

    typedef SharedPtr<KiriVoroTreeMapNode> KiriVoroTreeMapNodePtr;
}
#endif