/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-07 18:09:59
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 18:36:37
 * @FilePath: \Kiri2D\core\include\kiri2d\hdv_toolkit\voronoi\voronoi_nocaj_treemap.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _HDV_VORONOI_NOCAJ_TREEMAP_H_
#define _HDV_VORONOI_NOCAJ_TREEMAP_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_nocaj_core.h>

namespace HDV::VoronoiTreeMap
{

    class VoronoiNocajTreeMapNode
    {
    public:
        explicit VoronoiNocajTreeMapNode(UInt id, UInt depth, Vector2D pos, double percent)
            : mId(id), mDepth(depth)
        {
            mCore = std::make_shared<VoronoiNocajCore>();
            mSite = std::make_shared<Voronoi::VoronoiSite2>(pos.x, pos.y, id);
            mSite->setPercentage(percent);
        }

        virtual ~VoronoiNocajTreeMapNode() {}

        Voronoi::VoronoiSite2Ptr site() { return mSite; }

        std::vector<Primitives::Vertex2Ptr> childSites() const { return mCore->sites(); }
        Vector<SharedPtr<VoronoiNocajTreeMapNode>> childNodes() const { return mChildNodes; }

        bool isNoChildNode() const { return (mChildNodes.empty() ? true : false); }

        void initCore()
        {

            // KIRI_LOG_DEBUG("init Core");
            mCore->setBoundary(mSite->polygon());
            for (auto i = 0; i < mChildNodes.size(); i++)
                mCore->addSite(mChildNodes[i]->site());

            mCore->init();

            // for (size_t i = 0; i < mCore->sites().size(); i++)
            // {
            //     KIRI_LOG_DEBUG("site id={0}, isB={1}", mCore->sites()[i]->id(), mCore->sites()[i]->isBoundaryVertex());
            // }

            mCore->computeIterate();
        }

        void computeIterate()
        {
            mCore->setBoundary(mSite->polygon());

            mCore->init();

            mCore->computeIterate();
        }

        void addRndChildNodes()
        {
            auto childcnt = static_cast<UInt>(Random::get(0.0, 1.0) * 10) + 4;
            for (auto i = 0; i < childcnt; i++)
            {
                auto cpos = mSite->polygon()->rndInnerPoint();
                auto cnode = std::make_shared<VoronoiNocajTreeMapNode>(mId * 100 + i, mDepth + 1, cpos, Random::get(0.1, 1.0));
                mChildNodes.emplace_back(cnode);

                // KIRI_LOG_DEBUG("site id={0}", cnode->site()->id());
            }

            // KIRI_LOG_DEBUG("child node cnt = {0}", mChildNodes.size());
        }

    private:
        UInt mId, mDepth;
        Voronoi::VoronoiSite2Ptr mSite;
        VoronoiNocajCorePtr mCore;
        Vector<SharedPtr<VoronoiNocajTreeMapNode>> mChildNodes;
    };
    typedef SharedPtr<VoronoiNocajTreeMapNode> VoronoiNocajTreeMapNodePtr;

    class VoronoiNocajTreeMap
    {
    public:
        explicit VoronoiNocajTreeMap()
        {
            mRootCore = std::make_shared<VoronoiNocajCore>();
        }

        virtual ~VoronoiNocajTreeMap() {}

        void setRootBoundary(const std::shared_ptr<Voronoi::VoronoiPolygon2> &boundary)
        {
            mRootCore->setBoundary(boundary);
        }

        const VoronoiNocajCorePtr &rootCore() const { return mRootCore; }
        const std::shared_ptr<Voronoi::VoronoiPolygon2> &rootBoundary() const { return mRootCore->boundary(); }

        void GenExample()
        {
            auto boundary = rootBoundary();

            for (auto i = 0; i < 10; i++)
            {
                auto site_pos = boundary->rndInnerPoint();
                auto node = std::make_shared<VoronoiNocajTreeMapNode>(i, 1, site_pos, Random::get(0.1, 1.0));
                mNodes.emplace_back(node);
            }

            for (auto i = 0; i < mNodes.size(); i++)
                mRootCore->addSite(mNodes[i]->site());

            mRootCore->init();
            mRootCore->computeIterate();

            for (auto i = 0; i < mNodes.size(); i += 3)
            {
                mNodes[i]->addRndChildNodes();
                mNodes[i]->initCore();
            }
        }

        void mergeLeafNodeVoroSites(Vector<Voronoi::VoronoiSite2Ptr> &sites, const Vector<VoronoiNocajTreeMapNodePtr> &child)
        {
            if (!child.empty())
            {
                for (auto i = 0; i < child.size(); i++)
                {
                    auto child_nodes = child[i]->childNodes();
                    for (auto j = 0; j < child_nodes.size(); j++)
                    {
                        if (child_nodes[j]->isNoChildNode())
                            sites.emplace_back(child_nodes[j]->site());
                    }
                    mergeLeafNodeVoroSites(sites, child_nodes);
                }
            }
        }

        Vector<Voronoi::VoronoiSite2Ptr> leafNodeSites()
        {
            Vector<Voronoi::VoronoiSite2Ptr> sites;
            // KIRI_LOG_DEBUG("Root Core Sites Num={0}", mNodes.size());
            for (size_t i = 0; i < mNodes.size(); i++)
            {
                if (mNodes[i]->isNoChildNode())
                    sites.emplace_back(std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(mNodes[i]->site()));
            }

            mergeLeafNodeVoroSites(sites, mNodes);
            return sites;
        }

        void computeIterate()
        {
            mRootCore->computeIterate();

            computeIterate(mNodes);
        }

    private:
        UInt mMaxDepth = 0;
        VoronoiNocajCorePtr mRootCore;
        Vector<VoronoiNocajTreeMapNodePtr> mNodes;

        void computeIterate(Vector<VoronoiNocajTreeMapNodePtr> child)
        {
            if (!child.empty())
                for (auto i = 0; i < child.size(); i++)
                {
                    if (!child[i]->childSites().empty())
                    {
                        child[i]->computeIterate();
                        computeIterate(child[i]->childNodes());
                    }
                }
        }
    };

    typedef SharedPtr<VoronoiNocajTreeMap> KiriVoroTreeMapNocaj12Ptr;

}
#endif