/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-24 17:27:16
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_treemap.cpp
 */

#include <kiri2d/voronoi/voro_poropti_treemap.h>
namespace KIRI
{
    void KiriVoroPoroOptiTreeMap::InitTreeMapNodes()
    {
        mMaxDepth = mTreeMapData->GetMaxDepth();

        for (size_t i = 0; i < mTreeMapData->GetTotalNodeNum(); i++)
        {
            auto node = mTreeMapData->GetTreeMapNodeById(i);
            auto site = std::make_shared<KiriVoroSite>(Vector2F(0.f), node.weight);
            auto tnode = std::make_shared<KiriVoroPoroOptiTreeMapNode>(node.id, node.depth, site);
            mNodes.emplace_back(tnode);
        }

        for (size_t i = mMaxDepth; i > 0; i--)
        {
            auto nodes = mTreeMapData->GetNodesByDepth(i);
            for (size_t j = 0; j < nodes.size(); j++)
                mNodes[nodes[j].pid]->AddChildNode(mNodes[nodes[j].id]);
        }

        auto rootNode = mTreeMapData->GetRootNode();
        for (size_t i = 0; i < mMaxDepth; i++)
        {
            auto nodes = mTreeMapData->GetNodesByDepth(i);
            for (size_t j = 0; j < nodes.size(); j++)
            {
                auto nodeId = nodes[j].id;
                // setup boundary
                if (mNodes[nodeId]->GetNodeId() == rootNode.id)
                {
                    mNodes[nodeId]->SetSiteBoundary(mBoundary);
                }

                if (!mNodes[nodeId]->GetChildNodes().empty())
                {
                    // setup child nodes pos
                    for (size_t k = 0; k < mNodes[nodeId]->GetChildNodes().size(); k++)
                    {
                        auto cnode = mNodes[nodeId]->GetChildNodes()[k];
                        cnode->SetSitePos(mNodes[nodeId]->GetSiteBoundary()->rndInnerPoint());
                    }

                    mNodes[nodeId]->InitCore();
                }
            }
        }
    }

    void KiriVoroPoroOptiTreeMap::ComputeIterate()
    {
        for (size_t i = 0; i < mMaxDepth; i++)
        {
            auto nodes = mTreeMapData->GetNodesByDepth(i);
            for (size_t j = 0; j < nodes.size(); j++)
            {
                auto nodeId = nodes[j].id;
                if (!mNodes[nodeId]->GetChildNodes().empty())
                    mNodes[nodeId]->ComputeIterate();
            }
        }
    }
}