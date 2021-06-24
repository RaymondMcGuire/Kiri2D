/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-24 15:58:06
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_treemap_node.cpp
 */

#include <kiri2d/voronoi/voro_poropti_treemap_node.h>
#include <random>
namespace KIRI
{

    void KiriVoroPoroOptiTreeMapNode::InitCore()
    {
        for (size_t i = 0; i < mChildNodes.size(); i++)
            mCore->AddSite(mChildNodes[i]->GetSite());

        mCore->SetBoundaryPolygon2(mSite->GetCellPolygon());
        mCore->Init();
    }

    void KiriVoroPoroOptiTreeMapNode::ComputeIterate()
    {
        // re calc child node pos

        mCore->SetBoundaryPolygon2(mSite->GetCellPolygon());

        mCore->Init();

        mCore->ComputeIterate();
    }

    void KiriVoroPoroOptiTreeMapNode::AddChildNodes(const Vector<VoroTreeMapNode> &childs)
    {
        for (size_t i = 0; i < childs.size(); i++)
        {
            auto pos = mSite->GetCellPolygon()->GetRndInnerPoint();
            auto site = std::make_shared<KiriVoroSite>(pos, childs[i].weight);
            auto node = std::make_shared<KiriVoroPoroOptiTreeMapNode>(childs[i].id, childs[i].depth, site);
            mChildNodes.emplace_back(node);
        }
    }

}