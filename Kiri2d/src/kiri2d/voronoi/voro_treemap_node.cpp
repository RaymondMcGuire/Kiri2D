/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-03 00:00:17
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_treemap_node.cpp
 */

#include <kiri2d/voronoi/voro_treemap_node.h>
#include <random>
namespace KIRI
{

    void KiriVoroTreeMapNode::InitCore()
    {
        for (size_t i = 0; i < mChildNodes.size(); i++)
            mCore->addSite(mChildNodes[i]->GetSite());

        mCore->SetBoundaryPolygon2(mSite->GetCellPolygon());
        mCore->init();
    }

    void KiriVoroTreeMapNode::ComputeIterate()
    {
        // re calc child node pos

        mCore->SetBoundaryPolygon2(mSite->GetCellPolygon());

        mCore->init();

        mCore->ComputeIterate();
    }

    void KiriVoroTreeMapNode::AddChildNodes()
    {
        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto childcnt = static_cast<UInt>(dist(rndEngine) * 50) + 2;
        for (size_t i = 0; i < childcnt; i++)
        {
            auto cpos = mSite->GetCellPolygon()->rndInnerPoint();
            auto cnode = std::make_shared<KiriVoroTreeMapNode>(mId * 10, mDepth + 1, cpos, 0.1f);
            mChildNodes.emplace_back(cnode);
        }
    }

}