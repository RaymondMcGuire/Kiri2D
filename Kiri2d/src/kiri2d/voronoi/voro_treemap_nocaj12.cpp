/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-07 18:18:33
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_treemap_nocaj12.cpp
 */

#include <kiri2d/voronoi/voro_treemap_nocaj12.h>
#include <random>
namespace KIRI
{
    void KiriVoroTreeMapNocaj12::SetRootBoundary2(const Vector<Vector2F> &boundary)
    {
        for (size_t i = 0; i < boundary.size(); i++)
            mRootBoundary->AddPolygonVertex2(boundary[i]);
    }

    void KiriVoroTreeMapNocaj12::GenExample(float width, float height)
    {
        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto cnt = 0, maxcnt = 10;
        while (cnt < maxcnt)
        {
            auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
            if (mRootBoundary->Contains(sitePos2))
            {
                auto node = std::make_shared<KiriVoroTreeMapNode>(cnt, 1, sitePos2, dist(rndEngine));
                mNodes.emplace_back(node);
                cnt++;
            }
        }

        for (size_t i = 0; i < mNodes.size(); i++)
            mRootCore->AddSite(mNodes[i]->GetSite());

        mRootCore->SetBoundaryPolygon2(mRootBoundary);
        mRootCore->Init();
        mRootCore->ComputeIterate();

        for (size_t i = 0; i < mNodes.size(); i++)
        {
            //if (i != mNodes.size() / 2)
            mNodes[i]->AddChildNodes();

            mNodes[i]->InitCore();
        }
    }

    void KiriVoroTreeMapNocaj12::ComputeIterate(const Vector<KiriVoroTreeMapNodePtr> &child)
    {
        if (!child.empty())
            for (size_t i = 0; i < child.size(); i++)
            {
                if (!child[i]->GetChildSites().empty())
                {
                    child[i]->ComputeIterate();
                    ComputeIterate(child[i]->GetChildNodes());
                }
            }
    }

    void KiriVoroTreeMapNocaj12::ComputeIterate()
    {
        mRootCore->ComputeIterate();

        ComputeIterate(mNodes);
    }

}