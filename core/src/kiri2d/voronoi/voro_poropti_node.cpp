/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-10 21:50:08
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti_node.cpp
 */

#include <kiri2d/voronoi/voro_poropti_node.h>
#include <random>
namespace KIRI
{

    void KiriVoroPoroOptiNode::InitCore()
    {
        InitChildSites();

        mCore->SetBoundaryPolygon2(mSite->GetCellPolygon());
        mCore->init();
    }

    void KiriVoroPoroOptiNode::ComputeIterate()
    {
        // re calc child node pos
        // InitChildSites();
        mCore->SetBoundaryPolygon2(mSite->GetCellPolygon());

        mCore->init();

        // mCore->ComputeDiagram();
        mCore->ComputeLloyd(10);
    }

    void KiriVoroPoroOptiNode::InitChildSites()
    {
        std::random_device seed;
        std::default_random_engine engine(seed());
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto childcnt = 5;
        for (size_t i = 0; i < childcnt; i++)
        {
            auto cpos = mSite->GetCellPolygon()->rndInnerPoint();
            mCore->addSite(std::make_shared<KiriVoroSite>(cpos));
        }
    }

}