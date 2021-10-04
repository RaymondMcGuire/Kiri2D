/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-10-04 11:16:26
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_poropti.cpp
 */

#include <kiri2d/voronoi/voro_poropti.h>
#include <random>
namespace KIRI
{
    void KiriVoroPoroOpti::SetRootBoundary2(const Vector<Vector2F> &boundary)
    {
        for (size_t i = 0; i < boundary.size(); i++)
            mRootBoundary->AddPolygonVertex2(boundary[i]);
    }

    void KiriVoroPoroOpti::GenExample(float width, float height)
    {
        KIRI_LOG_DEBUG("KiriVoroPoroOpti: Start GenExample");

        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<float> dist(0.f, 1.f);
        std::uniform_real_distribution<float> rdist(-1.f, 1.f);

        std::vector<float> radiusRange;
        radiusRange.push_back(20.f);
        radiusRange.push_back(30.f);
        radiusRange.push_back(80.f);
        radiusRange.push_back(150.f);

        std::vector<float> radiusRangeProb;
        radiusRangeProb.push_back(0.5f);
        radiusRangeProb.push_back(0.4f);
        radiusRangeProb.push_back(0.1f);

        std::random_device engine;
        std::mt19937 gen(engine());
        std::piecewise_constant_distribution<float> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

        auto cnt = 0, maxcnt = 100;
        while (cnt < maxcnt)
        {
            auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);

            if (mRootBoundary->Contains(sitePos2))
            {
                //auto radius = avgRadius / 2.f * rdist(rndEngine) + avgRadius;
                auto radius = pcdis(gen);
                auto node = std::make_shared<KiriVoroPoroOptiNode>(sitePos2, radius);
                mNodes.emplace_back(node);
                cnt++;
            }
        }

        for (size_t i = 0; i < mNodes.size(); i++)
            mRootCore->AddSite(mNodes[i]->GetSite());

        mRootCore->SetBoundaryPolygon2(mRootBoundary);
        mRootCore->Init();
        mRootCore->ComputeIterate();
    }

    float KiriVoroPoroOpti::ComputeIterate()
    {
        return mRootCore->ComputeIterate();
    }

    void KiriVoroPoroOpti::ComputeChildIterate()
    {
        auto sites = mRootCore->GetSites();
        for (size_t i = 0; i < mNodes.size(); i++)
        {
            mNodes[i]->SetSite(sites[i]);
            mNodes[i]->ComputeIterate();
        }
    }

}