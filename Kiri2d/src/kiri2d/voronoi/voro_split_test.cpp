/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-08-02 17:47:15
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_split_test.cpp
 */

#include <kiri2d/voronoi/voro_split_test.h>
#include <random>
namespace KIRI
{
    Vector<Vector4F> KiriVoroSplit::GetMICBySSkel()
    {
        Vector<Vector4F> circles;
        auto sites = this->GetVoroSites();

        Vector<UInt> removeVoroIdxs;
        for (size_t i = 0; i < sites.size(); i++)
        {
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                if (poly->GetSkeletons().empty())
                {

                    poly->ComputeSSkel1998Convex();
                }

                auto mic = poly->ComputeMICByStraightSkeleton();
                circles.emplace_back(Vector4F(mic, sites[i]->GetRadius()));
            }
            else
            {
                removeVoroIdxs.emplace_back(sites[i]->GetIdx());
            }
        }

        if (!removeVoroIdxs.empty())
            mPowerDiagram->RemoveVoroSitesByIndexArray(removeVoroIdxs);

        return circles;
    }

    void KiriVoroSplit::Init()
    {
        mPowerDiagram->ComputeDiagram();
        ComputeGroup();
    }

    void KiriVoroSplit::ComputeGroup()
    {

        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto voroSite = mPowerDiagram->GetVoroSites();

        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto vi = std::dynamic_pointer_cast<KiriVoroGroupSite>(voroSite[i]);
            if (vi->GetIsGroup())
                continue;

            auto group_col = Vector3F(dist(rndEngine), dist(rndEngine), dist(rndEngine));
            vi->SetGroupId(mGroupCounter);
            vi->SetGroupColor(group_col);

            auto neighbors = voroSite[i]->GetNeighborSites();
            if (neighbors.size() < 2)
                continue;

            auto tmp_num = 0;
            for (size_t j = 0; j < neighbors.size(); j++)
            {
                auto nj = std::dynamic_pointer_cast<KiriVoroGroupSite>(neighbors[j]);
                if (nj->GetIsGroup())
                    continue;

                nj->SetGroupId(mGroupCounter);
                nj->SetGroupColor(group_col);
                tmp_num++;

                if (tmp_num == 2)
                    break;
            }

            if (tmp_num < 2)
            {
                vi->DisGroup();
                for (size_t j = 0; j < neighbors.size(); j++)
                {
                    auto nj = std::dynamic_pointer_cast<KiriVoroGroupSite>(neighbors[j]);

                    if (nj->GetIsGroup() && nj->GetGroupId() == mGroupCounter)
                        nj->DisGroup();
                }
            }
            else
                mGroupCounter++;
        }
    }

}