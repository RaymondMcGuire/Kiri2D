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

    Vec_Vec3F KiriVoroSplit::GetMIC(const KiriVoroSitePtr &site)
    {
        auto poly = site->GetCellPolygon();

        if (poly->GetSkeletons().empty())
        {
            poly->computeSSkel1998Convex();
        }

        return poly->ComputeMICByStraightSkeletonTest();
    }

    Vector<Vector4F> KiriVoroSplit::computeMICBySSkel()
    {
        auto sites = this->GetVoroSites();

        Vector<UInt> remove_voro_idxs;
        for (size_t i = 0; i < sites.size(); i++)
        {
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                if (poly->GetSkeletons().empty())
                {
                    poly->computeSSkel1998Convex();
                }

                auto mic = poly->computeMICByStraightSkeleton();
                mMIC.emplace_back(Vector4F(mic, sites[i]->radius()));
            }
            else
            {
                KIRI_LOG_DEBUG("error!!!");
            }
        }

        return mMIC;
    }

    void KiriVoroSplit::init()
    {
        mMIC.clear();
        mPowerDiagram->ComputeDiagram();
        // ComputeGroup();
    }

    void KiriVoroSplit::ComputeGroup()
    {

        std::random_device seed;
        std::default_random_engine engine(seed());
        std::uniform_real_distribution<float> dist(0.f, 1.f);

        auto voroSite = mPowerDiagram->GetVoroSites();

        for (size_t i = 0; i < voroSite.size(); i++)
        {
            auto vi = std::dynamic_pointer_cast<KiriVoroGroupSite>(voroSite[i]);
            if (vi->GetIsGroup())
                continue;

            auto neighbors = voroSite[i]->GetNeighborSites();
            if (neighbors.empty())
                continue;

            auto group_col = Vector3F(dist(engine), dist(engine), dist(engine));
            vi->SetGroupId(mGroupCounter);
            vi->SetGroupColor(group_col);

            auto mic = computeMICBySSkel();
            auto mic_i = mic[i];
            auto tmp_num = 0;
            for (size_t j = 0; j < neighbors.size(); j++)
            {

                auto micJ = mic[neighbors[j]->GetIdx()];
                // judge
                auto dist_ij = (Vector2F(mic_i.x, mic_i.y) - Vector2F(micJ.x, micJ.y)).length();
                if (dist_ij == (mic_i.z + micJ.z))
                {
                    auto nj = std::dynamic_pointer_cast<KiriVoroGroupSite>(neighbors[j]);
                    nj->SetGroupId(mGroupCounter);
                    nj->SetGroupColor(group_col);
                    tmp_num++;
                }

                if (tmp_num == 2)
                    break;
            }

            if (tmp_num == 0)
            {
                vi->DisGroup();
                // for (size_t j = 0; j < neighbors.size(); j++)
                // {
                //     auto nj = std::dynamic_pointer_cast<KiriVoroGroupSite>(neighbors[j]);

                //     if (nj->GetIsGroup() && nj->GetGroupId() == mGroupCounter)
                //         nj->DisGroup();
                // }
            }
            else
                mGroupCounter++;
        }
    }
}