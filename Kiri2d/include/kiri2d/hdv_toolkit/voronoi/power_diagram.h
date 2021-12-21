/***
 * @Author: Xu.WANG
 * @Date: 2021-12-21 21:10:15
 * @LastEditTime: 2021-12-21 21:13:35
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_POWER_DIAGRAM_H_
#define _HDV_POWER_DIAGRAM_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_mesh.h>

namespace HDV::Voronoi
{
    class PowerDiagram2D
    {
    public:
        explicit PowerDiagram2D() { mMesh = std::make_shared<VoronoiMesh2>(); }
        virtual ~PowerDiagram2D() noexcept {}

        void AddSite(const HDV::Primitives::Vertex2Ptr &site) { mSites.emplace_back(site); }
        std::vector<HDV::Primitives::Vertex2Ptr> GetSites() { return mSites; }

        void Compute()
        {
            mMesh->Generate(mSites, mAssignIds, mCheckInput);
        }

        void LloydIteration()
        {
            Reset();
            Move2Centroid();
            Compute();
        }

        VoronoiMesh2Ptr mMesh;

    private:
        bool mAssignIds = true;
        bool mCheckInput = false;

        std::vector<HDV::Primitives::Vertex2Ptr> mSites;

        void Move2Centroid()
        {
            for (size_t i = 0; i < mSites.size(); i++)
            {
                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(mSites[i]);
                if (site->GetIsBoundaryVertex())
                    continue;

                auto centroid = site->CellPolygon->GetCentroid();
                site->Set(centroid.x, centroid.y);
            }
        }

        void Reset()
        {
            for (size_t i = 0; i < mSites.size(); i++)
            {
                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(mSites[i]);
                site->Reset();
            }
        }
    };

} // namespace HDV::Voronoi

#endif /* _HDV_POWER_DIAGRAM_H_ */