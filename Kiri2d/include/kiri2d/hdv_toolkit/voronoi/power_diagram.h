/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2021-12-29 15:01:22
 * @LastEditors: Xu.WANG
 * @Description:
 */
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

        void SetBoundaryPolygon(const std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> &boundary)
        {
            mMesh->SetBoundaryPolygon(boundary);
            auto bbox = boundary->BBox;

            if (bbox.width() < 0.0 || bbox.height() < 0.0)
                KIRI_LOG_ERROR("bbox low={0},{1}, high={2},{3}, width={4}, height={5}", bbox.LowestPoint.x, bbox.LowestPoint.y, bbox.HighestPoint.x, bbox.HighestPoint.y, bbox.width(), bbox.height());

            auto site1 = std::make_shared<Voronoi::VoronoiSite2>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y - bbox.height());
            auto site2 = std::make_shared<Voronoi::VoronoiSite2>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y - bbox.height());
            auto site3 = std::make_shared<Voronoi::VoronoiSite2>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height());
            auto site4 = std::make_shared<Voronoi::VoronoiSite2>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height());

            site1->SetAsBoundaryVertex();
            site2->SetAsBoundaryVertex();
            site3->SetAsBoundaryVertex();
            site4->SetAsBoundaryVertex();

            AddSite(site1);
            AddSite(site2);
            AddSite(site3);
            AddSite(site4);
        }

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