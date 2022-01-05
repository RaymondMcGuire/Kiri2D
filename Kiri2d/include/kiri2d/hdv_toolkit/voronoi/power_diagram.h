/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2021-12-29 15:01:22
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

    class PowerDiagram3D
    {
    public:
        explicit PowerDiagram3D() { mMesh = std::make_shared<VoronoiMesh3>(); }
        virtual ~PowerDiagram3D() noexcept {}

        void AddSite(const HDV::Primitives::Vertex3Ptr &site) { mSites.emplace_back(site); }
        std::vector<HDV::Primitives::Vertex3Ptr> GetSites() { return mSites; }

        int mCounter = 0;
        int mTCounter = 0;

        void SetBoundaryPolygon(std::vector<csgjscpp::Polygon> boundary)
        {
            auto boundaryPolygon = std::make_shared<VoronoiPolygon3>();
            for (auto i = 0; i < boundary.size(); i++)
            {
                auto face = boundary[i];
                boundaryPolygon->AddVert3(Vector3D(face.vertices[0].pos.x, face.vertices[0].pos.y, face.vertices[0].pos.z));
                boundaryPolygon->AddVert3(Vector3D(face.vertices[1].pos.x, face.vertices[1].pos.y, face.vertices[1].pos.z));
                boundaryPolygon->AddVert3(Vector3D(face.vertices[2].pos.x, face.vertices[2].pos.y, face.vertices[2].pos.z));
            }

            auto bbox = boundaryPolygon->BBox;

            mMesh->SetBoundaryPolygon(boundary);
            mMesh->SetBoundaryBBox(bbox);

            KIRI_LOG_ERROR("bbox low={0},{1}, high={2},{3}, width={4}, height={5}", bbox.LowestPoint.x, bbox.LowestPoint.y, bbox.HighestPoint.x, bbox.HighestPoint.y, bbox.width(), bbox.height());

            if (bbox.width() < 0.0 || bbox.height() < 0.0 || bbox.depth() < 0.0)
                KIRI_LOG_ERROR("bbox low={0},{1}, high={2},{3}, width={4}, height={5}", bbox.LowestPoint.x, bbox.LowestPoint.y, bbox.HighestPoint.x, bbox.HighestPoint.y, bbox.width(), bbox.height());

            auto site1 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y - bbox.height(), bbox.LowestPoint.z - bbox.depth());
            auto site2 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height(), bbox.LowestPoint.z + 2.0 * bbox.depth());
            auto site3 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y - bbox.height(), bbox.LowestPoint.z + 2.0 * bbox.depth());
            auto site4 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height(), bbox.LowestPoint.z + 2.0 * bbox.depth());
            auto site5 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height(), bbox.LowestPoint.z - bbox.depth());
            auto site6 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y - bbox.height(), bbox.LowestPoint.z - bbox.depth());
            auto site7 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y - bbox.height(), bbox.LowestPoint.z + 2.0 * bbox.depth());
            auto site8 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height(), bbox.LowestPoint.z - bbox.depth());

            site1->SetAsBoundaryVertex();
            site2->SetAsBoundaryVertex();
            site3->SetAsBoundaryVertex();
            site4->SetAsBoundaryVertex();
            site5->SetAsBoundaryVertex();
            site6->SetAsBoundaryVertex();
            site7->SetAsBoundaryVertex();
            site8->SetAsBoundaryVertex();

            AddSite(site1);
            AddSite(site2);
            AddSite(site3);
            AddSite(site4);
            AddSite(site5);
            AddSite(site6);
            AddSite(site7);
            AddSite(site8);
        }

        // void SetBoundaryPolygon(const std::shared_ptr<VoronoiPolygon3> &boundary)
        // {
        //     mMesh->SetBoundaryPolygon(boundary);
        //     auto bbox = boundary->BBox;

        //     if (bbox.width() < 0.0 || bbox.height() < 0.0 || bbox.depth() < 0.0)
        //         KIRI_LOG_ERROR("bbox low={0},{1}, high={2},{3}, width={4}, height={5}", bbox.LowestPoint.x, bbox.LowestPoint.y, bbox.HighestPoint.x, bbox.HighestPoint.y, bbox.width(), bbox.height());

        //     auto site1 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y - bbox.height(), bbox.LowestPoint.z - bbox.depth());
        //     auto site2 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height(), bbox.LowestPoint.z + 2.0 * bbox.depth());
        //     auto site3 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y - bbox.height(), bbox.LowestPoint.z + 2.0 * bbox.depth());
        //     auto site4 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height(), bbox.LowestPoint.z + 2.0 * bbox.depth());
        //     auto site5 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height(), bbox.LowestPoint.z - bbox.depth());
        //     auto site6 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y - bbox.height(), bbox.LowestPoint.z - bbox.depth());
        //     auto site7 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y - bbox.height(), bbox.LowestPoint.z + 2.0 * bbox.depth());
        //     auto site8 = std::make_shared<Voronoi::VoronoiSite3>(bbox.LowestPoint.x + 2.0 * bbox.width(), bbox.LowestPoint.y + 2.0 * bbox.height(), bbox.LowestPoint.z - bbox.depth());

        //     site1->SetAsBoundaryVertex();
        //     site2->SetAsBoundaryVertex();
        //     site3->SetAsBoundaryVertex();
        //     site4->SetAsBoundaryVertex();
        //     site5->SetAsBoundaryVertex();
        //     site6->SetAsBoundaryVertex();
        //     site7->SetAsBoundaryVertex();
        //     site8->SetAsBoundaryVertex();

        //     AddSite(site1);
        //     AddSite(site2);
        //     AddSite(site3);
        //     AddSite(site4);
        //     AddSite(site5);
        //     AddSite(site6);
        //     AddSite(site7);
        //     AddSite(site8);
        // }

        void Init()
        {
            mConstrainSites.clear();
            Compute();
            // AddConstrain();
        }

        void LloydIteration()
        {
            // Reset();
            Move2Centroid();
            Compute();
            Move2Centroid();
            Compute();
        }

        VoronoiMesh3Ptr mMesh;

    private:
        bool mAssignIds = true;
        bool mCheckInput = false;

        std::unordered_set<int> mConstrainSites;

        std::vector<HDV::Primitives::Vertex3Ptr> mSites;

        void Compute()
        {
            mMesh->Generate(mSites, mAssignIds, mCheckInput);

            if (mTCounter % 2 == 0)
                mMesh->ExportVoronoiMeshObj(mCounter++);

            mTCounter++;
            // KIRI_LOG_DEBUG("resgion size={0}", mMesh->Regions.size());
        }

        void Move2Centroid()
        {
            for (size_t i = 0; i < mSites.size(); i++)
            {
                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(mSites[i]);
                if (site->GetIsBoundaryVertex())
                    continue;

                auto centroid = site->Polygon->GetCentroid();
                site->Set(centroid.x, centroid.y, centroid.z);
            }
        }

        void AddConstrain()
        {
            for (size_t i = 0; i < mSites.size(); i++)
            {
                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(mSites[i]);
                if (site->X() < 0.0)
                    mConstrainSites.insert(site->GetId());
            }

            mMesh->mConstrainSites = mConstrainSites;
        }

        void Reset()
        {
            for (size_t i = 0; i < mSites.size(); i++)
            {
                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(mSites[i]);
                site->Reset();
            }
        }
    };

} // namespace HDV::Voronoi

#endif /* _HDV_POWER_DIAGRAM_H_ */