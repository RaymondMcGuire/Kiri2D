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

        const std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> &GetBoundary()
        {
            return mMesh->mBoundaryPolygon;
        }

        void SetBoundaryPolygon(const std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> &boundary)
        {
            mMesh->SetBoundaryPolygon(boundary);

            auto mBBox = boundary->BBox;

            if (mBBox.width() < 0.0 || mBBox.height() < 0.0)
                KIRI_LOG_ERROR("mBBox low={0},{1}, high={2},{3}, width={4}, height={5}", mBBox.LowestPoint.x, mBBox.LowestPoint.y, mBBox.HighestPoint.x, mBBox.HighestPoint.y, mBBox.width(), mBBox.height());

            auto site1 = std::make_shared<Voronoi::VoronoiSite2>(mBBox.LowestPoint.x - mBBox.width(), mBBox.LowestPoint.y - mBBox.height());
            auto site2 = std::make_shared<Voronoi::VoronoiSite2>(mBBox.LowestPoint.x + 2.0 * mBBox.width(), mBBox.LowestPoint.y - mBBox.height());
            auto site3 = std::make_shared<Voronoi::VoronoiSite2>(mBBox.LowestPoint.x + 2.0 * mBBox.width(), mBBox.LowestPoint.y + 2.0 * mBBox.height());
            auto site4 = std::make_shared<Voronoi::VoronoiSite2>(mBBox.LowestPoint.x - mBBox.width(), mBBox.LowestPoint.y + 2.0 * mBBox.height());

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
        BoundingBox3D mBBox;

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

            mBBox = boundaryPolygon->BBox;

            mMesh->SetBoundaryPolygon(boundary);
            mMesh->SetBoundaryBBox(mBBox);

            if (mBBox.width() < 0.0 || mBBox.height() < 0.0 || mBBox.depth() < 0.0)
                KIRI_LOG_ERROR("mBBox low={0},{1}, high={2},{3}, width={4}, height={5}", mBBox.LowestPoint.x, mBBox.LowestPoint.y, mBBox.HighestPoint.x, mBBox.HighestPoint.y, mBBox.width(), mBBox.height());

            auto site1 = std::make_shared<Voronoi::VoronoiSite3>(mBBox.LowestPoint.x - mBBox.width(), mBBox.LowestPoint.y - mBBox.height(), mBBox.LowestPoint.z - mBBox.depth());
            auto site2 = std::make_shared<Voronoi::VoronoiSite3>(mBBox.LowestPoint.x + 2.0 * mBBox.width(), mBBox.LowestPoint.y + 2.0 * mBBox.height(), mBBox.LowestPoint.z + 2.0 * mBBox.depth());
            auto site3 = std::make_shared<Voronoi::VoronoiSite3>(mBBox.LowestPoint.x - mBBox.width(), mBBox.LowestPoint.y - mBBox.height(), mBBox.LowestPoint.z + 2.0 * mBBox.depth());
            auto site4 = std::make_shared<Voronoi::VoronoiSite3>(mBBox.LowestPoint.x - mBBox.width(), mBBox.LowestPoint.y + 2.0 * mBBox.height(), mBBox.LowestPoint.z + 2.0 * mBBox.depth());
            auto site5 = std::make_shared<Voronoi::VoronoiSite3>(mBBox.LowestPoint.x - mBBox.width(), mBBox.LowestPoint.y + 2.0 * mBBox.height(), mBBox.LowestPoint.z - mBBox.depth());
            auto site6 = std::make_shared<Voronoi::VoronoiSite3>(mBBox.LowestPoint.x + 2.0 * mBBox.width(), mBBox.LowestPoint.y - mBBox.height(), mBBox.LowestPoint.z - mBBox.depth());
            auto site7 = std::make_shared<Voronoi::VoronoiSite3>(mBBox.LowestPoint.x + 2.0 * mBBox.width(), mBBox.LowestPoint.y - mBBox.height(), mBBox.LowestPoint.z + 2.0 * mBBox.depth());
            auto site8 = std::make_shared<Voronoi::VoronoiSite3>(mBBox.LowestPoint.x + 2.0 * mBBox.width(), mBBox.LowestPoint.y + 2.0 * mBBox.height(), mBBox.LowestPoint.z - mBBox.depth());

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

        void Init()
        {
            mConstrainSites.clear();
            Compute();
            AddConstrain();
        }

        void LloydIteration()
        {
            // Reset();
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
            mMesh->ExportVoronoiMeshObj(mCounter++);
            //  KIRI_LOG_DEBUG("resgion size={0}", mMesh->Regions.size());
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
            auto center = (mBBox.LowestPoint + mBBox.HighestPoint) / 2.0;
            for (size_t i = 0; i < mSites.size(); i++)
            {
                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(mSites[i]);

                if (site->Z() < (center.z + mBBox.depth() * 0.2))
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

    typedef std::shared_ptr<PowerDiagram2D> PowerDiagram2Ptr;
    typedef std::shared_ptr<PowerDiagram3D> PowerDiagram3Ptr;

} // namespace HDV::Voronoi

#endif /* _HDV_POWER_DIAGRAM_H_ */