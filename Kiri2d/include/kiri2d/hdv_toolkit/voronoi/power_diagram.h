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
        explicit PowerDiagram2D()
        {
            mMesh = std::make_shared<VoronoiMesh2>();
        }

        virtual ~PowerDiagram2D()
        {
        }

        void addSite(const HDV::Primitives::Vertex2Ptr &site)
        {
            mSites.emplace_back(site);
        }

        std::vector<HDV::Primitives::Vertex2Ptr> sites()
        {
            return mSites;
        }

        const std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> &GetBoundary()
        {
            return mMesh->mBoundaryPolygon;
        }

        void setBoundaryPolygon(const std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> &boundary)
        {
            mMesh->setBoundaryPolygon(boundary);

            auto mBBox = boundary->BBox;

            if (mBBox.width() < 0.0 || mBBox.height() < 0.0)
                KIRI_LOG_ERROR("mBBox low={0},{1}, high={2},{3}, width={4}, height={5}", mBBox.LowestPoint.x, mBBox.LowestPoint.y, mBBox.HighestPoint.x, mBBox.HighestPoint.y, mBBox.width(), mBBox.height());

            auto site1 = std::make_shared<Voronoi::VoronoiSite2>(mBBox.LowestPoint.x - mBBox.width(), mBBox.LowestPoint.y - mBBox.height());
            auto site2 = std::make_shared<Voronoi::VoronoiSite2>(mBBox.LowestPoint.x + 2.0 * mBBox.width(), mBBox.LowestPoint.y - mBBox.height());
            auto site3 = std::make_shared<Voronoi::VoronoiSite2>(mBBox.LowestPoint.x + 2.0 * mBBox.width(), mBBox.LowestPoint.y + 2.0 * mBBox.height());
            auto site4 = std::make_shared<Voronoi::VoronoiSite2>(mBBox.LowestPoint.x - mBBox.width(), mBBox.LowestPoint.y + 2.0 * mBBox.height());

            site1->setAsBoundaryVertex();
            site2->setAsBoundaryVertex();
            site3->setAsBoundaryVertex();
            site4->setAsBoundaryVertex();

            addSite(site1);
            addSite(site2);
            addSite(site3);
            addSite(site4);
        }

        void compute()
        {
            mMesh->generate(mSites, mAssignIds, mCheckInput);
        }

        void removeVoroSitesByIndexArray(std::vector<int> indexs)
        {

            auto removeSites = std::remove_if(mSites.begin(),
                                              mSites.end(),
                                              [=](const HDV::Primitives::Vertex2Ptr &site)
                                              {
                                                  if (std::find(indexs.begin(), indexs.end(), site->id()) != indexs.end())
                                                      return true;
                                                  else
                                                      return false;
                                              });

            mSites.erase(removeSites, mSites.end());
        }

        void move2Centroid()
        {
            //#pragma omp parallel for
            for (auto i = 0; i < mSites.size(); i++)
            {
                if (mSites[i]->isBoundaryVertex())
                    continue;

                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(mSites[i]);

                if (site->cellPolygon())
                {
                    auto centroid = site->cellPolygon()->centroid();
                    if (mMesh->mBoundaryPolygon->contains(centroid))
                        site->set(centroid.x, centroid.y);
                }
            }
        }

        void lloydIteration()
        {
            reset();
            compute();
            move2Centroid();
        }

        VoronoiMesh2Ptr mMesh;

    private:
        bool mAssignIds = true;
        bool mCheckInput = false;

        std::vector<HDV::Primitives::Vertex2Ptr> mSites;

        void reset()
        {
            for (auto i = 0; i < mSites.size(); i++)
            {
                if (mSites[i]->isBoundaryVertex())
                    continue;

                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(mSites[i]);
                site->reset();
                site->cellPolygon() = nullptr;
            }
        }
    };

    class PowerDiagram3D
    {
    public:
        explicit PowerDiagram3D()
        {
            mMesh = std::make_shared<VoronoiMesh3>();
        }

        virtual ~PowerDiagram3D()
        {
        }

        void addSite(const HDV::Primitives::Vertex3Ptr &site) { mSites.emplace_back(site); }
        std::vector<HDV::Primitives::Vertex3Ptr> sites() { return mSites; }

        int mCounter = 0;
        BoundingBox3D mBBox;

        void removeVoroSitesByIndexArray(std::vector<int> indexs)
        {

            auto removeSites = std::remove_if(mSites.begin(),
                                              mSites.end(),
                                              [=](const HDV::Primitives::Vertex3Ptr &site)
                                              {
                                                  if (std::find(indexs.begin(), indexs.end(), site->id()) != indexs.end())
                                                      return true;
                                                  else
                                                      return false;
                                              });

            mSites.erase(removeSites, mSites.end());
        }

        const std::shared_ptr<VoronoiPolygon3> &GetBoundary()
        {
            return mBoundaryPolygon;
        }

        void setBoundaryPolygon(std::vector<csgjscpp::Polygon> boundary)
        {
            auto boundaryMesh = csgjscpp::modelfrompolygons(boundary);

            std::vector<Vector3D> Positions;
            std::vector<Vector3D> Normals;
            std::vector<int> Indices;

            for (auto idx = 0; idx < boundaryMesh.indices.size(); idx += 3)
            {
                auto indIdx1 = boundaryMesh.indices[idx];
                auto indIdx2 = boundaryMesh.indices[idx + 1];
                auto indIdx3 = boundaryMesh.indices[idx + 2];

                auto faceVert1 = boundaryMesh.vertices[indIdx1].pos;
                auto faceVert2 = boundaryMesh.vertices[indIdx2].pos;
                auto faceVert3 = boundaryMesh.vertices[indIdx3].pos;

                auto faceNorm1 = boundaryMesh.vertices[indIdx1].normal;
                auto faceNorm2 = boundaryMesh.vertices[indIdx2].normal;
                auto faceNorm3 = boundaryMesh.vertices[indIdx3].normal;

                Positions.emplace_back(Vector3D(faceVert1.x, faceVert1.y, faceVert1.z));
                Positions.emplace_back(Vector3D(faceVert2.x, faceVert2.y, faceVert2.z));
                Positions.emplace_back(Vector3D(faceVert3.x, faceVert3.y, faceVert3.z));

                Normals.emplace_back(Vector3D(faceNorm1.x, faceNorm1.y, faceNorm1.z));
                Normals.emplace_back(Vector3D(faceNorm2.x, faceNorm2.y, faceNorm2.z));
                Normals.emplace_back(Vector3D(faceNorm3.x, faceNorm3.y, faceNorm3.z));

                Indices.emplace_back(idx);
                Indices.emplace_back(idx + 1);
                Indices.emplace_back(idx + 2);
            }

            mBoundaryPolygon = std::make_shared<VoronoiPolygon3>();
            mBoundaryPolygon->Positions = Positions;
            mBoundaryPolygon->Normals = Normals;
            mBoundaryPolygon->Indices = Indices;
            mBoundaryPolygon->updateBBox();

            mBBox = mBoundaryPolygon->BBox;

            mMesh->setBoundaryPolygon(boundary);
            mMesh->setBoundaryBBox(mBBox);

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

            site1->setAsBoundaryVertex();
            site2->setAsBoundaryVertex();
            site3->setAsBoundaryVertex();
            site4->setAsBoundaryVertex();
            site5->setAsBoundaryVertex();
            site6->setAsBoundaryVertex();
            site7->setAsBoundaryVertex();
            site8->setAsBoundaryVertex();

            addSite(site1);
            addSite(site2);
            addSite(site3);
            addSite(site4);
            addSite(site5);
            addSite(site6);
            addSite(site7);
            addSite(site8);
        }

        void compute()
        {
            mMesh->generate(mSites, mAssignIds, mCheckInput);
            // mMesh->exportVoronoiMeshObj(mCounter++);
            //   KIRI_LOG_DEBUG("resgion size={0}", mMesh->Regions.size());
        }

        void move2Centroid()
        {
            for (auto i = 0; i < mSites.size(); i++)
            {
                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(mSites[i]);
                if (site->isBoundaryVertex())
                    continue;

                auto centroid = site->polygon()->centroid();
                site->set(centroid.x, centroid.y, centroid.z);
            }
        }

        void init()
        {
            mConstrainSites.clear();
            compute();
            addConstrain();
        }

        void lloydIteration()
        {
            // reset();
            compute();
            move2Centroid();
        }

        void exportObj()
        {
            mMesh->exportVoronoiMeshObj(mCounter++);
        }

        VoronoiMesh3Ptr mMesh;

    private:
        bool mAssignIds = true;
        bool mCheckInput = false;

        std::unordered_set<int> mConstrainSites;

        std::shared_ptr<VoronoiPolygon3> mBoundaryPolygon;

        std::vector<HDV::Primitives::Vertex3Ptr> mSites;

        void addConstrain()
        {
            auto center = (mBBox.LowestPoint + mBBox.HighestPoint) / 2.0;
            for (auto i = 0; i < mSites.size(); i++)
            {
                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(mSites[i]);

                if (site->z() < (center.z + mBBox.depth() * 0.2))
                    mConstrainSites.insert(site->id());
            }

            mMesh->mConstrainSites = mConstrainSites;
        }

        void reset()
        {
            for (auto i = 0; i < mSites.size(); i++)
            {
                auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite3>(mSites[i]);
                site->reset();
            }
        }
    };

    typedef std::shared_ptr<PowerDiagram2D> PowerDiagram2Ptr;
    typedef std::shared_ptr<PowerDiagram3D> PowerDiagram3Ptr;

} // namespace HDV::Voronoi

#endif /* _HDV_POWER_DIAGRAM_H_ */