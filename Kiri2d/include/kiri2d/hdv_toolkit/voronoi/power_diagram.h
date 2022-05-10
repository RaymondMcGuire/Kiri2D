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

        const std::shared_ptr<Voronoi::VoronoiPolygon2> &GetBoundary()
        {
            return mMesh->boundaryPolygon();
        }

        void setBoundaryPolygon(const std::shared_ptr<Voronoi::VoronoiPolygon2> &boundary)
        {
            mMesh->setBoundaryPolygon(boundary);

            auto mBBox = boundary->bbox();

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

            auto remove_array = std::remove_if(mSites.begin(),
                                               mSites.end(),
                                               [=](const HDV::Primitives::Vertex2Ptr &site)
                                               {
                                                   if (std::find(indexs.begin(), indexs.end(), site->id()) != indexs.end())
                                                       return true;
                                                   else
                                                       return false;
                                               });

            mSites.erase(remove_array, mSites.end());
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
                    if (mMesh->boundaryPolygon()->contains(centroid))
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

    private:
        bool mAssignIds = true;
        bool mCheckInput = false;

        VoronoiMesh2Ptr mMesh;

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

        void removeVoroSitesByIndexArray(std::vector<int> indexs)
        {

            auto remove_array = std::remove_if(mSites.begin(),
                                               mSites.end(),
                                               [=](const HDV::Primitives::Vertex3Ptr &site)
                                               {
                                                   if (std::find(indexs.begin(), indexs.end(), site->id()) != indexs.end())
                                                       return true;
                                                   else
                                                       return false;
                                               });

            mSites.erase(remove_array, mSites.end());
        }

        const std::shared_ptr<VoronoiPolygon3> &GetBoundary()
        {
            return mBoundaryPolygon;
        }

        void setBoundaryPolygon(std::vector<csgjscpp::Polygon> boundary)
        {
            auto boundary_mesh = csgjscpp::modelfrompolygons(boundary);

            std::vector<Vector3D> positions;
            std::vector<Vector3D> normals;
            std::vector<int> indices;

            for (auto idx = 0; idx < boundary_mesh.indices.size(); idx += 3)
            {
                auto index1 = boundary_mesh.indices[idx];
                auto index2 = boundary_mesh.indices[idx + 1];
                auto index3 = boundary_mesh.indices[idx + 2];

                auto face_vert1 = boundary_mesh.vertices[index1].pos;
                auto face_vert2 = boundary_mesh.vertices[index2].pos;
                auto face_vert3 = boundary_mesh.vertices[index3].pos;

                auto face_norm1 = boundary_mesh.vertices[index1].normal;
                auto face_norm2 = boundary_mesh.vertices[index2].normal;
                auto face_norm3 = boundary_mesh.vertices[index3].normal;

                positions.emplace_back(Vector3D(face_vert1.x, face_vert1.y, face_vert1.z));
                positions.emplace_back(Vector3D(face_vert2.x, face_vert2.y, face_vert2.z));
                positions.emplace_back(Vector3D(face_vert3.x, face_vert3.y, face_vert3.z));

                normals.emplace_back(Vector3D(face_norm1.x, face_norm1.y, face_norm1.z));
                normals.emplace_back(Vector3D(face_norm2.x, face_norm2.y, face_norm2.z));
                normals.emplace_back(Vector3D(face_norm3.x, face_norm3.y, face_norm3.z));

                indices.emplace_back(idx);
                indices.emplace_back(idx + 1);
                indices.emplace_back(idx + 2);
            }

            mBoundaryPolygon = std::make_shared<VoronoiPolygon3>();
            mBoundaryPolygon->positions() = positions;
            mBoundaryPolygon->normals() = normals;
            mBoundaryPolygon->indices() = indices;
            mBoundaryPolygon->updateBBox();

            mBBox = mBoundaryPolygon->bbox();

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

    private:
        int mCounter = 0;
        bool mAssignIds = true;
        bool mCheckInput = false;

        BoundingBox3D mBBox;
        VoronoiMesh3Ptr mMesh;

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

            mMesh->constainSites() = mConstrainSites;
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