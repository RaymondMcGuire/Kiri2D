/***
 * @Author: Xu.WANG
 * @Date: 2021-12-08 20:02:06
 * @LastEditTime: 2021-12-08 20:03:53
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_VORONOI_MESH_H_
#define _HDV_VORONOI_MESH_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_region.h>
#include <kiri2d/hdv_toolkit/voronoi/voronoi_site.h>
#include <kiri2d/hdv_toolkit/delaunay/delaunay_triangulation2.h>
#include <kiri2d/hdv_toolkit/delaunay/delaunay_triangulation3.h>
#include <kiri2d/poly/PolygonClipping.h>
#include <kiri2d/hdv_toolkit/utils/tinyobj_utils.h>

#define CSGJSCPP_IMPLEMENTATION
#define CSGJSCPP_REAL double
#include <csgjs.h>

namespace HDV::Voronoi
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiMesh
    {
    public:
        explicit VoronoiMesh(int dimension) { Dimension = dimension; }
        virtual ~VoronoiMesh() {}

        int Dimension;
        std::vector<std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>> Cells;
        std::vector<std::shared_ptr<VoronoiRegion<VERTEXPTR, VERTEX>>> Regions;

        virtual void clear()
        {
            for (auto i = 0; i < Cells.size(); i++)
                Cells[i]->clear();

            for (auto i = 0; i < Regions.size(); i++)
                Regions[i]->clear();

            Cells.clear();
            Regions.clear();
        }

        virtual void Generate(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) = 0;

        virtual void Region2Polygon() = 0;

    protected:
        void GenerateVoronoi(std::vector<VERTEXPTR> input, const std::shared_ptr<HDV::Delaunay::DelaunayTriangulation<VERTEXPTR, VERTEX>> &delaunay, bool assignIds = true, bool checkInput = false)
        {
            clear();

            delaunay->Generate(input, assignIds, checkInput);

            // #pragma omp parallel for
            for (auto i = 0; i < delaunay->Vertices.size(); i++)
            {
                delaunay->Vertices[i]->setTag(i);
                // KIRI_LOG_DEBUG("delaunay->Vertices id={0}, Vertices={1},{2},{3}", delaunay->Vertices[i]->id(), delaunay->Vertices[i]->positions()[0], delaunay->Vertices[i]->positions()[1], delaunay->Vertices[i]->positions()[2]);
            }

            for (auto i = 0; i < delaunay->Cells.size(); i++)
            {
                delaunay->Cells[i]->CircumCenter->setId(i);
                delaunay->Cells[i]->mSimplex->setTag(i);
                Cells.emplace_back(delaunay->Cells[i]);
            }

            std::vector<std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>> cells;
            std::map<int, std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>> neighbourCell;
            // KIRI_LOG_DEBUG("------region generate-------");
            for (auto i = 0; i < delaunay->Vertices.size(); i++)
            {
                cells.clear();

                auto vertex = delaunay->Vertices[i];
                vertex->neighbors().clear();

                for (auto j = 0; j < delaunay->Cells.size(); j++)
                {
                    auto simplex = delaunay->Cells[j]->mSimplex;

                    for (auto k = 0; k < simplex->vertices().size(); k++)
                    {
                        // KIRI_LOG_DEBUG("simplex tag={0}, vertex tag={1}", simplex->vertices()[k]->tag(), vertex->tag());
                        if (simplex->vertices()[k]->tag() == vertex->tag())
                        {

                            cells.emplace_back(delaunay->Cells[j]);
                            break;
                        }
                    }
                }

                // KIRI_LOG_DEBUG("cell size={0}", cells.size());

                if (cells.size() > 0)
                {
                    auto region = std::make_shared<VoronoiRegion<VERTEXPTR, VERTEX>>();

                    std::unordered_set<int> neighborsId;
                    for (auto j = 0; j < cells.size(); j++)
                    {
                        auto simplex = cells[j]->mSimplex;
                        for (auto k = 0; k < simplex->vertices().size(); k++)
                        {
                            if (simplex->vertices()[k]->id() != vertex->id() && !simplex->vertices()[k]->isBoundaryVertex())
                            {
                                if (neighborsId.find(simplex->vertices()[k]->id()) == neighborsId.end())
                                {
                                    vertex->neighbors().emplace_back(simplex->vertices()[k]);
                                    neighborsId.insert(simplex->vertices()[k]->id());
                                }
                            }
                        }

                        region->Cells.emplace_back(cells[j]);
                    }

                    neighbourCell.clear();

                    for (auto j = 0; j < cells.size(); j++)
                    {
                        neighbourCell.insert(std::pair<int, std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>>(cells[j]->CircumCenter->id(), cells[j]));
                    }

                    for (auto j = 0; j < cells.size(); j++)
                    {
                        auto simplex = cells[j]->mSimplex;

                        for (auto k = 0; k < simplex->adjacents().size(); k++)
                        {
                            if (simplex->adjacents()[k] == nullptr)
                                continue;

                            auto tag = simplex->adjacents()[k]->tag();

                            if (neighbourCell.find(tag) != neighbourCell.end())
                            {
                                auto edge = std::make_shared<VoronoiEdge<VERTEXPTR, VERTEX>>(cells[j], neighbourCell[tag]);
                                region->Edges.emplace_back(edge);
                            }
                        }
                    }

                    region->Id = Regions.size();
                    region->site = delaunay->Vertices[i];

                    // auto site = std::dynamic_pointer_cast<VoronoiSite3>(region->site);
                    // KIRI_LOG_DEBUG("id={0}; pos={1},{2},{3}; region id={4}", site->id(), site->x()(), site->y()(), site->z()(), region->Id);

                    Regions.emplace_back(region);
                }
            }

            // KIRI_LOG_DEBUG("Regions size={0}", Regions.size());

            Region2Polygon();

            delaunay->clear();
        }
    };

    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiMesh2D : public VoronoiMesh<VERTEXPTR, VERTEX>
    {
    public:
        explicit VoronoiMesh2D() : VoronoiMesh<VERTEXPTR, VERTEX>(2) {}
        virtual ~VoronoiMesh2D() {}

        std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> mBoundaryPolygon;

        void Generate(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) override
        {
            auto delaunay = std::make_shared<HDV::Delaunay::DelaunayTriangulation2>();
            this->GenerateVoronoi(input, delaunay, assignIds, checkInput);
        }

        void SetBoundaryPolygon(const std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> &boundary)
        {
            mBoundaryPolygon = boundary;
        }

        void Region2Polygon() override
        {

            // #pragma omp parallel for
            for (auto i = 0; i < Regions.size(); i++)
            {
                std::vector<VERTEXPTR> Positions;
                auto count = 0;
                auto region = Regions[i];

                if (region->site->isBoundaryVertex())
                    continue;

                for (auto j = 0; j < region->Cells.size(); j++)
                {
                    auto vert = region->Cells[j]->CircumCenter;
                    Positions.emplace_back(std::make_shared<Primitives::Vertex2>(vert->x(), vert->y(), count++));
                }

                auto hull = std::make_shared<HDV::Hull::ConvexHull<VERTEXPTR>>(Dimension);
                hull->Generate(Positions);

                auto simplexs = hull->GetSortSimplexsList();

                // KIRI_LOG_DEBUG("------simplexs size={0}------", simplexs.size());
                auto cell_polygon = std::make_shared<VoronoiCellPolygon<VERTEXPTR, VERTEX>>();
                for (auto j = 0; j < simplexs.size(); j++)
                {
                    if (j == 0)
                        cell_polygon->AddVert2(Vector2D(simplexs[j].x, simplexs[j].y));

                    if (j != simplexs.size() - 1)
                        cell_polygon->AddVert2(Vector2D(simplexs[j].z, simplexs[j].w));

                    // KIRI_LOG_DEBUG("simplexs = ({0},{1})-({2},{3})", simplexs[j].x, simplexs[j].y, simplexs[j].z, simplexs[j].w);
                }

                // clip voronoi cell polygon
                if (cell_polygon->Positions.size() > 2)
                {
                    if (mBoundaryPolygon->BBox.overlaps(cell_polygon->BBox))
                    {
                        if (!mBoundaryPolygon->BBox.contains(cell_polygon->BBox))
                        {
                            auto A = mBoundaryPolygon->Positions;
                            auto B = cell_polygon->Positions;

                            std::vector<PolyClip::Point2d> polyA;
                            std::vector<PolyClip::Point2d> polyB;

                            for (size_t ai = 0; ai < A.size(); ai++)
                                polyA.push_back(PolyClip::Point2d(A[ai].x, A[ai].y));

                            for (size_t bi = 0; bi < B.size(); bi++)
                                polyB.push_back(PolyClip::Point2d(B[bi].x, B[bi].y));

                            PolyClip::Polygon polygon1(polyA);
                            PolyClip::Polygon polygon2(polyB);
                            auto bintersection = PolyClip::PloygonOpration::DetectIntersection(polygon1, polygon2);
                            std::vector<std::vector<PolyClip::Point2d>> possible_result;

                            if (bintersection && PolyClip::PloygonOpration::Mark(polygon1, polygon2, possible_result, PolyClip::MarkIntersection))
                            {
                                auto clipedPolygon = std::make_shared<VoronoiCellPolygon<VERTEXPTR, VERTEX>>();

                                std::vector<std::vector<PolyClip::Point2d>> results = PolyClip::PloygonOpration::ExtractIntersectionResults(polygon1);
                                for (int pp = 0; pp < results.size(); ++pp)
                                {

                                    for (size_t ppp = 0; ppp < results[pp].size(); ppp++)
                                    {
                                        auto polyn = results[pp][ppp];
                                        clipedPolygon->AddVert2(Vector2D(polyn.x_, polyn.y_));
                                    }
                                }

                                //! TODO
                                clipedPolygon->Positions.pop_back();
                                std::reverse(clipedPolygon->Positions.begin(), clipedPolygon->Positions.end());

                                cell_polygon = clipedPolygon;
                            }
                        }
                        else
                        {
                            cell_polygon = nullptr;
                        }
                    }
                    else
                    {
                        cell_polygon = nullptr;
                    }
                }
                else
                {
                    cell_polygon = nullptr;
                }

                auto site = std::dynamic_pointer_cast<VoronoiSite2>(Regions[i]->site);
                site->CellPolygon = cell_polygon;

                hull->clear();

                // if (!cell_polygon->BBox.contains(Vector2D(site->x()(), site->y()())))
                //     KIRI_LOG_ERROR("bbox not contain!!");
            }
        }
    };

    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiMesh3D : public VoronoiMesh<VERTEXPTR, VERTEX>
    {
    public:
        explicit VoronoiMesh3D() : VoronoiMesh<VERTEXPTR, VERTEX>(3) {}
        virtual ~VoronoiMesh3D() {}

        bool mNeedClipBoundary = true;

        int mIndexOffset = 0;
        tinyobj::attrib_t mAttrib;
        std::vector<tinyobj::shape_t> mTinyObjShapes;
        std::vector<tinyobj::material_t> mTinyObjmaterials;

        std::unordered_set<int> mConstrainSites;

        std::vector<csgjscpp::Polygon> mBoundaryPolygon;
        BoundingBox3D mBoundaryBBox;

        void ExportVoronoiMeshObj(int idx)
        {
            TinyObjWriter(UInt2Str4Digit(idx), mAttrib, mTinyObjShapes, mTinyObjmaterials);
        }

        void TinyObjclear()
        {
            mIndexOffset = 0;
            mTinyObjShapes.clear();
            mTinyObjmaterials.clear();
            mAttrib = tinyobj::attrib_t();
        }

        void TinyObjAppend(
            std::vector<Vector3D> positions,
            std::vector<Vector3D> normals,
            std::vector<int> indices)
        {
            tinyobj::shape_t tinyObjShape;
            for (auto j = 0; j < positions.size() / 3; j++)
            {

                auto idx1 = j * 3;
                auto idx2 = j * 3 + 1;
                auto idx3 = j * 3 + 2;

                mAttrib.vertices.emplace_back(static_cast<float>(positions[idx1].x));
                mAttrib.vertices.emplace_back(static_cast<float>(positions[idx1].y));
                mAttrib.vertices.emplace_back(static_cast<float>(positions[idx1].z));

                mAttrib.vertices.emplace_back(static_cast<float>(positions[idx2].x));
                mAttrib.vertices.emplace_back(static_cast<float>(positions[idx2].y));
                mAttrib.vertices.emplace_back(static_cast<float>(positions[idx2].z));

                mAttrib.vertices.emplace_back(static_cast<float>(positions[idx3].x));
                mAttrib.vertices.emplace_back(static_cast<float>(positions[idx3].y));
                mAttrib.vertices.emplace_back(static_cast<float>(positions[idx3].z));

                mAttrib.normals.emplace_back(static_cast<float>(normals[idx1].x));
                mAttrib.normals.emplace_back(static_cast<float>(normals[idx1].y));
                mAttrib.normals.emplace_back(static_cast<float>(normals[idx1].z));

                mAttrib.normals.emplace_back(static_cast<float>(normals[idx2].x));
                mAttrib.normals.emplace_back(static_cast<float>(normals[idx2].y));
                mAttrib.normals.emplace_back(static_cast<float>(normals[idx2].z));

                mAttrib.normals.emplace_back(static_cast<float>(normals[idx3].x));
                mAttrib.normals.emplace_back(static_cast<float>(normals[idx3].y));
                mAttrib.normals.emplace_back(static_cast<float>(normals[idx3].z));

                tinyobj::index_t i1, i2, i3;
                i1.vertex_index = indices[idx1] + mIndexOffset;
                i2.vertex_index = indices[idx2] + mIndexOffset;
                i3.vertex_index = indices[idx3] + mIndexOffset;

                i1.normal_index = -1;
                i2.normal_index = -1;
                i3.normal_index = -1;

                i1.texcoord_index = -1;
                i2.texcoord_index = -1;
                i3.texcoord_index = -1;

                tinyObjShape.mesh.indices.emplace_back(i1);
                tinyObjShape.mesh.indices.emplace_back(i2);
                tinyObjShape.mesh.indices.emplace_back(i3);

                tinyObjShape.mesh.num_face_vertices.emplace_back(3);
                tinyObjShape.mesh.material_ids.emplace_back(-1);
            }
            mIndexOffset += positions.size();
            mTinyObjShapes.emplace_back(tinyObjShape);
        }

        void Generate(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) override
        {
            auto delaunay = std::make_shared<HDV::Delaunay::DelaunayTriangulation3>();
            this->GenerateVoronoi(input, delaunay, assignIds, checkInput);
        }

        void SetBoundaryPolygon(std::vector<csgjscpp::Polygon> boundary)
        {
            mBoundaryPolygon = boundary;
        }

        void SetBoundaryBBox(BoundingBox3D bbox)
        {
            mBoundaryBBox = bbox;
        }

        void Region2Polygon() override
        {
            auto counter = 0;
            // clear obj writer
            TinyObjclear();
            // KIRI_LOG_DEBUG("----------Region2Polygon------------------");
            for (auto i = 0; i < Regions.size(); i++)
            {
                std::vector<VERTEXPTR> Positions;
                auto count = 0;
                auto region = Regions[i];

                if (region->site->isBoundaryVertex())
                    continue;

                for (auto j = 0; j < region->Cells.size(); j++)
                {
                    auto vert = region->Cells[j]->CircumCenter;
                    Positions.emplace_back(std::make_shared<Primitives::Vertex3>(vert->x(), vert->y(), vert->z(), count++));
                }

                auto hull = std::make_shared<HDV::Hull::ConvexHull<VERTEXPTR>>(Dimension);
                hull->Generate(Positions);

                auto simplexs = hull->GetSimplexs();
                auto polygon = std::make_shared<VoronoiPolygon3>();
                // KIRI_LOG_DEBUG("hull vert size={0}", hull->GetVertices().size());

                for (auto j = 0; j < simplexs.size(); j++)
                {
                    for (auto k = 0; k < 3; k++)
                    {
                        auto vertk = simplexs[j]->vertices()[k];
                        auto vec3 = Vector3D(vertk->x(), vertk->y(), vertk->z());
                        polygon->Positions.emplace_back(vec3);
                    }

                    auto normal3 = Vector3D(simplexs[j]->normals()[0], simplexs[j]->normals()[1], simplexs[j]->normals()[2]);
                    polygon->Normals.emplace_back(normal3);
                    polygon->Normals.emplace_back(normal3);
                    polygon->Normals.emplace_back(normal3);

                    polygon->Indices.emplace_back(j * 3 + 0);
                    polygon->Indices.emplace_back(j * 3 + 1);
                    polygon->Indices.emplace_back(j * 3 + 2);
                }
                polygon->UpdateBBox();
                hull->clear();

                auto site = std::dynamic_pointer_cast<VoronoiSite3>(region->site);

                if (!mNeedClipBoundary)
                {
                    // KIRI_LOG_DEBUG("xid={0}; pos={1},{2},{3}", site->id(), site->x()(), site->y()(), site->z()());
                    TinyObjAppend(polygon->Positions, polygon->Normals, polygon->Indices);
                }
                else
                {
                    TinyObjclear();
                    //    KIRI_LOG_DEBUG("start clipped mesh idx={0}", i);
                    std::vector<csgjscpp::Polygon> voroPolygons;
                    for (size_t j = 0; j < polygon->Indices.size() / 3; j++)
                    {

                        auto idx1 = j * 3;
                        auto idx2 = j * 3 + 1;
                        auto idx3 = j * 3 + 2;

                        auto pos1 = polygon->Positions[polygon->Indices[idx1]];
                        auto pos2 = polygon->Positions[polygon->Indices[idx2]];
                        auto pos3 = polygon->Positions[polygon->Indices[idx3]];

                        auto norm1 = polygon->Normals[polygon->Indices[idx1]];
                        auto norm2 = polygon->Normals[polygon->Indices[idx2]];
                        auto norm3 = polygon->Normals[polygon->Indices[idx3]];

                        std::vector<csgjscpp::Vertex> csgPositions;

                        csgPositions.push_back({csgjscpp::Vector(pos1.x, pos1.y, pos1.z), csgjscpp::Vector(norm1.x, norm1.y, norm1.z), csgjscpp::green});
                        csgPositions.push_back({csgjscpp::Vector(pos2.x, pos2.y, pos2.z), csgjscpp::Vector(norm2.x, norm2.y, norm2.z), csgjscpp::green});
                        csgPositions.push_back({csgjscpp::Vector(pos3.x, pos3.y, pos3.z), csgjscpp::Vector(norm3.x, norm3.y, norm3.z), csgjscpp::green});

                        voroPolygons.push_back(csgjscpp::Polygon(csgPositions));
                    }

                    auto voroMesh = csgjscpp::modelfrompolygons(voroPolygons);

                    auto boundaryMesh = csgjscpp::modelfrompolygons(mBoundaryPolygon);

                    auto clippedMesh = csgjscpp::csgintersection(boundaryMesh, voroMesh);

                    // KIRI_LOG_DEBUG("end clipped mesh idx={0}", i);

                    std::vector<Vector3D> clippedPositions;
                    std::vector<Vector3D> clippedNormals;
                    std::vector<int> clippedIndices;

                    for (auto idx = 0; idx < clippedMesh.indices.size(); idx += 3)
                    {
                        auto indIdx1 = clippedMesh.indices[idx];
                        auto indIdx2 = clippedMesh.indices[idx + 1];
                        auto indIdx3 = clippedMesh.indices[idx + 2];

                        auto faceVert1 = clippedMesh.vertices[indIdx1].pos;
                        auto faceVert2 = clippedMesh.vertices[indIdx2].pos;
                        auto faceVert3 = clippedMesh.vertices[indIdx3].pos;

                        auto faceNorm1 = clippedMesh.vertices[indIdx1].normal;
                        auto faceNorm2 = clippedMesh.vertices[indIdx2].normal;
                        auto faceNorm3 = clippedMesh.vertices[indIdx3].normal;

                        clippedPositions.emplace_back(Vector3D(faceVert1.x, faceVert1.y, faceVert1.z));
                        clippedPositions.emplace_back(Vector3D(faceVert2.x, faceVert2.y, faceVert2.z));
                        clippedPositions.emplace_back(Vector3D(faceVert3.x, faceVert3.y, faceVert3.z));

                        clippedNormals.emplace_back(Vector3D(faceNorm1.x, faceNorm1.y, faceNorm1.z));
                        clippedNormals.emplace_back(Vector3D(faceNorm2.x, faceNorm2.y, faceNorm2.z));
                        clippedNormals.emplace_back(Vector3D(faceNorm3.x, faceNorm3.y, faceNorm3.z));

                        clippedIndices.emplace_back(idx);
                        clippedIndices.emplace_back(idx + 1);
                        clippedIndices.emplace_back(idx + 2);
                    }

                    polygon->Positions = clippedPositions;
                    polygon->Normals = clippedNormals;
                    polygon->Indices = clippedIndices;
                    polygon->UpdateBBox();

                    site->Polygon = polygon;

                    // clip condition
                    // if (!mConstrainSites.empty())
                    //     if (mConstrainSites.find(site->id()) == mConstrainSites.end())
                    //         continue;

                    TinyObjAppend(clippedPositions, clippedNormals, clippedIndices);

                    ExportVoronoiMeshObj(counter++);
                }
            }
        }
    };

    typedef VoronoiMesh2D<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2> VoronoiMesh2;
    typedef VoronoiMesh3D<HDV::Primitives::Vertex3Ptr, HDV::Primitives::Vertex3> VoronoiMesh3;

    typedef std::shared_ptr<VoronoiMesh2D<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> VoronoiMesh2Ptr;
    typedef std::shared_ptr<VoronoiMesh3D<HDV::Primitives::Vertex3Ptr, HDV::Primitives::Vertex3>> VoronoiMesh3Ptr;

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_MESH_H_ */