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

namespace HDV::Voronoi
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiMesh
    {
    public:
        explicit VoronoiMesh(int dimension) { Dimension = dimension; }
        virtual ~VoronoiMesh() noexcept {}

        int Dimension;
        std::vector<std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>> Cells;
        std::vector<std::shared_ptr<VoronoiRegion<VERTEXPTR, VERTEX>>> Regions;

        virtual void Clear()
        {
            Cells.clear();
            Regions.clear();
        }

        virtual void Generate(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) = 0;

        virtual void LloydIteration(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) = 0;

    protected:
        void GenerateVoronoi(std::vector<VERTEXPTR> input, const std::shared_ptr<HDV::Delaunay::DelaunayTriangulation<VERTEXPTR, VERTEX>> &delaunay, bool assignIds = true, bool checkInput = false)
        {
            Clear();

            delaunay->Generate(input, assignIds, checkInput);

            for (auto i = 0; i < delaunay->Vertices.size(); i++)
            {
                delaunay->Vertices[i]->SetTag(i);
                // KIRI_LOG_DEBUG("Vertices={0},{1}", delaunay->Vertices[i]->mPosition[0], delaunay->Vertices[i]->mPosition[1]);
            }

            for (auto i = 0; i < delaunay->Cells.size(); i++)
            {
                delaunay->Cells[i]->CircumCenter->SetId(i);
                delaunay->Cells[i]->mSimplex->SetTag(i);
                Cells.emplace_back(delaunay->Cells[i]);
            }

            std::vector<std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>> cells;
            std::map<int, std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>> neighbourCell;

            for (auto i = 0; i < delaunay->Vertices.size(); i++)
            {

                cells.clear();

                auto vertex = delaunay->Vertices[i];

                for (auto j = 0; j < delaunay->Cells.size(); j++)
                {
                    auto simplex = delaunay->Cells[j]->mSimplex;

                    for (auto k = 0; k < simplex->Vertices.size(); k++)
                    {
                        // KIRI_LOG_DEBUG("simplex tag={0}, vertex tag={1}", simplex->Vertices[k]->GetTag(), vertex->GetTag());
                        if (simplex->Vertices[k]->GetTag() == vertex->GetTag())
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

                    for (auto j = 0; j < cells.size(); j++)
                    {
                        region->Cells.emplace_back(cells[j]);
                    }

                    neighbourCell.clear();

                    for (auto j = 0; j < cells.size(); j++)
                    {
                        neighbourCell.insert(std::pair<int, std::shared_ptr<HDV::Delaunay::DelaunayCell<VERTEXPTR, VERTEX>>>(cells[j]->CircumCenter->GetId(), cells[j]));
                    }

                    for (auto j = 0; j < cells.size(); j++)
                    {
                        auto simplex = cells[j]->mSimplex;

                        for (auto k = 0; k < simplex->Adjacent.size(); k++)
                        {
                            if (simplex->Adjacent[k] == nullptr)
                                continue;

                            auto tag = simplex->Adjacent[k]->GetTag();

                            if (neighbourCell.find(tag) != neighbourCell.end())
                            {
                                auto edge = std::make_shared<VoronoiEdge<VERTEXPTR, VERTEX>>(cells[j], neighbourCell[tag]);
                                region->Edges.emplace_back(edge);
                            }
                        }
                    }

                    region->Id = Regions.size();
                    region->site = delaunay->Vertices[i];
                    Regions.emplace_back(region);
                }
            }

            Region2Polygon();
        }

        void Region2Polygon()
        {

            // clip boundary
            auto BoundaryPolygon = std::make_shared<VoronoiCellPolygon<VERTEXPTR, VERTEX>>();
            BoundaryPolygon->AddVert2(Vector2F(-200.f, -200.f));
            BoundaryPolygon->AddVert2(Vector2F(-200.f, 200.f));
            BoundaryPolygon->AddVert2(Vector2F(200.f, 200.f));
            BoundaryPolygon->AddVert2(Vector2F(200.f, -200.f));
            // KIRI_LOG_DEBUG("Regions={0}", Regions.size());
            for (auto i = 0; i < Regions.size(); i++)
            {
                std::vector<VERTEXPTR> verts;
                auto count = 0;
                auto region = Regions[i];
                for (auto j = 0; j < region->Edges.size(); j++)
                {
                    auto edge = region->Edges[j];
                    auto from = edge->From->CircumCenter;
                    auto to = edge->To->CircumCenter;
                    // Polygons.emplace_back(Vector4F(from->X(), from->Y(), to->X(), to->Y()));
                    verts.emplace_back(std::make_shared<Primitives::Vertex2>(from->X(), from->Y(), count++));
                    verts.emplace_back(std::make_shared<Primitives::Vertex2>(to->X(), to->Y(), count++));
                    // KIRI_LOG_DEBUG("start={0},{1}; end={2},{3}", from->X(), from->Y(), to->X(), to->Y());
                }

                // KIRI_LOG_DEBUG("------verts size={0}------", verts.size());
                auto hull = std::make_shared<HDV::Hull::ConvexHull<VERTEXPTR>>(Dimension);
                hull->Generate(verts);

                auto simplexs = hull->GetSortSimplexsList();

                auto cell_polygon = std::make_shared<VoronoiCellPolygon<VERTEXPTR, VERTEX>>();
                for (auto j = 0; j < simplexs.size(); j++)
                {
                    cell_polygon->AddVert2(Vector2F(simplexs[j].x, simplexs[j].y));
                    cell_polygon->AddVert2(Vector2F(simplexs[j].z, simplexs[j].w));
                    // KIRI_LOG_DEBUG("simplexs = ({0},{1})-({2},{3})", simplexs[j].x, simplexs[j].y, simplexs[j].z, simplexs[j].w);
                }

                if (region->site->GetIsBoundaryVertex())
                    continue;

                // clip voronoi cell polygon
                if (cell_polygon->Verts.size() > 2)
                {
                    if (BoundaryPolygon->BBox.overlaps(cell_polygon->BBox))
                    {
                        if (BoundaryPolygon->BBox.contains(cell_polygon->BBox))
                        {
                        }
                        else
                        {
                            auto A = BoundaryPolygon->Verts;
                            auto B = cell_polygon->Verts;

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

                            // if (!result.getContours().empty() && compute_result == true)

                            if (bintersection && PolyClip::PloygonOpration::Mark(polygon1, polygon2, possible_result, PolyClip::MarkIntersection))
                            {
                                auto clipedPolygon = std::make_shared<VoronoiCellPolygon<VERTEXPTR, VERTEX>>();

                                std::vector<std::vector<PolyClip::Point2d>> results = PolyClip::PloygonOpration::ExtractIntersectionResults(polygon1);
                                for (int pp = 0; pp < results.size(); ++pp)
                                {

                                    for (size_t ppp = 0; ppp < results[pp].size(); ppp++)
                                    {
                                        auto polyn = results[pp][ppp];
                                        clipedPolygon->AddVert2(Vector2F(polyn.x_, polyn.y_));
                                    }
                                }

                                cell_polygon = clipedPolygon;
                            }
                            else
                            {
                            }
                        }
                    }
                }

                auto site = std::dynamic_pointer_cast<VoronoiSite2>(Regions[i]->site);
                site->CellPolygon = cell_polygon;

                // KIRI_LOG_DEBUG("site verts={0}", site->CellPolygon->Verts.size());
            }
        }
    };

    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiMesh2D : public VoronoiMesh<VERTEXPTR, VERTEX>
    {
    public:
        explicit VoronoiMesh2D() : VoronoiMesh<VERTEXPTR, VERTEX>(2) {}
        virtual ~VoronoiMesh2D() noexcept {}

        void Generate(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) override
        {
            auto delaunay = std::make_shared<HDV::Delaunay::DelaunayTriangulation2>();
            this->GenerateVoronoi(input, delaunay, assignIds, checkInput);
        }

        void LloydIteration(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) override
        {
            this->Generate(input, assignIds, checkInput);
        }
    };

    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiMesh3D : public VoronoiMesh<VERTEXPTR, VERTEX>
    {
    public:
        explicit VoronoiMesh3D() : VoronoiMesh<VERTEXPTR, VERTEX>(3) {}
        virtual ~VoronoiMesh3D() noexcept {}

        void Generate(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) override
        {
            auto delaunay = std::make_shared<HDV::Delaunay::DelaunayTriangulation3>();
            this->GenerateVoronoi(input, delaunay, assignIds, checkInput);
        }

        void LloydIteration(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) override
        {
            this->Generate(input, assignIds, checkInput);
        }
    };

    typedef VoronoiMesh2D<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2> VoronoiMesh2;
    typedef VoronoiMesh3D<HDV::Primitives::Vertex3Ptr, HDV::Primitives::Vertex3> VoronoiMesh3;

    typedef std::shared_ptr<VoronoiMesh2D<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> VoronoiMesh2Ptr;
    typedef std::shared_ptr<VoronoiMesh3D<HDV::Primitives::Vertex3Ptr, HDV::Primitives::Vertex3>> VoronoiMesh3Ptr;

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_MESH_H_ */