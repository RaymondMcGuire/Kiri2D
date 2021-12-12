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
        std::vector<Vector4F> Polygons;

        virtual void Clear()
        {
            Cells.clear();
            Regions.clear();
            Polygons.clear();
        }

        virtual void Generate(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) = 0;

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
                    Regions.emplace_back(region);
                }
            }

            Region2Polygon();
        }

        bool EdgeApprox(const std::shared_ptr<VoronoiEdge<VERTEXPTR, VERTEX>> &l, const std::shared_ptr<VoronoiEdge<VERTEXPTR, VERTEX>> &r)
        {
            auto l_from = l->From->CircumCenter;
            auto l_to = l->To->CircumCenter;
            auto r_from = r->From->CircumCenter;
            auto r_to = r->To->CircumCenter;

            auto dim = l_from->GetDimension();

            // check l from == r to case
            auto edge_chk_flag = true;
            for (auto i = 0; i < dim; i++)
            {
                if (std::abs(l_from->mPosition[i] - r_to->mPosition[i]) > std::numeric_limits<float>::epsilon())
                {
                    edge_chk_flag = false;
                    break;
                }

                if (std::abs(l_to->mPosition[i] - r_from->mPosition[i]) > std::numeric_limits<float>::epsilon())
                {
                    edge_chk_flag = false;
                    break;
                }
            }

            if (edge_chk_flag)
                return true;

            for (auto i = 0; i < dim; i++)
            {
                if (std::abs(l_from->mPosition[i] - r_from->mPosition[i]) > std::numeric_limits<float>::epsilon())
                    return false;

                if (std::abs(l_to->mPosition[i] - r_to->mPosition[i]) > std::numeric_limits<float>::epsilon())
                    return false;
            }

            return true;
        }

        void Region2Polygon()
        {

            for (auto i = 0; i < Regions.size(); i++)
            {
                auto region = Regions[i];
                // std::vector<std::shared_ptr<VoronoiEdge<VERTEXPTR, VERTEX>>> edges;

                // for (auto j = 0; j < region->Edges.size() - 1; j++)
                // {
                //     bool flag = true;
                //     for (auto k = j + 1; k < region->Edges.size(); k++)
                //     {
                //         if (EdgeApprox(region->Edges[j], region->Edges[k]))
                //         {
                //             flag = false;
                //             break;
                //         }
                //     }

                //     if (flag)
                //         edges.emplace_back(region->Edges[j]);
                // }

                for (auto j = 0; j < region->Edges.size(); j++)
                {
                    auto edge = region->Edges[j];
                    auto from = edge->From->CircumCenter;
                    auto to = edge->To->CircumCenter;
                    Polygons.emplace_back(Vector4F(from->X(), from->Y(), to->X(), to->Y()));
                    // verts.emplace_back(std::make_shared<Primitives::Vertex2>(from->X(), from->Y(), count++));
                    // verts.emplace_back(std::make_shared<Primitives::Vertex2>(to->X(), to->Y(), count++));
                    // KIRI_LOG_DEBUG("start={0},{1}; end={2},{3}", from->X(), from->Y(), to->X(), to->Y());
                }

                // KIRI_LOG_DEBUG("------verts size={0}------", verts.size());
                // auto hull = std::make_shared<HDV::Hull::ConvexHull<VERTEXPTR>>(Dimension);
                // hull->Generate(verts);

                // auto simplexs = hull->GetSimplexs();

                // for (auto j = 0; j < simplexs.size(); j++)
                // {
                //     auto from = Vector2F(simplexs[j]->Vertices[0]->X(), simplexs[j]->Vertices[0]->Y());
                //     auto to = Vector2F(simplexs[j]->Vertices[1]->X(), simplexs[j]->Vertices[1]->Y());
                //     // KIRI_LOG_DEBUG("start={0},{1}; end={2},{3}", from.x, from.y, to.x, to.y);
                //     Polygons.emplace_back(Vector4F(from.x, from.y, to.x, to.y));
                // }
                // KIRI_LOG_DEBUG("-------------------------");
            }
        }
    };

    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiMesh2D : public VoronoiMesh<VERTEXPTR, VERTEX>
    {
    public:
        explicit VoronoiMesh2D() : VoronoiMesh(2) {}
        virtual ~VoronoiMesh2D() noexcept {}

        void Generate(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) override
        {
            auto delaunay = std::make_shared<HDV::Delaunay::DelaunayTriangulation2>();
            GenerateVoronoi(input, delaunay, assignIds, checkInput);
        }

        void ClipWithBoundary(std::vector<PolyClip::Point2d> polyA)
        {
        }
    };

    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiMesh3D : public VoronoiMesh<VERTEXPTR, VERTEX>
    {
    public:
        explicit VoronoiMesh3D() : VoronoiMesh(3) {}
        virtual ~VoronoiMesh3D() noexcept {}

        void Generate(std::vector<VERTEXPTR> input, bool assignIds = true, bool checkInput = false) override
        {
            auto delaunay = std::make_shared<HDV::Delaunay::DelaunayTriangulation3>();
            GenerateVoronoi(input, delaunay, assignIds, checkInput);
        }
    };

    typedef VoronoiMesh2D<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2> VoronoiMesh2;
    typedef VoronoiMesh3D<HDV::Primitives::Vertex3Ptr, HDV::Primitives::Vertex3> VoronoiMesh3;

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_MESH_H_ */