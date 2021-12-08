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

namespace HDV::Voronoi
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr>
    class VoronoiMesh
    {
    public:
        explicit VoronoiMesh(int dimension) { Dimension = dimension; }
        virtual ~VoronoiMesh() noexcept {}

        int Dimension;
        std::vector<std::shared_ptr<HDV::Delaunay::DelaunayCell>> Cells;
        std::vector<std::shared_ptr<VoronoiRegion>> Regions;

        virtual void Clear()
        {
            Cells.clear();
            Regions.clear();
        }

        virtual void Generate(std::vector<VERTEXPTR> input, const std::shared_ptr<DelaunayTriangulation<VERTEXPTR>> &delaunay, bool assignIds = true, bool checkInput = false)
        {
            Clear();

            delaunay->Generate(input, assignIds, checkInput);

            for (auto i = 0; i < delaunay->Vertices.size(); i++)
            {
                delaunay->Vertices[i]->SetTag(i);
            }

            // for (auto i = 0; i < delaunay->Vertices.size(); i++)
            // {
            //     delaunay->Cells[i]->CircumCenter.Id = i;
            //     delaunay->Cells[i].Simplex.Tag = i;
            //     Cells.Add(delaunay->Cells[i]);
            // }

            // List<DelaunayCell<VERTEXPTR>> cells = new List<DelaunayCell<VERTEXPTR>>();
            // Dictionary<int, DelaunayCell<VERTEXPTR>> neighbourCell = new Dictionary<int, DelaunayCell<VERTEXPTR>>();

            // for (int i = 0; i < delaunay.Vertices.Count; i++)
            // {

            //     cells.Clear();

            //     VERTEXPTR vertex = delaunay.Vertices[i];

            //     for (int j = 0; j < delaunay->Vertices.size(); j++)
            //     {
            //         Simplex<VERTEXPTR> simplex = delaunay->Cells[j].Simplex;

            //         for (int k = 0; k < simplex.Vertices.Length; k++)
            //         {
            //             if (simplex.Vertices[k].Tag == vertex.Tag)
            //             {
            //                 cells.Add(delaunay->Cells[j]);
            //                 break;
            //             }
            //         }
            //     }

            //     if (cells.Count > 0)
            //     {
            //         VoronoiRegion<VERTEXPTR> region = new VoronoiRegion<VERTEXPTR>();

            //         for (int j = 0; j < cells.Count; j++)
            //         {
            //             region->Cells.Add(cells[j]);
            //         }

            //         neighbourCell.Clear();

            //         for (int j = 0; j < cells.Count; j++)
            //         {
            //             neighbourCell.Add(cells[j]->CircumCenter.Id, cells[j]);
            //         }

            //         for (int j = 0; j < cells.Count; j++)
            //         {
            //             Simplex<VERTEXPTR> simplex = cells[j].Simplex;

            //             for (int k = 0; k < simplex.Adjacent.Length; k++)
            //             {
            //                 if (simplex.Adjacent[k] == null)
            //                     continue;

            //                 int tag = simplex.Adjacent[k].Tag;

            //                 if (neighbourCell.ContainsKey(tag))
            //                 {
            //                     VoronoiEdge<VERTEXPTR> edge = new VoronoiEdge<VERTEXPTR>(cells[j], neighbourCell[tag]);
            //                     region.Edges.Add(edge);
            //                 }
            //             }
            //         }

            //         region.Id = Regions.Count;
            //         Regions.Add(region);
            //     }
            // }
        }
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_MESH_H_ */