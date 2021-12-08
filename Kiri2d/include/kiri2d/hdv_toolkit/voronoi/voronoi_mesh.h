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
    template <typename VERTEX = HDV::Primitives::VertexPtr>
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

        virtual void Generate(std::vector<VERTEX> input, const std::shared_ptr<DelaunayTriangulation<VERTEX>> &delaunay, bool assignIds = true, bool checkInput = false)
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

            // List<DelaunayCell<VERTEX>> cells = new List<DelaunayCell<VERTEX>>();
            // Dictionary<int, DelaunayCell<VERTEX>> neighbourCell = new Dictionary<int, DelaunayCell<VERTEX>>();

            // for (int i = 0; i < delaunay.Vertices.Count; i++)
            // {

            //     cells.Clear();

            //     VERTEX vertex = delaunay.Vertices[i];

            //     for (int j = 0; j < delaunay->Vertices.size(); j++)
            //     {
            //         Simplex<VERTEX> simplex = delaunay->Cells[j].Simplex;

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
            //         VoronoiRegion<VERTEX> region = new VoronoiRegion<VERTEX>();

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
            //             Simplex<VERTEX> simplex = cells[j].Simplex;

            //             for (int k = 0; k < simplex.Adjacent.Length; k++)
            //             {
            //                 if (simplex.Adjacent[k] == null)
            //                     continue;

            //                 int tag = simplex.Adjacent[k].Tag;

            //                 if (neighbourCell.ContainsKey(tag))
            //                 {
            //                     VoronoiEdge<VERTEX> edge = new VoronoiEdge<VERTEX>(cells[j], neighbourCell[tag]);
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