/***
 * @Author: Xu.WANG
 * @Date: 2021-12-21 13:08:13
 * @LastEditTime: 2021-12-21 13:10:12
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\hdv_toolkit\voronoi\voronoi_site.h
 */

#ifndef _HDV_VORONOI_SITE_H_
#define _HDV_VORONOI_SITE_H_

#pragma once

#include <kiri2d/hdv_toolkit/voronoi/voronoi_cell_polygon.h>
#include <kiri2d/hdv_toolkit/primitives/vertex2.h>
#include <kiri2d/hdv_toolkit/primitives/vertex3.h>

namespace HDV::Voronoi
{
    class VoronoiSite2 : public HDV::Primitives::Vertex2
    {
    public:
        explicit VoronoiSite2() : HDV::Primitives::Vertex2() {}
        explicit VoronoiSite2(int id) : HDV::Primitives::Vertex2(id) {}
        explicit VoronoiSite2(float x, float y) : HDV::Primitives::Vertex2(x, y) {}
        explicit VoronoiSite2(float x, float y, int id) : HDV::Primitives::Vertex2(x, y, id) {}

        virtual ~VoronoiSite2() noexcept {}

        std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> CellPolygon;
    };
    typedef std::shared_ptr<VoronoiSite2> VoronoiSite2Ptr;
} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_SITE_H_ */