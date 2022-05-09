/***
 * @Author: Xu.WANG
 * @Date: 2021-12-30 11:28:04
 * @LastEditTime: 2022-02-09 12:48:22
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
    class VoronoiSite2
        : public HDV::Primitives::Vertex2
    {
    public:
        explicit VoronoiSite2()
            : HDV::Primitives::Vertex2()
        {
        }

        explicit VoronoiSite2(int id)
            : HDV::Primitives::Vertex2(id)
        {
        }

        explicit VoronoiSite2(double x, double y)
            : HDV::Primitives::Vertex2(x, y)
        {
        }

        explicit VoronoiSite2(double x, double y, int id)
            : HDV::Primitives::Vertex2(x, y, id)
        {
        }

        virtual ~VoronoiSite2()
        {
        }

        std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> &cellPolygon()
        {
            return mCellPolygon;
        }

    private:
        std::shared_ptr<VoronoiCellPolygon<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2>> mCellPolygon;
    };
    typedef std::shared_ptr<VoronoiSite2> VoronoiSite2Ptr;

    class CapacityVoronoiSite2 : public VoronoiSite2
    {
    public:
        explicit CapacityVoronoiSite2()
            : VoronoiSite2()
        {
        }

        explicit CapacityVoronoiSite2(int id)
            : VoronoiSite2(id)
        {
        }

        explicit CapacityVoronoiSite2(double x, double y)
            : VoronoiSite2(x, y)
        {
        }

        explicit CapacityVoronoiSite2(double x, double y, int id)
            : VoronoiSite2(x, y, id)
        {
        }

        virtual ~CapacityVoronoiSite2()
        {
        }

        constexpr double capacity() const
        {
            return mCapacity;
        }

    private:
        double mCapacity = 0.0;
    };
    typedef std::shared_ptr<CapacityVoronoiSite2> CapacityVoronoiSite2Ptr;

    class VoronoiSite3 : public HDV::Primitives::Vertex3
    {
    public:
        explicit VoronoiSite3()
            : HDV::Primitives::Vertex3()
        {
        }

        explicit VoronoiSite3(int id)
            : HDV::Primitives::Vertex3(id)
        {
        }

        explicit VoronoiSite3(double x, double y, double z)
            : HDV::Primitives::Vertex3(x, y, z)
        {
        }

        explicit VoronoiSite3(double x, double y, double z, int id)
            : HDV::Primitives::Vertex3(x, y, z, id)
        {
        }

        virtual ~VoronoiSite3()
        {
        }

        std::shared_ptr<VoronoiPolygon3> &polygon()
        {
            return mPolygon;
        }

    private:
        std::shared_ptr<VoronoiPolygon3> mPolygon;
    };
    typedef std::shared_ptr<VoronoiSite3> VoronoiSite3Ptr;
} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_SITE_H_ */