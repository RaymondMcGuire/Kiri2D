/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-05-26 12:33:12
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\power_diagram.h
 */

#ifndef _KIRI_POWER_DIAGRAM_H_
#define _KIRI_POWER_DIAGRAM_H_

#pragma once

#include <kiri2d/geo/convex_hull3.h>
#include <kiri2d/voronoi/voro_site.h>

namespace KIRI
{

    class KiriPowerDiagram : public KiriVertex3
    {
    public:
        explicit KiriPowerDiagram::KiriPowerDiagram()
        {
            mConvexHull = std::make_unique<KiriConvexHull3>();
            mConvexClip = std::make_shared<KiriConvexClip2>();
        }

        ~KiriPowerDiagram() noexcept {}

        const Vector<KiriVoroSitePtr> &GetVoroSites() const { return mVoroSites; }
        const Vector<KiriVoroSitePtr> &GetBoundaryVoroSites() const { return mBoundaryVoroSites; }

        void AddVoroSite(const Vector2F &pos) { mVoroSites.emplace_back(std::make_shared<KiriVoroSite>(pos.x, pos.y)); }
        void AddPowerSite(const Vector2F &pos, float weight) { mVoroSites.emplace_back(std::make_shared<KiriVoroSite>(pos.x, pos.y, weight)); }

        void SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary);

        void PermutateVoroSites();
        void ComputeDiagram();

        void PrintVoroSites();

    private:
        Vector<bool> mVisitedVoroSites;
        Vector<KiriVoroSitePtr> mVoroSites;
        Vector<KiriVoroSitePtr> mBoundaryVoroSites;
        Vector<KiriFace3Ptr> mFacetsAroundVertex;
        KiriConvexClip2Ptr mConvexClip;
        KiriConvexHull3Ptr mConvexHull;
        KiriVoroCellPolygon2Ptr mBoundaryPolygon2;

        void ComputeVoroCells();
        void ComputeFacetsAroundVertex(const KiriEdge3Ptr &edge);

        /*** 
         * @description: Uses the linear time algorithm of O'Rourke to compute the intersection of two convex polygons.
         * @param {const KiriVoroCellPolygon2Ptr} &vcp1
         * @param {const KiriVoroCellPolygon2Ptr} &vcp2
         * @return {*}
         */
        KiriVoroCellPolygon2Ptr VoroCellPolygonClip(const KiriVoroCellPolygon2Ptr &vcp1, const KiriVoroCellPolygon2Ptr &vcp2);
    };

    typedef SharedPtr<KiriPowerDiagram> KiriPowerDiagramPtr;
}
#endif