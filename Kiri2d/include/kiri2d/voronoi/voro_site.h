/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-05-26 17:56:10
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_site.h
 */

#ifndef _KIRI_VORO_SITE_H_
#define _KIRI_VORO_SITE_H_

#pragma once

#include <kiri2d/data/vertex3.h>
#include <kiri2d/voronoi/voro_cell_polygon2.h>

namespace KIRI
{

    class KiriVoroSite : public KiriVertex3
    {
    public:
        explicit KiriVoroSite::KiriVoroSite()
            : KiriVoroSite(0.f, 0.f, 0.f) {}

        explicit KiriVoroSite::KiriVoroSite(float x, float y)
            : KiriVoroSite(x, y, MEpsilon<float>()) {}

        explicit KiriVoroSite::KiriVoroSite(float x, float y, float weight)
            : KiriVertex3(Vector3F(x, y, ProjectZ(x, y, weight)))
        {
            mWeight = weight;
        }

        virtual ~KiriVoroSite() noexcept {}

        constexpr float GetWeight() const { return mWeight; }
        const KiriVoroCellPolygon2Ptr &GetCellPolygon() const { return mVoroCellPolygon2; }

        void ResetValue(const Vector3F &val)
        {
            SetValue(Vector3F(val.x, val.y, ProjectZ(val.x, val.y, val.z)));
            mNeighborSites.clear();
            mLastNeighborSites.clear();
            mVoroCellPolygon2->Reset();
        }

        void SetCellPolygon(const KiriVoroCellPolygon2Ptr &polygon)
        {
            mVoroCellPolygon2 = polygon;
        }
        void SetAsBoundarySite() { mBoundarySite = true; }
        void SetNeighborSites(const Vector<SharedPtr<KiriVoroSite>> &neighbor)
        {
            SetLastNeighborSites(mNeighborSites);
            mNeighborSites = neighbor;
        }
        void SetLastNeighborSites(const Vector<SharedPtr<KiriVoroSite>> &lastNeighbor) { mLastNeighborSites = lastNeighbor; }

        const bool IsBoundarySite() const { return mBoundarySite; }

    private:
        float ProjectZ(float x, float y, float weight) { return (x * x + y * y - weight); }

        float mWeight;
        bool mBoundarySite = false;

        Vector<SharedPtr<KiriVoroSite>> mNeighborSites;
        Vector<SharedPtr<KiriVoroSite>> mLastNeighborSites;

        KiriVoroCellPolygon2Ptr mVoroCellPolygon2;
    };

    typedef SharedPtr<KiriVoroSite> KiriVoroSitePtr;
}
#endif