/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-06-13 23:50:07
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
        explicit KiriVoroSite()
            : KiriVoroSite(0.f, 0.f, 0.f) {}

        explicit KiriVoroSite(float x, float y)
            : KiriVoroSite(x, y, MEpsilon<float>()) {}

        explicit KiriVoroSite(Vector2F pos)
            : KiriVoroSite(pos.x, pos.y, MEpsilon<float>()) {}

        explicit KiriVoroSite(Vector2F pos, float percent)
            : KiriVoroSite(pos.x, pos.y, MEpsilon<float>(), percent) {}

        explicit KiriVoroSite(float x, float y, float weight)
            : KiriVertex3(Vector3F(x, y, ProjectZ(x, y, weight)))
        {
            mWeight = weight;
            mPercentage = MEpsilon<float>();
            mRadius = MEpsilon<float>();
        }

        explicit KiriVoroSite(float x, float y, float weight, float percent)
            : KiriVertex3(Vector3F(x, y, ProjectZ(x, y, weight)))
        {
            mWeight = weight;
            mPercentage = percent;
            mRadius = MEpsilon<float>();
        }

        virtual ~KiriVoroSite() noexcept {}

        void SetWeight(float weight)
        {
            auto v = GetValue();
            mWeight = weight;
            SetValue(Vector3F(v.x, v.y, ProjectZ(v.x, v.y, weight)));
        }

        void SetPercentage(float percent) { mPercentage = percent; }
        void SetRadius(float radius) { mRadius = radius; }

        constexpr float GetWeight() const { return mWeight; }
        constexpr float GetPercentage() const { return mPercentage; }
        constexpr float GetRadius() const { return mRadius; }

        const KiriVoroCellPolygon2Ptr &GetCellPolygon() const { return mVoroCellPolygon2; }

        const Vector<SharedPtr<KiriVoroSite>> &GetNeighborSites() const { return mNeighborSites; }

        float GetDistance2(const SharedPtr<KiriVoroSite> &site)
        {
            auto dx = GetValue().x - site->GetValue().x;
            auto dy = GetValue().y - site->GetValue().y;
            return std::sqrt(dx * dx + dy * dy);
        }

        void ResetValue(const Vector2F &val)
        {
            SetValue(Vector3F(val.x, val.y, ProjectZ(val.x, val.y, mWeight)));
        }

        void ResetValue(const Vector3F &val)
        {
            mWeight = val.z;
            SetValue(Vector3F(val.x, val.y, ProjectZ(val.x, val.y, val.z)));
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

        void PrintSite()
        {
            KIRI_LOG_DEBUG("site idx={0}", GetIdx());
            KIRI_LOG_DEBUG("site weight={0}, percentage={1}", mWeight, mPercentage);
        }

    private:
        float ProjectZ(float x, float y, float weight) { return (x * x + y * y - weight); }

        float mWeight, mPercentage, mRadius;
        bool mBoundarySite = false;

        Vector<SharedPtr<KiriVoroSite>> mNeighborSites;
        Vector<SharedPtr<KiriVoroSite>> mLastNeighborSites;

        KiriVoroCellPolygon2Ptr mVoroCellPolygon2;
    };

    typedef SharedPtr<KiriVoroSite> KiriVoroSitePtr;
}
#endif