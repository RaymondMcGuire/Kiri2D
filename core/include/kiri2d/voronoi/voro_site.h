/***
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-10-04 11:21:59
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
            : KiriVoroSite(x, y, std::numeric_limits<float>::epsilon()) {}

        explicit KiriVoroSite(Vector2F pos)
            : KiriVoroSite(pos.x, pos.y, std::numeric_limits<float>::epsilon()) {}

        explicit KiriVoroSite(Vector2F pos, float percent)
            : KiriVoroSite(pos.x, pos.y, std::numeric_limits<float>::epsilon(), percent) {}

        explicit KiriVoroSite(Vector3F data)
            : KiriVoroSite(data.x, data.y, data.z) {}

        explicit KiriVoroSite(Vector4F data)
            : KiriVoroSite(data.x, data.y, data.z)
        {
            mRadius = data.w;
        }

        explicit KiriVoroSite(float x, float y, float weight)
            : KiriVertex3(Vector3F(x, y, ProjectZ(x, y, weight)))
        {
            mWeight = weight;
            mPercentage = std::numeric_limits<float>::epsilon();
            mRadius = std::numeric_limits<float>::epsilon();
        }

        explicit KiriVoroSite(float x, float y, float weight, float percent)
            : KiriVertex3(Vector3F(x, y, ProjectZ(x, y, weight)))
        {
            mWeight = weight;
            mPercentage = percent;
            mRadius = std::numeric_limits<float>::epsilon();
        }

        virtual ~KiriVoroSite() {}

        void setWeight(float weight)
        {
            auto v = GetValue();
            mWeight = weight;
            SetValue(Vector3F(v.x, v.y, ProjectZ(v.x, v.y, weight)));
        }

        void SetPercentage(float percent) { mPercentage = percent; }
        void setRadius(float radius) { mRadius = radius; }

        constexpr float weight() const { return mWeight; }
        constexpr float GetPercentage() const { return mPercentage; }
        constexpr float radius() const { return mRadius; }

        const KiriVoroCellPolygon2Ptr &GetCellPolygon() const { return mVoroCellPolygon2; }

        const Vector<SharedPtr<KiriVoroSite>> &GetNeighborSites() const { return neighbors; }

        float GetDistance2(const SharedPtr<KiriVoroSite> &site)
        {
            auto dx = GetValue().x - site->GetValue().x;
            auto dy = GetValue().y - site->GetValue().y;
            return std::sqrt(dx * dx + dy * dy);
        }

        void ResetWeight() { mWeight = 0.f; }

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
            SetLastNeighborSites(neighbors);
            neighbors = neighbor;
        }
        void SetLastNeighborSites(const Vector<SharedPtr<KiriVoroSite>> &lastNeighbor) { mLastNeighborSites = lastNeighbor; }

        const bool IsBoundarySite() const { return mBoundarySite; }

        const bool IsEnable() const { return mEnable; }

        void Enable() { mEnable = true; }
        void Disable() { mEnable = false; }

        void SetFreeze(bool freeze) { mIsFrozen = freeze; }
        bool GetIsFrozen() { return mIsFrozen; }

        void PrintSite()
        {
            KIRI_LOG_DEBUG("site idx={0}", GetIdx());
            KIRI_LOG_DEBUG("site weight={0}, percentage={1}", mWeight, mPercentage);
        }

    private:
        float ProjectZ(float x, float y, float weight) { return (x * x + y * y - weight); }

        float mWeight, mPercentage, mRadius;

        bool mEnable = true;
        bool mIsFrozen = false;
        bool mBoundarySite = false;

        Vector<SharedPtr<KiriVoroSite>> neighbors;
        Vector<SharedPtr<KiriVoroSite>> mLastNeighborSites;

        KiriVoroCellPolygon2Ptr mVoroCellPolygon2;
    };

    typedef SharedPtr<KiriVoroSite> KiriVoroSitePtr;
}
#endif