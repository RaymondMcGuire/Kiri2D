/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-05-25 23:44:54
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri\KiriCore\include\kiri2d\voronoi\voro_cell_polygon2.h
 */

#ifndef _KIRI_VORO_CELL_POLYGON2_H_
#define _KIRI_VORO_CELL_POLYGON2_H_

#pragma once

#include <kiri2d/geo/convex_clip2.h>

namespace KIRI
{

    class KiriVoroCellPolygon2
    {
    public:
        explicit KiriVoroCellPolygon2::KiriVoroCellPolygon2()
        {

            mVoroSitesList = std::make_shared<KiriVector2List>();
        }

        ~KiriVoroCellPolygon2() noexcept {}

        const KiriVector2ListPtr &GetVoroSitesList() const { return mVoroSitesList; }
        const BoundingBox2F &GetBBox() const { return mBBox2; }
        const Vector<Vector2F> &GetPolygonVertices() const { return mPolygonVertices2; }
        void AddPolygonVertex2(const Vector2F &v2) { mPolygonVertices2.emplace_back(Vector2F(v2)); }

        void Print();
        void UpdateBBox();
        void ComputeVoroSitesList();

    private:
        BoundingBox2F mBBox2;
        KiriVector2ListPtr mVoroSitesList;
        Vector<Vector2F> mPolygonVertices2;
    };

    typedef SharedPtr<KiriVoroCellPolygon2> KiriVoroCellPolygon2Ptr;
}
#endif