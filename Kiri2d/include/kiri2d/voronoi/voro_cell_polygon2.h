/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-06-16 16:29:38
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_cell_polygon2.h
 */

#ifndef _KIRI_VORO_CELL_POLYGON2_H_
#define _KIRI_VORO_CELL_POLYGON2_H_

#pragma once

#include <kiri2d/geo/convex_clip2.h>

namespace KIRI
{
    struct IntersectionStatus
    {
        bool intersection = false;
        Vector<Vector4F> shrink;
    };

    class KiriVoroCellPolygon2
    {
    public:
        explicit KiriVoroCellPolygon2()
        {
            mVoroSitesList = std::make_shared<KiriVector2List>();
        }

        ~KiriVoroCellPolygon2() noexcept {}

        const KiriVector2ListPtr &GetVoroSitesList() const { return mVoroSitesList; }
        const BoundingBox2F &GetBBox() const { return mBBox2; }
        const Vector<Vector2F> &GetPolygonVertices() const { return mPolygonVertices2; }
        void AddPolygonVertex2(const Vector2F &v2) { mPolygonVertices2.emplace_back(Vector2F(v2)); }

        void Print();
        void PrintPolyVertices();
        void PrintVoroSitesList();
        void UpdateBBox();
        void ComputeVoroSitesList();

        UInt GetLength() const { return mPolygonVertices2.size(); }

        void Reset()
        {
            mBBox2.reset();
            mVoroSitesList->RemoveAll();
            mPolygonVertices2.clear();
            mBisectors.clear();
            mShrinks.clear();
            mSkeletons.clear();
        }

        bool CheckBBox();
        bool Contains(const Vector2F &v);

        Vector2F GetRndInnerPoint();

        /*** 
         * @description: Nürnberg, R. (2013). Calculating the volume and centroid of a polyhedron in 3d. 
         * @url:http://paulbourke.net/geometry/polygonmesh/centroid.pdf
         * @param {*}
         * @return {*}
         */
        float GetPolygonArea();

        /*** 
         * @description: Nürnberg, R. (2013). Calculating the volume and centroid of a polyhedron in 3d. 
         * @url:http://paulbourke.net/geometry/polygonmesh/centroid.pdf
         * @param {*}
         * @return {*}
         */
        Vector2F GetPolygonCentroid();

        float ComputeMinDisInPoly(const Vector2F &p);

        bool IsClockwise(const Vector<Vector4F> &poly);

        IntersectionStatus ComputeShrink(const Vector<Vector4F> &poly, float lambda);
        void ComputeStraightSkeleton(float lambda);
        const Vector<Vector4F> &GetShrinks() const { return mShrinks; }
        const Vector<Vector4F> &GetSkeletons() const { return mSkeletons; }

        Vector3F ComputeMICByStraightSkeleton();

    private:
        BoundingBox2F mBBox2;
        KiriVector2ListPtr mVoroSitesList;
        Vector<Vector2F> mPolygonVertices2;

        Vector<Vector2F> mBisectors;
        Vector<Vector4F> mShrinks, mSkeletons;

        void ComputeBisectors(const Vector<Vector4F> &poly, float lambda);
    };

    typedef SharedPtr<KiriVoroCellPolygon2> KiriVoroCellPolygon2Ptr;
}
#endif