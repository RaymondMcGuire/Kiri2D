/***
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-10-26 16:07:06
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

        ~KiriVoroCellPolygon2() {}

        const KiriVector2ListPtr &GetVoroSitesList() const { return mVoroSitesList; }
        const BoundingBox2F &GetBBox() const { return mBBox2; }
        const Vector<Vector2F> &GetPolygonVertices() const { return mPolygonVertices2; }
        void AddPolygonVertex2(const Vector2F &v2) { mPolygonVertices2.emplace_back(Vector2F(v2)); }
        void PopPolygonVertex2() { mPolygonVertices2.pop_back(); }
        void ReversePolygonVertex2() { std::reverse(mPolygonVertices2.begin(), mPolygonVertices2.end()); }

        void Print();
        void PrintPolyVertices();
        void PrintVoroSitesList();
        void UpdateBBox();
        void ComputeVoroSitesList();

        void SetColor(Vector3F color) { mColor = color; }
        Vector3F GetColor() { return mColor; }

        UInt GetLength() const { return mPolygonVertices2.size(); }

        void reset()
        {
            mBBox2.reset();
            mVoroSitesList->removeAll();
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

        const Vector<Vector4F> &GetSkeletons() const { return mSkeletons; }

        // SSkel_1998
        void ComputeSSkel1998Convex();

        Vector3F ComputeMICByStraightSkeleton();
        Vec_Vec3F ComputeMICByStraightSkeletonTest();

        Vec_Vec3F ComputeAllCByStraightSkeleton();

    private:
        BoundingBox2F mBBox2;
        KiriVector2ListPtr mVoroSitesList;
        Vector<Vector2F> mPolygonVertices2;

        Vector<Vector2F> mBisectors;
        Vector<Vector4F> mShrinks, mSkeletons;

        Vector3F mColor;
    };

    typedef SharedPtr<KiriVoroCellPolygon2> KiriVoroCellPolygon2Ptr;
}
#endif