/***
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:49:01
 * @LastEditTime: 2021-06-10 17:10:18
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\geo\convex_hull3.h
 */

#ifndef _KIRI_CONVEX_HULL3_H_
#define _KIRI_CONVEX_HULL3_H_

#pragma once

#include <kiri2d/geo/face_conflict_list.h>
#include <kiri2d/geo/vertex_conflict_list.h>

namespace KIRI
{
    class KiriConvexHull3
    {
    public:
        KiriConvexHull3()
        {
        }

        ~KiriConvexHull3()
        {
        }

        void AddVertex(const Vector3F &v3);
        void AddVertex(const KiriVertex3Ptr &v3);
        void AddFacet(const KiriFace3Ptr &f3);

        UInt GetFaceEdgeId(const KiriFace3Ptr &f, const KiriVertex3Ptr &a, const KiriVertex3Ptr &b);
        void LinkEdge(UInt eAId, UInt eBId);
        void LinkEdge(const KiriEdge3Ptr &eA, const KiriEdge3Ptr &eB);
        bool LinkFace(const KiriFace3Ptr &fA, const KiriFace3Ptr &fB, const KiriVertex3Ptr &a, const KiriVertex3Ptr &b);
        bool LinkFaceEdge(const KiriFace3Ptr &f, const KiriEdge3Ptr &e);

        void ResetFace(const KiriFace3Ptr &f, UInt idx);
        void CleanTwinEdge(const KiriEdge3Ptr &e);

        const Vector<KiriFace3Ptr> &GetFacets() const { return mCurFacets; }
        const Vector<KiriEdge3Ptr> &GetEdges() const { return mEdges; }
        const Vector<KiriVertex3Ptr> &vertices() const { return mVertices; }
        const Vector<KiriEdge3Ptr> &GetHorizonEdges() const { return mHorizonEdges; }

        const KiriEdge3Ptr &GetEdgeById(UInt id) const { return mEdges[id]; }

        const UInt GetNumOfVertices() const { return mVertices.size(); }
        const UInt GetNumOfFacets() const { return mCurFacets.size(); }

        void FindHorizonEdgeList(const KiriEdge3Ptr &edge);
        const bool IsHorizonEdge(const KiriEdge3Ptr &edge);

        bool ComputeConvexHull();

        void reset();

        void PrintVertexInfo();
        void PrintCurFacetsInfo();
        void PrintConflictGraphInfo();

    private:
        UInt mProcessedCount = 0;

        Vector<KiriVertex3Ptr> mVertices;
        Vector<KiriEdge3Ptr> mEdges;
        Vector<KiriEdge3Ptr> mHorizonEdges;
        Vector<KiriFace3Ptr> mCurFacets;
        Vector<KiriFace3Ptr> mVisFacets;
        Vector<KiriFace3Ptr> mCreatedFacets;

        Vector<KiriFaceConflictListsPtr> mFConflict;
        Vector<KiriVertexConflictListsPtr> mVConflict;

        void BuildTetrahedron();
        void BuildConflictGraph(KiriFace3Ptr &f, KiriVertex3Ptr &v);
        void BuildConflictGraphByHorizonEdge(KiriFace3Ptr &hef1, KiriFace3Ptr &hef2, KiriFace3Ptr &f);
        void RemoveConflictGraphByFace(KiriFace3Ptr &f);
    };
    typedef UniquePtr<KiriConvexHull3> KiriConvexHull3Ptr;
} // namespace KIRI

#endif /* _KIRI_CONVEX_HULL3_H_ */