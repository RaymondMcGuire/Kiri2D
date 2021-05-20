/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:33:21
 * @LastEditTime: 2021-05-18 22:54:51
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\geo\convex_hull3.cpp
 */

#include <kiri2d/geo/convex_hull3.h>

namespace KIRI2D
{

    void KiriConvexHull3::PrintVertexInfo()
    {
        KIRI_LOG_DEBUG("vertex info: num = {0}", mVertices.size());
        for (size_t i = 0; i < mVertices.size(); i++)
        {
            KIRI_LOG_DEBUG("vertex value:({0},{1},{2})", mVertices[i].GetValue().x, mVertices[i].GetValue().y, mVertices[i].GetValue().z);
        }
    }

    void KiriConvexHull3::PrintCurFacetsInfo()
    {
        KIRI_LOG_DEBUG("current facets info: num = {0}", mCurFacets.size());
        for (size_t i = 0; i < mCurFacets.size(); i++)
        {
            KIRI_LOG_DEBUG("current facets value: idx={0}", mCurFacets[i].GetIdx());
            mCurFacets[i].PrintFaceInfo();
        }
    }

    void KiriConvexHull3::PrintConflictGraphInfo()
    {
        KIRI_LOG_DEBUG("----------CONFLICT GRAPH INFO----------");

        for (size_t i = 0; i < mFConflict.size(); i++)
            mFConflict[i].PrintFaceConflictList();

        for (size_t i = 0; i < mVConflict.size(); i++)
            mVConflict[i].PrintVertexConflictList();

        KIRI_LOG_DEBUG("---------------------------------------");
    }

    void KiriConvexHull3::ComputeConvexHull()
    {
        BuildTetrahedron();

        while (mProcessedCount < mVertices.size())
        {
            auto p_r = mVertices[mProcessedCount];

            // vertex in convex hull
            if (mVConflict[p_r.GetIdx()].Size() == 0)
            {
                ++mProcessedCount;
                continue;
            }

            // outside
            mVisFacets.clear();
            mVConflict[p_r.GetIdx()].BuildVisibleList(mVisFacets);

            // find horizon edges
            for (size_t i = 0; i < mVisFacets.size(); i++)
            {
                auto vf = mVisFacets[i];
                for (size_t j = 0; j < 3; j++)
                {
                    auto vfe = vf.GetEdgesByIdx(j);
                    if (vfe->GetTwinEdge() != NULL)
                    {
                        // twin edge
                        auto te = vfe->GetTwinEdge();
                        // twin edge face
                        auto tef = mCurFacets[te->GetId() / 3];
                        if (!tef.GetVisible())
                            mHorizonEdges.emplace_back(te);
                    }
                }
            }

            // create new facets
            for (size_t i = 0; i < mHorizonEdges.size(); i++)
            {
                KIRI_LOG_DEBUG("start={0},{1},{2}/ end={3},{4},{5}",
                               mHorizonEdges[i]->GetOriginVertex().GetValue().x, mHorizonEdges[i]->GetOriginVertex().GetValue().y, mHorizonEdges[i]->GetOriginVertex().GetValue().z,
                               mHorizonEdges[i]->GetDestVertex().GetValue().x, mHorizonEdges[i]->GetDestVertex().GetValue().y, mHorizonEdges[i]->GetDestVertex().GetValue().z);
                // auto newf = KiriFace3(p_r, mHorizonEdges[i]->GetOriginVertex(), mHorizonEdges[i]->GetDestVertex(), mHorizonEdges[i]->GetTwinEdge()->GetNextEdge()->GetDestVertex());
                // AddFacet(newf);
                // mCreatedFacets.emplace_back(newf);
            }

            ++mProcessedCount;
        }
    }

    void KiriConvexHull3::AddVertex(Vector3F v3)
    {
        auto vert = KiriVertex3(v3);
        vert.SetIdx(mVertices.size());
        mVertices.emplace_back(vert);
        mVConflict.emplace_back(KiriVertexConflictLists());
    }

    void KiriConvexHull3::AddFacet(KiriFace3 &f3)
    {
        f3.SetIdx(mCurFacets.size());
        mCurFacets.emplace_back(f3);
        mFConflict.emplace_back(KiriFaceConflictLists());
    }

    void KiriConvexHull3::BuildConflictGraph(KiriFace3 f, KiriVertex3 v)
    {
        mFConflict[f.GetIdx()].Push(v);
        mVConflict[v.GetIdx()].Push(f);
    }

    void KiriConvexHull3::BuildTetrahedron()
    {
        // check
        if (mVertices.size() <= 3)
        {
            KIRI_LOG_ERROR("BuildTetrahedron ERROR: A tetrahedron needs at least 4 points!");
            return;
        }

        // randomize

        // reset indices
        for (size_t i = 0; i < mVertices.size(); i++)
        {
            mVertices[i].SetIdx(i);
        }

        // find 4 linear indepent vertex
        bool linear_dependent = true;
        KiriVertex3 v0, v1, v2, v3;
        v0 = mVertices[0];
        v1 = mVertices[1];

        for (size_t i = 2; i < mVertices.size(); i++)
        {
            // not linearly dependent
            if (!(v0.LinearDependent(mVertices[i].GetValue()) && v1.LinearDependent(mVertices[i].GetValue())))
            {
                KIRI_LOG_DEBUG("not linearly dependent, i={0}", i);
                v2 = mVertices[i];
                linear_dependent = false;
                if (i != 2)
                {
                    v2.SetIdx(2);
                    mVertices[2].SetIdx(i);
                    mVertices[i] = mVertices[2];
                    mVertices[2] = v2;
                }
                break;
            }
        }

        // check exist linearly independent or not
        if (linear_dependent)
        {
            KIRI_LOG_ERROR("BuildTetrahedron ERROR: Cannot find enough linearly independent points!");
            return;
        }

        bool inside_face = true;
        auto f0 = KiriFace3(v0, v1, v2);
        for (size_t i = 3; i < mVertices.size(); i++)
        {
            if ((f0.GetNormal().dot(f0.GetVertexByIdx(0).GetValue()) != f0.GetNormal().dot(mVertices[i].GetValue())))
            {
                v3 = mVertices[i];
                inside_face = false;
                if (i != 3)
                {
                    v3.SetIdx(3);
                    mVertices[3].SetIdx(i);
                    mVertices[i] = mVertices[3];
                    mVertices[3] = v3;
                }
                break;
            }
        }

        // check v4 lie in face f0
        if (inside_face)
        {
            KIRI_LOG_ERROR("BuildTetrahedron ERROR: Cannot find enough non-planar points!");
            return;
        }

        f0.OrientFace(v3);
        auto f1 = KiriFace3(v0, v2, v3, v1);
        auto f2 = KiriFace3(v0, v1, v3, v2);
        auto f3 = KiriFace3(v1, v2, v3, v0);

        AddFacet(f0);
        AddFacet(f1);
        AddFacet(f2);
        AddFacet(f3);
        mProcessedCount = 4;

        // connect facets
        f0.LinkFace(f1, v0, v2);
        f0.LinkFace(f2, v0, v1);
        f0.LinkFace(f3, v1, v2);
        f1.LinkFace(f2, v0, v3);
        f1.LinkFace(f3, v2, v3);
        f2.LinkFace(f3, v3, v1);

        // build conflict graph
        for (size_t i = mProcessedCount; i < mVertices.size(); i++)
        {
            auto p_t = mVertices[i];

            if (f0.CheckConflict(p_t.GetValue()))
                BuildConflictGraph(f0, p_t);

            if (f1.CheckConflict(p_t.GetValue()))
                BuildConflictGraph(f1, p_t);

            if (f2.CheckConflict(p_t.GetValue()))
                BuildConflictGraph(f2, p_t);

            if (f3.CheckConflict(p_t.GetValue()))
                BuildConflictGraph(f3, p_t);
        }
    }
}