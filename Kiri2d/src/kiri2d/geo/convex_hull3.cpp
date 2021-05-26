/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:33:21
 * @LastEditTime: 2021-05-26 17:27:46
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\geo\convex_hull3.cpp
 */

#include <kiri2d/geo/convex_hull3.h>

namespace KIRI
{

    void KiriConvexHull3::PrintVertexInfo()
    {
        KIRI_LOG_DEBUG("vertex info: num = {0}", mVertices.size());
        for (size_t i = 0; i < mVertices.size(); i++)
        {
            KIRI_LOG_DEBUG("vertex idx:{0}", mVertices[i]->GetIdx());
            KIRI_LOG_DEBUG("vertex value:({0},{1},{2})", mVertices[i]->GetValue().x, mVertices[i]->GetValue().y, mVertices[i]->GetValue().z);
        }
    }

    void KiriConvexHull3::PrintCurFacetsInfo()
    {
        KIRI_LOG_DEBUG("current facets info: num = {0}", mCurFacets.size());
        for (size_t i = 0; i < mCurFacets.size(); i++)
        {
            KIRI_LOG_DEBUG("current facets value: idx={0}", mCurFacets[i]->GetIdx());
            mCurFacets[i]->PrintFaceInfo();
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

    const bool KiriConvexHull3::IsHorizonEdge(const KiriEdge3Ptr &edge)
    {
        auto twin = edge->GetTwinEdge();
        if (twin != NULL)
        {
            // twin edge face
            auto tef = mCurFacets[twin->GetId() / 3];
            auto ef = mCurFacets[edge->GetId() / 3];
            if (tef->GetVisible() && !ef->GetVisible())
                return true;
        }
        return false;
    }

    void KiriConvexHull3::Reset()
    {
        mProcessedCount = 0;
        mVertices.clear();
        mHorizonEdges.clear();
        mCurFacets.clear();
        mVisFacets.clear();
        mCreatedFacets.clear();
        mFConflict.clear();
        mVConflict.clear();
    }

    void KiriConvexHull3::FindHorizonEdgeList(const KiriEdge3Ptr &edge)
    {
        if (IsHorizonEdge(edge))
        {
            if (mHorizonEdges.size() > 0 && edge->IsEqual(mHorizonEdges[0]))
                return;
            else
            {
                //edge->PrintEdgeInfo();
                //KIRI_LOG_DEBUG("Current edge idx={0}, next edge idx={1}", edge->GetId(), edge->GetNextEdge()->GetId());
                mHorizonEdges.emplace_back(edge);
                FindHorizonEdgeList(edge->GetNextEdge());
            }
        }
        else if (edge->GetTwinEdge() != NULL)
        {
            //edge->PrintEdgeInfo();
            //KIRI_LOG_DEBUG("Current edge idx={0}, twin={1}, twin next edge idx={2}", edge->GetId(), edge->GetTwinEdge()->GetId(), edge->GetTwinEdge()->GetNextEdge()->GetId());
            FindHorizonEdgeList(edge->GetTwinEdge()->GetNextEdge());
        }
    }

    void KiriConvexHull3::ComputeConvexHull()
    {
        BuildTetrahedron();

        while (mProcessedCount < mVertices.size())
        {

            // for (size_t i = 0; i < mCurFacets.size(); i++)
            // {
            //     mCurFacets[i]->PrintFaceInfo();
            // }

            auto p_r = mVertices[mProcessedCount];

            // vertex in convex hull
            if (mVConflict[p_r->GetIdx()].Size() == 0)
            {
                //KIRI_LOG_DEBUG("Current idx={0}, no conlict!", mProcessedCount);
                ++mProcessedCount;
                continue;
            }

            mVisFacets.clear();
            mCreatedFacets.clear();
            mHorizonEdges.clear();

            // outside
            mVConflict[p_r->GetIdx()].BuildVisibleList(mVisFacets);
            //KIRI_LOG_DEBUG("mVConflict p_r idx={0}, num of conflict ={1}!", mProcessedCount, mVConflict[mProcessedCount].Size());
            // KIRI_LOG_DEBUG("mVConflict p_r idx={0}, num of conflict ={1}!", 5, mVConflict[5].Size());

            // find horizon edges
            //KIRI_LOG_DEBUG("mCurFacets size={0}!", mCurFacets.size());
            //KIRI_LOG_DEBUG("--------------------------------------FIND HORIZON EDGES");
            bool find_horizon_edges = false;
            for (size_t i = 0; i < mVisFacets.size(); i++)
            {
                if (find_horizon_edges)
                    break;

                // KIRI_LOG_DEBUG("mVisFacets idx={0}!", mVisFacets[i]->GetIdx());
                auto vf = mVisFacets[i];
                for (size_t j = 0; j < 3; j++)
                {
                    auto vfe = vf->GetEdgesByIdx(j);
                    if (vfe->GetTwinEdge() != NULL)
                    {
                        auto te = vfe->GetTwinEdge();
                        if (IsHorizonEdge(te))
                        {
                            FindHorizonEdgeList(vfe);
                            find_horizon_edges = true;
                            break;
                        }
                    }
                }
            }

            //KIRI_LOG_DEBUG("mHorizonEdges size={0}!", mHorizonEdges.size());
            // create new facets
            KiriFace3Ptr first, last;
            for (size_t i = 0; i < mHorizonEdges.size(); i++)
            {
                //mHorizonEdges[i]->PrintEdgeInfo();
                auto newf = std::make_shared<KiriFace3>(p_r, mHorizonEdges[i]->GetOriginVertex(), mHorizonEdges[i]->GetDestVertex(), mHorizonEdges[i]->GetTwinEdge()->GetNextEdge()->GetDestVertex());
                AddFacet(newf);
                mCreatedFacets.emplace_back(newf);

                // add new conflicts for new facet
                BuildConflictGraphByHorizonEdge(mCurFacets[mHorizonEdges[i]->GetId() / 3], mCurFacets[mHorizonEdges[i]->GetTwinEdge()->GetId() / 3], newf);
                // link new facet with horizon edge
                newf->LinkEdge(mHorizonEdges[i]);

                if (last != NULL)
                {
                    newf->LinkFace(last, p_r, mHorizonEdges[i]->GetOriginVertex());
                }

                last = newf;
                if (first == NULL)
                    first = newf;
            }

            // link the first and last facet
            if (first != NULL && last != NULL)
            {
                //KIRI_LOG_DEBUG("Link first 2 last");
                last->LinkFace(first, p_r, mHorizonEdges[0]->GetOriginVertex());
            }

            //KIRI_LOG_DEBUG("mCreatedFacets size={0}!", mCreatedFacets.size());
            // remove conflict facet
            if (mCreatedFacets.size() != 0)
            {
                for (size_t i = 0; i < mVisFacets.size(); i++)
                    RemoveConflictGraphByFace(mVisFacets[i]);
                mCreatedFacets.clear();

                // for (size_t tmp = mProcessedCount; tmp < mVertices.size(); tmp++)
                // {
                //     KIRI_LOG_DEBUG("After Remove mVConflict p_r idx={0}, num of conflict ={1}!", tmp, mVConflict[tmp].Size());
                // }
            }
            ++mProcessedCount;
            mVConflict[p_r->GetIdx()].ResetVisible();
        }
    }
    void KiriConvexHull3::AddVertex(const KiriVertex3Ptr &v3)
    {
        v3->SetIdx(mVertices.size());
        mVertices.emplace_back(v3);
        mVConflict.emplace_back(KiriVertexConflictLists());
    }

    void KiriConvexHull3::AddVertex(const Vector3F &v3)
    {
        auto vert = std::make_shared<KiriVertex3>(v3);
        vert->SetIdx(mVertices.size());
        mVertices.emplace_back(vert);
        mVConflict.emplace_back(KiriVertexConflictLists());
    }

    void KiriConvexHull3::AddFacet(const KiriFace3Ptr &f3)
    {
        f3->SetIdx(mCurFacets.size());
        f3->GenerateEdges();

        mCurFacets.emplace_back(f3);
        mFConflict.emplace_back(KiriFaceConflictLists());
    }

    void KiriConvexHull3::BuildConflictGraph(KiriFace3Ptr &f, KiriVertex3Ptr &v)
    {
        mFConflict[f->GetIdx()].Push(v);
        mVConflict[v->GetIdx()].Push(f);
    }

    void KiriConvexHull3::RemoveConflictGraphByFace(KiriFace3Ptr &f)
    {
        auto idx = f->GetIdx();

        for (size_t i = 0; i < mVConflict.size(); i++)
            mVConflict[i].Remove([=](KiriFace3Ptr x)
                                 { return x->GetIdx() == idx; });

        //KIRI_LOG_DEBUG("RemoveConflictGraphByFace face idx={0}!", f->GetIdx());

        // reset facet
        f->SetIdx(-1);
        for (size_t i = 0; i < 3; i++)
            f->GetEdgesByIdx(i)->CleanEdge();

        mFConflict[idx].RemoveAll();
        if (idx == mCurFacets.size() - 1)
        {
            mCurFacets.pop_back();
            mFConflict.pop_back();
            return;
        }
        if (idx >= mCurFacets.size() || idx < 0)
        {
            KIRI_LOG_ERROR("RemoveConflictGraphByFace: Face idx ERROR!!");
            return;
        }

        mFConflict[idx] = mFConflict.back();
        mFConflict.pop_back();

        mCurFacets[idx] = mCurFacets.back();
        mCurFacets.pop_back();

        // remake face info
        mCurFacets[idx]->SetIdx(idx);
        for (size_t i = 0; i < 3; i++)
            mCurFacets[idx]->GetEdgesByIdx(i)->SetId(idx * 3 + i);

        // for (size_t i = 0; i < mCurFacets.size(); i++)
        // {
        //     mCurFacets[i]->PrintFaceInfo();
        // }
    }

    /*** 
     * @description: 
     * @param {KiriFace3Ptr} hef1 incident facet of the horizon edge
     * @param {KiriFace3Ptr} hef2 incident facet of the horizon edge
     * @param {KiriFace3Ptr} f new facet
     * @return {*}
     */
    void KiriConvexHull3::BuildConflictGraphByHorizonEdge(KiriFace3Ptr &hef1, KiriFace3Ptr &hef2, KiriFace3Ptr &f)
    {
        // Vector<KiriVertex3Ptr> vf, vf1, vf2;
        // mFConflict[hef1->GetIdx()].BuildVertexList(vf1);
        // mFConflict[hef2->GetIdx()].BuildVertexList(vf2);

        // UInt i = 0, j = 0;

        // while (i < vf1.size() || j < vf2.size())
        // {
        //     if (i < vf1.size() && j < vf2.size())
        //     {
        //         auto v1 = vf1[i];
        //         auto v2 = vf2[j];

        //         if (v1->GetIdx() == v2->GetIdx())
        //         {
        //             vf.emplace_back(v1);
        //             ++i;
        //             ++j;
        //         }
        //         else if (v1->GetIdx() > v2->GetIdx())
        //         {
        //             vf.emplace_back(v1);
        //             ++i;
        //         }
        //         else
        //         {
        //             vf.emplace_back(v2);
        //             ++j;
        //         }
        //     }
        //     else if (i < vf1.size())
        //     {
        //         vf.emplace_back(vf1[i++]);
        //     }
        //     else
        //     {
        //         vf.emplace_back(vf2[j++]);
        //     }
        // }

        // // check conflicts
        // for (i = 0; i < vf.size(); i++)
        // {
        //     auto v = vf[i];
        //     if (f->CheckConflict(v->GetValue()))
        //         BuildConflictGraph(f, v);
        // }

        // for (size_t tmp = mProcessedCount; tmp < mVertices.size(); tmp++)
        // {
        //     KIRI_LOG_DEBUG("After Build mVConflict p_r idx={0}, num of conflict ={1}!", tmp, mVConflict[tmp].Size());
        // }

        for (size_t i = mProcessedCount; i < mVertices.size(); i++)
        {
            auto p_t = mVertices[i];

            if (f->CheckConflict(p_t->GetValue()))
                BuildConflictGraph(f, p_t);
            //KIRI_LOG_DEBUG(" mVConflict p_r idx={0}, num of conflict ={1}!", mProcessedCount, mVConflict[mProcessedCount].Size());
        }
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
            mVertices[i]->SetIdx(i);
        }

        // find 4 linear indepent vertex
        bool linear_dependent = true;
        KiriVertex3Ptr v0, v1, v2, v3;
        v0 = mVertices[0];
        v1 = mVertices[1];

        for (size_t i = 2; i < mVertices.size(); i++)
        {
            // not linearly dependent
            if (!(v0->LinearDependent(mVertices[i]->GetValue()) && v1->LinearDependent(mVertices[i]->GetValue())))
            {
                // KIRI_LOG_DEBUG("not linearly dependent, i={0}", i);
                v2 = mVertices[i];
                linear_dependent = false;
                if (i != 2)
                {
                    v2->SetIdx(2);
                    mVertices[2]->SetIdx(i);
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
        auto f0 = std::make_shared<KiriFace3>(v0, v1, v2);
        for (size_t i = 3; i < mVertices.size(); i++)
        {
            if ((f0->GetNormal().dot(f0->GetVertexByIdx(0)->GetValue()) != f0->GetNormal().dot(mVertices[i]->GetValue())))
            {
                v3 = mVertices[i];
                inside_face = false;
                if (i != 3)
                {
                    v3->SetIdx(3);
                    mVertices[3]->SetIdx(i);
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

        f0->OrientFace(v3);
        auto f1 = std::make_shared<KiriFace3>(v0, v2, v3, v1);
        auto f2 = std::make_shared<KiriFace3>(v0, v1, v3, v2);
        auto f3 = std::make_shared<KiriFace3>(v1, v2, v3, v0);

        AddFacet(f0);
        AddFacet(f1);
        AddFacet(f2);
        AddFacet(f3);
        mProcessedCount = 4;

        // connect facets
        f0->LinkFace(f1, v0, v2);
        f0->LinkFace(f2, v0, v1);
        f0->LinkFace(f3, v1, v2);
        f1->LinkFace(f2, v0, v3);
        f1->LinkFace(f3, v2, v3);
        f2->LinkFace(f3, v3, v1);

        // build conflict graph
        for (size_t i = mProcessedCount; i < mVertices.size(); i++)
        {
            auto p_t = mVertices[i];

            if (f0->CheckConflict(p_t->GetValue()))
                BuildConflictGraph(f0, p_t);

            if (f1->CheckConflict(p_t->GetValue()))
                BuildConflictGraph(f1, p_t);

            if (f2->CheckConflict(p_t->GetValue()))
                BuildConflictGraph(f2, p_t);

            if (f3->CheckConflict(p_t->GetValue()))
                BuildConflictGraph(f3, p_t);

            //KIRI_LOG_DEBUG(" mVConflict p_r idx={0}, num of conflict ={1}!", mProcessedCount, mVConflict[mProcessedCount].Size());
        }
    }
}