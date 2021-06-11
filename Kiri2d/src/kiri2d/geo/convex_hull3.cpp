/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:33:21
 * @LastEditTime: 2021-06-11 01:40:38
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
            mFConflict[i]->PrintFaceConflictList();

        for (size_t i = 0; i < mVConflict.size(); i++)
            mVConflict[i]->PrintVertexConflictList();

        KIRI_LOG_DEBUG("---------------------------------------");
    }

    const bool KiriConvexHull3::IsHorizonEdge(const KiriEdge3Ptr &edge)
    {
        auto twinId = edge->GetTwinEdgeId();
        if (twinId != -1)
        {
            // twin edge face
            auto tef = mCurFacets[twinId / 3];
            auto ef = mCurFacets[edge->GetId() / 3];
            // KIRI_LOG_DEBUG("twin edge face id={0}, vis={1}, ef id={2} !ef={3}", twinId, tef->GetVisible(), edge->GetId(), !ef->GetVisible());
            if (tef->GetVisible() && !ef->GetVisible())
                return true;
        }
        return false;
    }

    void KiriConvexHull3::Reset()
    {
        mProcessedCount = 0;
        mVertices.clear();
        mEdges.clear();
        mHorizonEdges.clear();
        mCurFacets.clear();
        mVisFacets.clear();
        mCreatedFacets.clear();

        for (size_t i = 0; i < mFConflict.size(); i++)
            mFConflict[i]->RemoveAll();

        for (size_t i = 0; i < mVConflict.size(); i++)
            mVConflict[i]->RemoveAll();

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
                mHorizonEdges.emplace_back(edge);
                FindHorizonEdgeList(mEdges[edge->GetNextEdgeId()]);
                //FindHorizonEdgeList(edge->GetNextEdge());
            }
        }
        else if (edge->GetTwinEdgeId() != -1)
        {

            FindHorizonEdgeList(mEdges[mEdges[edge->GetTwinEdgeId()]->GetNextEdgeId()]);
        }
    }

    bool KiriConvexHull3::ComputeConvexHull()
    {
        BuildTetrahedron();

        while (mProcessedCount < mVertices.size())
        {
            // KIRI_LOG_DEBUG("mProcessedCount={0}", mProcessedCount);
            auto p_r = mVertices[mProcessedCount];

            // vertex in convex hull
            if (mVConflict[p_r->GetIdx()]->Size() == 0)
            {
                ++mProcessedCount;
                continue;
            }

            mVisFacets.clear();
            mCreatedFacets.clear();
            mHorizonEdges.clear();

            // outside
            mVConflict[p_r->GetIdx()]->BuildVisibleList(mVisFacets);

            // find horizon edges
            bool find_horizon_edges = false;
            for (size_t i = 0; i < mVisFacets.size(); i++)
            {
                if (find_horizon_edges)
                    break;

                auto vf = mVisFacets[i];
                for (size_t j = 0; j < 3; j++)
                {
                    // auto vfe = vf->GetEdgesByIdx(j);
                    // if (vfe->GetTwinEdge() != NULL)
                    // {
                    //     auto te = vfe->GetTwinEdge();
                    //     if (IsHorizonEdge(te))
                    //     {
                    //         FindHorizonEdgeList(vfe);
                    //         find_horizon_edges = true;
                    //         break;
                    //     }
                    // }
                    auto vfeId = vf->GetEdgeIdByIdx(j);
                    if (mEdges[vfeId]->GetTwinEdgeId() != -1)
                    {
                        auto te = mEdges[mEdges[vfeId]->GetTwinEdgeId()];
                        //KIRI_LOG_DEBUG("face id={0}, edge id={1}, twin edge id={2}", vf->GetIdx(), vfeId, mEdges[vfeId]->GetTwinEdgeId());
                        if (IsHorizonEdge(te))
                        {
                            FindHorizonEdgeList(mEdges[vfeId]);
                            find_horizon_edges = true;
                            break;
                        }
                    }
                }
            }

            //KIRI_LOG_DEBUG("mHorizonEdges size={0}", mHorizonEdges.size());
            // create new facets
            KiriFace3Ptr first = NULL, last = NULL;
            for (size_t i = 0; i < mHorizonEdges.size(); i++)
            {
                //auto newf = std::make_shared<KiriFace3>(p_r, mHorizonEdges[i]->GetOriginVertex(), mHorizonEdges[i]->GetDestVertex(), mHorizonEdges[i]->GetTwinEdge()->GetNextEdge()->GetDestVertex());
                auto newf =
                    std::make_shared<KiriFace3>(
                        p_r,
                        mHorizonEdges[i]->GetOriginVertex(),
                        mHorizonEdges[i]->GetDestVertex(),
                        mEdges[mEdges[mHorizonEdges[i]->GetTwinEdgeId()]->GetNextEdgeId()]->GetDestVertex());

                AddFacet(newf);
                mCreatedFacets.emplace_back(newf);

                // add new conflicts for new facet
                //BuildConflictGraphByHorizonEdge(mCurFacets[mHorizonEdges[i]->GetId() / 3], mCurFacets[mHorizonEdges[i]->GetTwinEdge()->GetId() / 3], newf);
                BuildConflictGraphByHorizonEdge(mCurFacets[mHorizonEdges[i]->GetId() / 3], mCurFacets[mHorizonEdges[i]->GetTwinEdgeId() / 3], newf);

                // link new facet with horizon edge
                //newf->LinkEdge(mHorizonEdges[i]);
                if (!LinkFaceEdge(newf, mHorizonEdges[i]))
                    return false;

                if (last != NULL)
                {
                    //newf->LinkFace(last, p_r, mHorizonEdges[i]->GetOriginVertex());
                    if (!LinkFace(newf, last, p_r, mHorizonEdges[i]->GetOriginVertex()))
                        return false;
                }

                last = newf;
                if (first == NULL)
                    first = newf;
            }

            // link the first and last facet
            if (first != NULL && last != NULL)
                if (!LinkFace(last, first, p_r, mHorizonEdges[0]->GetOriginVertex()))
                    return false;

            // remove conflict facet
            if (mCreatedFacets.size() != 0)
            {
                for (size_t i = 0; i < mVisFacets.size(); i++)
                    RemoveConflictGraphByFace(mVisFacets[i]);
                mCreatedFacets.clear();
            }
            ++mProcessedCount;
            mVConflict[p_r->GetIdx()]->ResetVisible();
        }

        return true;
    }
    void KiriConvexHull3::AddVertex(const KiriVertex3Ptr &v3)
    {
        v3->SetIdx(mVertices.size());
        mVertices.emplace_back(v3);
        mVConflict.emplace_back(std::make_shared<KiriVertexConflictLists>());
    }

    void KiriConvexHull3::AddVertex(const Vector3F &v3)
    {
        auto vert = std::make_shared<KiriVertex3>(v3);
        vert->SetIdx(mVertices.size());
        mVertices.emplace_back(vert);
        mVConflict.emplace_back(std::make_shared<KiriVertexConflictLists>());
    }

    void KiriConvexHull3::AddFacet(const KiriFace3Ptr &f3)
    {
        f3->SetIdx(mCurFacets.size());
        f3->GenerateEdges();

        // ------------------------------
        // generate edges for f3
        auto edgeA = std::make_shared<KiriEdge3>(f3->GetEdgeIdByIdx(0), f3->GetVertexByIdx(0), f3->GetVertexByIdx(1));
        auto edgeB = std::make_shared<KiriEdge3>(f3->GetEdgeIdByIdx(1), f3->GetVertexByIdx(1), f3->GetVertexByIdx(2));
        auto edgeC = std::make_shared<KiriEdge3>(f3->GetEdgeIdByIdx(2), f3->GetVertexByIdx(2), f3->GetVertexByIdx(0));

        edgeA->SetNextEdgeId(f3->GetEdgeIdByIdx(1));
        edgeB->SetNextEdgeId(f3->GetEdgeIdByIdx(2));
        edgeC->SetNextEdgeId(f3->GetEdgeIdByIdx(0));

        edgeA->SetPrevEdgeId(f3->GetEdgeIdByIdx(2));
        edgeB->SetPrevEdgeId(f3->GetEdgeIdByIdx(0));
        edgeC->SetPrevEdgeId(f3->GetEdgeIdByIdx(1));

        mEdges.emplace_back(edgeA);
        mEdges.emplace_back(edgeB);
        mEdges.emplace_back(edgeC);
        // KIRI_LOG_DEBUG("AddFacet: facet id={0}, edge id={1},{2},{3}", f3->GetIdx(), edgeA->GetId(), edgeB->GetId(), edgeC->GetId());
        // ------------------------------

        mCurFacets.emplace_back(f3);
        mFConflict.emplace_back(std::make_shared<KiriFaceConflictLists>());
    }

    UInt KiriConvexHull3::GetFaceEdgeId(const KiriFace3Ptr &f, const KiriVertex3Ptr &a, const KiriVertex3Ptr &b)
    {
        auto edgeId = -1;
        for (size_t i = 0; i < f->GetEdgeCount(); i++)
        {
            auto eId = f->GetEdgeIdByIdx(i);
            auto edge = GetEdgeById(eId);
            //KIRI_LOG_DEBUG("edge idx ={0}, edge id={1}", eId, edge->GetId());
            auto bSameEdge = edge->IsEqual(a, b);
            if (bSameEdge)
            {
                edgeId = eId;
                break;
            }
        }

        // if (edgeId == -1)
        // {
        //     KIRI_LOG_ERROR("Debug error Link face!!");
        //     for (size_t i = 0; i < f->GetEdgeCount(); i++)
        //     {
        //         auto eId = f->GetEdgeIdByIdx(i);
        //         auto edge = GetEdgeById(eId);
        //         edge->PrintEdgeInfo();
        //     }
        //     KIRI_LOG_ERROR("Edge Info");
        //     a->Print();
        //     b->Print();
        // }

        return edgeId;
    }

    void KiriConvexHull3::CleanTwinEdge(const KiriEdge3Ptr &e)
    {
        auto twinId = e->GetTwinEdgeId();
        if (twinId != -1)
        {
            if (twinId < mEdges.size() && mEdges[twinId]->GetTwinEdgeId() == e->GetId())
                mEdges[twinId]->SetTwinEdgeId(-1);

            mEdges[e->GetId()]->SetTwinEdgeId(-1);
            //KIRI_LOG_DEBUG("array twin id={0}, cur twin id={0}", mEdges[e->GetId()]->GetTwinEdgeId(), e->GetTwinEdgeId());
        }
    }

    void KiriConvexHull3::ResetFace(const KiriFace3Ptr &f, UInt idx)
    {
        // get original edge data
        auto edgeA = mEdges[f->GetEdgeIdByIdx(0)];
        auto edgeB = mEdges[f->GetEdgeIdByIdx(1)];
        auto edgeC = mEdges[f->GetEdgeIdByIdx(2)];

        // reset face idx info
        f->SetIdx(idx);
        f->GenerateEdges();

        // reset edge id
        edgeA->SetId(f->GetEdgeIdByIdx(0));
        edgeB->SetId(f->GetEdgeIdByIdx(1));
        edgeC->SetId(f->GetEdgeIdByIdx(2));

        // relink edge
        edgeA->SetNextEdgeId(f->GetEdgeIdByIdx(1));
        edgeB->SetNextEdgeId(f->GetEdgeIdByIdx(2));
        edgeC->SetNextEdgeId(f->GetEdgeIdByIdx(0));

        edgeA->SetPrevEdgeId(f->GetEdgeIdByIdx(2));
        edgeB->SetPrevEdgeId(f->GetEdgeIdByIdx(0));
        edgeC->SetPrevEdgeId(f->GetEdgeIdByIdx(1));

        // reset twin edge
        if (edgeA->GetTwinEdgeId() != -1)
            mEdges[edgeA->GetTwinEdgeId()]->SetTwinEdgeId(edgeA->GetId());

        if (edgeB->GetTwinEdgeId() != -1)
            mEdges[edgeB->GetTwinEdgeId()]->SetTwinEdgeId(edgeB->GetId());

        if (edgeC->GetTwinEdgeId() != -1)
            mEdges[edgeC->GetTwinEdgeId()]->SetTwinEdgeId(edgeC->GetId());

        // reset edge array
        mEdges[f->GetEdgeIdByIdx(0)] = edgeA;
        mEdges[f->GetEdgeIdByIdx(1)] = edgeB;
        mEdges[f->GetEdgeIdByIdx(2)] = edgeC;

        // reset facet array
        mCurFacets[idx] = f;
    }

    void KiriConvexHull3::LinkEdge(UInt eAId, UInt eBId)
    {
        mEdges[eAId]->SetTwinEdgeId(eBId);
        mEdges[eBId]->SetTwinEdgeId(eAId);
    }

    void KiriConvexHull3::LinkEdge(const KiriEdge3Ptr &eA, const KiriEdge3Ptr &eB)
    {
        LinkEdge(eA->GetId(), eB->GetId());
    }

    bool KiriConvexHull3::LinkFaceEdge(const KiriFace3Ptr &f, const KiriEdge3Ptr &e)
    {
        auto curEdgeId = GetFaceEdgeId(f, e->GetOriginVertex(), e->GetDestVertex());

        if (curEdgeId == -1)
        {
            KIRI_LOG_ERROR("LinkFaceEdge ERROR: Current edge is not exist, cannot connect edges!");
            return false;
        }

        if (curEdgeId == e->GetId())
        {
            KIRI_LOG_ERROR("LinkFaceEdge ERROR: face idx={0}, face edge idx={1}, link edge idx={2}", f->GetIdx(), curEdgeId, e->GetId());
            return false;
        }

        LinkEdge(e, mEdges[curEdgeId]);

        return true;
    }

    bool KiriConvexHull3::LinkFace(const KiriFace3Ptr &fA, const KiriFace3Ptr &fB, const KiriVertex3Ptr &a, const KiriVertex3Ptr &b)
    {
        auto twinEdgeId = GetFaceEdgeId(fB, a, b);

        if (twinEdgeId == -1)
        {
            // KIRI_LOG_ERROR("LinkFace ERROR: Twin edge is not exist, cannot connect edges!");
            return false;
        }

        auto curEdgeId = GetFaceEdgeId(fA, a, b);

        if (curEdgeId != -1)
        {
            //KIRI_LOG_DEBUG("Link face, twin edge id={0}, cur edge id={1}", twinEdgeId, curEdgeId);
            LinkEdge(twinEdgeId, curEdgeId);
            return true;
        }
        // else
        //     KIRI_LOG_ERROR("LinkFace ERROR: Current edge is not exist, cannot connect edges!");

        return false;
    }

    void KiriConvexHull3::BuildConflictGraph(KiriFace3Ptr &f, KiriVertex3Ptr &v)
    {
        mFConflict[f->GetIdx()]->Push(v);
        mVConflict[v->GetIdx()]->Push(f);
    }

    void KiriConvexHull3::RemoveConflictGraphByFace(KiriFace3Ptr &f)
    {
        // KIRI_LOG_DEBUG("Before Delete Face, face array size={0}, edge size={1}", mCurFacets.size(), mEdges.size());
        // KIRI_LOG_DEBUG("+++++++++++++++++++++++++++++++++++++++++");
        // for (size_t i = 0; i < mEdges.size(); i++)
        // {
        //     mEdges[i]->PrintEdgeInfo();
        // }

        auto idx = f->GetIdx();

        for (size_t i = 0; i < mVConflict.size(); i++)
            mVConflict[i]->Remove([=](KiriFace3Ptr x)
                                  { return x->GetIdx() == idx; });

        //KIRI_LOG_DEBUG("RemoveConflictGraphByFace face idx={0}!", f->GetIdx());

        // reset facet
        f->SetIdx(-1);
        // for (size_t i = 0; i < 3; i++)
        //     f->GetEdgesByIdx(i)->CleanEdge();
        for (size_t i = 0; i < 3; i++)
        {
            CleanTwinEdge(mEdges[f->GetEdgeIdByIdx(i)]);
        }
        // mEdges[f->GetEdgeIdByIdx(i)]->CleanEdge();

        mFConflict[idx]->RemoveAll();
        if (idx == mCurFacets.size() - 1)
        {
            mCurFacets.pop_back();
            // clean edges
            mEdges.pop_back();
            mEdges.pop_back();
            mEdges.pop_back();

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

        // move back element -> idx
        ResetFace(mCurFacets.back(), idx);
        mCurFacets.pop_back();
        mEdges.pop_back();
        mEdges.pop_back();
        mEdges.pop_back();

        // KIRI_LOG_DEBUG("After Delete Face, face array size={0}, edge size={1}", mCurFacets.size(), mEdges.size());
        // KIRI_LOG_DEBUG("****************************************************");
        // KIRI_LOG_DEBUG("Delete idx = {0}", idx);
        // for (size_t i = 0; i < mEdges.size(); i++)
        // {
        //     mEdges[i]->PrintEdgeInfo();
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
            mVertices[i]->SetIdx(i);

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
        // f0->LinkFace(f1, v0, v2);
        // f0->LinkFace(f2, v0, v1);
        // f0->LinkFace(f3, v1, v2);
        // f1->LinkFace(f2, v0, v3);
        // f1->LinkFace(f3, v2, v3);
        // f2->LinkFace(f3, v3, v1);
        LinkFace(f0, f1, v0, v2);
        LinkFace(f0, f2, v0, v1);
        LinkFace(f0, f3, v1, v2);
        LinkFace(f1, f2, v0, v3);
        LinkFace(f1, f3, v2, v3);
        LinkFace(f2, f3, v3, v1);

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