/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-20 21:44:20
 * @LastEditTime: 2021-09-13 10:35:28
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\power_diagram.cpp
 */

#include <kiri2d/voronoi/power_diagram.h>
#include <kiri2d/geo/geo_plane3.h>
#include <kiri2d/poly/PolygonClipping.h>
#include <random>

namespace KIRI
{
    void KiriPowerDiagram::RemoveVoroSitesByIndex(UInt idx)
    {
        if (!mVoroSites.empty() && idx < mVoroSites.size())
        {
            mVoroSites[idx] = mVoroSites.back();
            mVoroSites.pop_back();
        }
    }

    void KiriPowerDiagram::RemoveVoroSitesByIndexArray(Vector<UInt> indexs)
    {

        auto removeSites = std::remove_if(mVoroSites.begin(),
                                          mVoroSites.end(),
                                          [=](const KiriVoroSitePtr &site)
                                          {
                                              if (std::find(indexs.begin(), indexs.end(), site->GetIdx()) != indexs.end())
                                                  return true;
                                              else
                                                  return false;
                                          });

        mVoroSites.erase(removeSites, mVoroSites.end());
    }

    Vector3F KiriPowerDiagram::ComputeMaxInscribedCircle()
    {
        auto maxCirVec = Vector2F(0.f);
        auto maxCirRad = Tiny<float>();
        if (!mVoroSites.empty())
        {
            for (size_t i = 0; i < mVoroSites.size(); i++)
            {
                auto polyVertices = mVoroSites[i]->GetCellPolygon()->GetPolygonVertices();
                for (size_t j = 0; j < polyVertices.size(); j++)
                {
                    if (mBoundaryPolygon2->Contains(polyVertices[j]))
                    {
                        auto minDis = mBoundaryPolygon2->ComputeMinDisInPoly(polyVertices[j]);
                        if (minDis > maxCirRad)
                        {
                            maxCirRad = minDis;
                            maxCirVec = polyVertices[j];
                        }
                    }
                }
            }
        }
        return Vector3F(maxCirVec.x, maxCirVec.y, maxCirRad);
    }

    float KiriPowerDiagram::ComputeMinPorosity()
    {
        auto maxCir = ComputeMaxInscribedCircle();
        auto maxCirArea = maxCir.z * maxCir.z * KIRI_PI<float>();
        auto boundaryArea = mBoundaryPolygon2->GetPolygonArea();
        return (boundaryArea - maxCirArea) / boundaryArea;
    }

    void KiriPowerDiagram::SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary)
    {
        mBoundaryPolygon2 = boundary;
        mBoundaryPolygon2->UpdateBBox();
        mBoundaryPolygon2->ComputeVoroSitesList();
        auto bbox = mBoundaryPolygon2->GetBBox();

        if (bbox.width() < 0.f || bbox.height() < 0.f)
            KIRI_LOG_DEBUG("bbox low={0},{1}, high={2},{3}, width={4}, height={5}", bbox.LowestPoint.x, bbox.LowestPoint.y, bbox.HighestPoint.x, bbox.HighestPoint.y, bbox.width(), bbox.height());

        auto site1 = std::make_shared<KiriVoroSite>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y - bbox.height());
        auto site2 = std::make_shared<KiriVoroSite>(bbox.LowestPoint.x + 2.f * bbox.width(), bbox.LowestPoint.y - bbox.height());
        auto site3 = std::make_shared<KiriVoroSite>(bbox.LowestPoint.x + 2.f * bbox.width(), bbox.LowestPoint.y + 2.f * bbox.height());
        auto site4 = std::make_shared<KiriVoroSite>(bbox.LowestPoint.x - bbox.width(), bbox.LowestPoint.y + 2.f * bbox.height());

        site1->SetAsBoundarySite();
        site2->SetAsBoundarySite();
        site3->SetAsBoundarySite();
        site4->SetAsBoundarySite();

        mBoundaryVoroSites.clear();
        mBoundaryVoroSites.emplace_back(site1);
        mBoundaryVoroSites.emplace_back(site2);
        mBoundaryVoroSites.emplace_back(site3);
        mBoundaryVoroSites.emplace_back(site4);
    }

    void KiriPowerDiagram::PermutateVoroSites()
    {
        if (mVoroSites.empty())
            return;

        auto num = mVoroSites.size();
        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_int_distribution<> dist(0, num - 1);

        for (size_t i = 0; i < num; i++)
        {
            auto rndIdx = dist(rndEngine);
            auto tmp = mVoroSites[rndIdx];
            mVoroSites[rndIdx] = mVoroSites[i];
            mVoroSites[i] = tmp;
        }
    }

    void KiriPowerDiagram::ComputeFacetsAroundVertex(const KiriEdge3Ptr &edge)
    {
        if (!mFacetsAroundVertex.empty())
            mFacetsAroundVertex.clear();

        auto prevEdge = edge;
        KiriVoroSitePtr site = std::dynamic_pointer_cast<KiriVoroSite>(edge->GetDestVertex());

        Vector<KiriVoroSitePtr> neighborSitesList;
        auto edges = mConvexHull->GetEdges();
        do
        {
            //prevEdge = prevEdge->GetTwinEdge()->GetPrevEdge();
            if (prevEdge->GetTwinEdgeId() == -1)
                break;

            prevEdge = edges[edges[prevEdge->GetTwinEdgeId()]->GetPrevEdgeId()];

            KiriVoroSitePtr neighborSite = std::dynamic_pointer_cast<KiriVoroSite>(prevEdge->GetOriginVertex());
            // add neighbor sites
            if (!neighborSite->IsBoundarySite())
                neighborSitesList.emplace_back(neighborSite);

            auto facet = mConvexHull->GetFacets()[prevEdge->GetId() / 3];
            if (facet->IsVisibleFromBelow())
                mFacetsAroundVertex.emplace_back(facet);

        } while (prevEdge != edge);
        site->SetNeighborSites(neighborSitesList);

        // KIRI_LOG_DEBUG("-----------------PRINT SITE-------------");
        // site->Print();
        // KIRI_LOG_DEBUG("-----------------PRINT NEIGHBORS-------------");
        // for (size_t i = 0; i < neighborSitesList.size(); i++)
        // {
        //     neighborSitesList[i]->Print();
        // }
    }

    void KiriPowerDiagram::Reset()
    {
        mConvexHull->Reset();
        mConvexClip->Reset();
        mVisitedVoroSites.clear();
        mFacetsAroundVertex.clear();
    }

    void KiriPowerDiagram::ReGenVoroSites()
    {
        KIRI_LOG_INFO("ReGenVoroSites::Start, size={0}", mVoroSites.size());
        if (mBoundaryPolygon2 == NULL)
        {
            KIRI_LOG_ERROR("ReGenVoroSites::Please set boundary polygon!!");
            return;
        }

        if (!mVoroSites.empty())
        {
            for (size_t i = 0; i < mVoroSites.size(); i++)
            {
                auto pos = mVoroSites[i]->GetValue();
                if (!mBoundaryPolygon2->Contains(Vector2F(pos.x, pos.y)))
                    mVoroSites[i]->ResetValue(mBoundaryPolygon2->GetRndInnerPoint());
                else
                {
                    std::random_device seedGen;
                    std::default_random_engine rndEngine(seedGen());
                    std::uniform_real_distribution<float> dist(0.f, 0.1f);
                    mVoroSites[i]->ResetValue(Vector2F(pos.x, pos.y) + Vector2F(dist(rndEngine), dist(rndEngine)));

                    // KIRI_LOG_INFO("ReGenVoroSites::pos={0},{1},{2}", pos.x, pos.y, pos.z);
                }
            }
        }
        else
            KIRI_LOG_ERROR("ReGenVoroSites::No voro site!!");

        ComputeDiagram();
    }

    bool KiriPowerDiagram::Move2Centroid()
    {
        bool outside = false;
        // move sites to centroid
        for (size_t j = 0; j < mVoroSites.size(); j++)
        {
            auto poly = mVoroSites[j]->GetCellPolygon();
            if (poly != NULL)
            {
                auto cen = poly->GetPolygonCentroid();
                if (mBoundaryPolygon2->Contains(cen))
                    mVoroSites[j]->ResetValue(cen);
                else
                {
                    outside = true;
                    KIRI_LOG_DEBUG("centroid is placed outside of polygon boundaries!! = {0},{1}", cen.x, cen.y);
                    auto rndPos = mBoundaryPolygon2->GetRndInnerPoint();
                    mVoroSites[j]->ResetValue(rndPos);
                    // mVoroSites[j]->SetCellPolygon(NULL);
                }
            }

            //KIRI_LOG_DEBUG("voro site idx={0}, value=({1},{2},{3})", mVoroSites[j]->GetIdx(), mVoroSites[j]->GetValue().x, mVoroSites[j]->GetValue().y, mVoroSites[j]->GetValue().z);
        }

        return outside;
    }

    bool KiriPowerDiagram::MoveVoroSites(Vector<Vector2F> movement)
    {
        bool outside = false;
        // move sites to centroid
        for (size_t j = 0; j < mVoroSites.size(); j++)
        {
            auto curPos = Vector2F(mVoroSites[j]->GetValue().x, mVoroSites[j]->GetValue().y);
            auto newPos = curPos + movement[j];

            if (mBoundaryPolygon2->Contains(newPos))
                mVoroSites[j]->ResetValue(newPos);
            else
            {
                outside = true;
                mVoroSites[j]->ResetValue(mBoundaryPolygon2->GetRndInnerPoint());
            }
        }

        return outside;
    }

    void KiriPowerDiagram::LloydRelaxation()
    {
        //TODO need optimize
        ComputeDiagram();

        for (size_t i = 0; i < mRelaxIterNumber; i++)
        {
            Move2Centroid();

            ComputeDiagram();
        }
    }

    void KiriPowerDiagram::LloydIterate()
    {
        Move2Centroid();

        ComputeDiagram();
    }

    bool KiriPowerDiagram::ComputeDiagram()
    {
        if (!mVoroSites.empty())
        {
            //PermutateVoroSites();

            Reset();

            for (size_t i = 0; i < mVoroSites.size(); i++)
            {
                mConvexHull->AddVertex(std::dynamic_pointer_cast<KiriVertex3>(mVoroSites[i]));
                mVisitedVoroSites.emplace_back(false);
            }

            for (size_t i = 0; i < mBoundaryVoroSites.size(); i++)
            {
                mConvexHull->AddVertex(std::dynamic_pointer_cast<KiriVertex3>(mBoundaryVoroSites[i]));
                mVisitedVoroSites.emplace_back(false);
            }

            if (!mConvexHull->ComputeConvexHull())
                return false;

            ComputeVoroCells();

            return true;
        }

        return false;
        //KIRI_LOG_INFO("Compute Power Diagram Finished!!");
    }

    KiriVoroCellPolygon2Ptr KiriPowerDiagram::VoroCellPolygonClip(const KiriVoroCellPolygon2Ptr &vcp1, const KiriVoroCellPolygon2Ptr &vcp2)
    {
        if (!vcp1->GetBBox().overlaps(vcp2->GetBBox()))
        {
            // vcp1->Print();
            // vcp2->Print();
            // KIRI_LOG_ERROR("VoroCellPolygonClip: vcp1 low={0},{1}; high={2},{3};vcp2 low={4},{5}; high={6},{7}",
            //                vcp1->GetBBox().LowestPoint.x,
            //                vcp1->GetBBox().LowestPoint.y,
            //                vcp1->GetBBox().HighestPoint.x,
            //                vcp1->GetBBox().HighestPoint.y,
            //                vcp2->GetBBox().LowestPoint.x,
            //                vcp2->GetBBox().LowestPoint.y,
            //                vcp2->GetBBox().HighestPoint.x,
            //                vcp2->GetBBox().HighestPoint.y);
            // KIRI_LOG_ERROR("VoroCellPolygonClip: Bounding box no overlap!!");
            return NULL;
        }

        if (vcp1->GetBBox().contains(vcp2->GetBBox()))
        {
            //KIRI_LOG_DEBUG("Contains");
            return vcp2;
        }

        //vcp2->PrintPolyVertices();

        vcp1->ComputeVoroSitesList();
        vcp2->ComputeVoroSitesList();

        //vcp2->PrintPolyVertices();
        //vcp2->PrintVoroSitesList();

        // auto rmpo = vcp2->GetPolygonVertices();
        // KIRI_LOG_DEBUG("---------------");
        // for (size_t i = 0; i < rmpo.size(); i++)
        // {
        //     KIRI_LOG_DEBUG("({0},{1})", rmpo[i].x, rmpo[i].y);
        // }

        bool clipStatus = mConvexClip->ComputeConvexPolygonIntersection(vcp1->GetVoroSitesList(), vcp2->GetVoroSitesList());

        if (!clipStatus)
            return NULL;

        auto intersection = mConvexClip->GetIntersectionList();

        //KIRI_LOG_DEBUG("Voro cell intersection size={0}", intersection->Size());
        //intersection->PrintVertexList();
        if (intersection != NULL && intersection->Size() > 0)
        {
            auto node = intersection->GetHead();
            double firstX = node->value.x;
            double firstY = node->value.y;

            auto clipedPolygon = std::make_shared<KiriVoroCellPolygon2>();
            clipedPolygon->AddPolygonVertex2(Vector2F(node->value.x, node->value.y));
            double lastX = node->value.x;
            double lastY = node->value.y;
            for (size_t i = 1; i < intersection->Size(); i++)
            {
                node = node->next;

                if (lastX != node->value.x || lastY != node->value.y)
                { // do not add point
                    // if its the
                    // same as
                    // before
                    if (i != (intersection->Size() - 1) || (node->value.x != firstX) || node->value.y != firstY)
                    { // do not add if it is the
                        // end point and the same as
                        // the first point
                        clipedPolygon->AddPolygonVertex2(Vector2F(node->value.x, node->value.y));
                    }
                }
            }

            clipedPolygon->UpdateBBox();
            clipedPolygon->ComputeVoroSitesList();
            // clipedPolygon->Print();
            return clipedPolygon;
        }

        //no intersection between the two polygons, so check if one is inside the other
        if (vcp1->GetBBox().contains(vcp2->GetPolygonVertices()[0]))
            return vcp2;

        // no intersection between the polygons at all
        return NULL;
    }

    void KiriPowerDiagram::ComputeVoroCells()
    {
        auto edges = mConvexHull->GetEdges();
        for (size_t i = 0; i < mConvexHull->GetNumOfFacets(); i++)
        {
            auto facet = mConvexHull->GetFacets()[i];
            if (facet->IsVisibleFromBelow())
            {
                for (size_t e = 0; e < 3; e++)
                {
                    //auto edge = facet->GetEdgesByIdx(e);
                    auto edge = edges[facet->GetEdgeIdByIdx(e)];

                    auto dst = edge->GetDestVertex();
                    auto site = std::dynamic_pointer_cast<KiriVoroSite>(dst);

                    if (!mVisitedVoroSites[dst->GetIdx()])
                    {
                        mVisitedVoroSites[dst->GetIdx()] = true;
                        if (site->IsBoundarySite())
                            continue;

                        ComputeFacetsAroundVertex(edge);

                        auto cellPoly = std::make_shared<KiriVoroCellPolygon2>();
                        auto lastX = Huge<float>(), lastY = Huge<float>();
                        auto dx = 1.f, dy = 1.f;

                        //edge->PrintEdgeInfo();

                        for (size_t j = 0; j < mFacetsAroundVertex.size(); j++)
                        {
                            auto f = mFacetsAroundVertex[j];
                            auto p = std::make_shared<KiriGeoPlane3>(
                                f->GetVertexByIdx(0)->GetValue(),
                                f->GetVertexByIdx(1)->GetValue(),
                                f->GetVertexByIdx(2)->GetValue());
                            auto dual = p->GetDualPointMappedToPlane();

                            if (lastX != Huge<float>())
                            {
                                dx = lastX - dual.x;
                                dy = lastY - dual.y;
                                if (dx < 0.f)
                                    dx = -dx;

                                if (dy < 0.f)
                                    dy = -dy;
                            }
                            if (dx > MEpsilon<float>() || dy > MEpsilon<float>())
                            {
                                //KIRI_LOG_DEBUG("dual={0},{1}", dual.x, dual.y);
                                cellPoly->AddPolygonVertex2(dual);
                                lastX = dual.x;
                                lastY = dual.y;
                            }
                        }

                        if (!site->IsBoundarySite())
                        {
                            cellPoly->UpdateBBox();
                            cellPoly->ComputeVoroSitesList();

                            if (cellPoly->GetLength() > 2)
                            {

                                if (mBoundaryPolygon2->GetBBox().overlaps(cellPoly->GetBBox()))
                                {
                                    if (mBoundaryPolygon2->GetBBox().contains(cellPoly->GetBBox()))
                                    {
                                        site->SetCellPolygon(cellPoly);
                                    }
                                    else
                                    {
                                        mBoundaryPolygon2->ComputeVoroSitesList();
                                        cellPoly->ComputeVoroSitesList();

                                        std::vector<PolyClip::Point2d> polyA;
                                        std::vector<PolyClip::Point2d> polyB;

                                        auto A = mBoundaryPolygon2->GetPolygonVertices();
                                        auto B = cellPoly->GetPolygonVertices();
                                        for (size_t ai = 0; ai < A.size(); ai++)
                                            polyA.push_back(PolyClip::Point2d(A[ai].x, A[ai].y));

                                        for (size_t bi = 0; bi < B.size(); bi++)
                                            polyB.push_back(PolyClip::Point2d(B[bi].x, B[bi].y));

                                        PolyClip::Polygon polygon1(polyA);
                                        PolyClip::Polygon polygon2(polyB);
                                        PolyClip::PloygonOpration::DetectIntersection(polygon1, polygon2);
                                        std::vector<std::vector<PolyClip::Point2d>> possible_result;
                                        if (PolyClip::PloygonOpration::Mark(polygon1, polygon2, possible_result, PolyClip::MarkIntersection))
                                        {
                                            auto clipedPolygon = std::make_shared<KiriVoroCellPolygon2>();
                                            std::vector<std::vector<PolyClip::Point2d>> results = PolyClip::PloygonOpration::ExtractIntersectionResults(polygon1);
                                            for (int pp = 0; pp < results.size(); ++pp)
                                            {

                                                for (size_t ppp = 0; ppp < results[pp].size(); ppp++)
                                                {
                                                    auto polyn = results[pp][ppp];
                                                    clipedPolygon->AddPolygonVertex2(Vector2F(polyn.x_, polyn.y_));
                                                }
                                            }
                                            clipedPolygon->PopPolygonVertex2();
                                            clipedPolygon->ReversePolygonVertex2();

                                            clipedPolygon->UpdateBBox();
                                            clipedPolygon->ComputeVoroSitesList();
                                            //clipedPolygon->Print();

                                            site->SetCellPolygon(clipedPolygon);
                                        }
                                        else
                                        {
                                            if (mBoundaryPolygon2->GetBBox().contains(cellPoly->GetPolygonVertices()[0]))
                                            {
                                                site->SetCellPolygon(cellPoly);
                                            }
                                        }
                                    }
                                }

                                // auto newPoly = VoroCellPolygonClip(mBoundaryPolygon2, cellPoly);
                                // if (newPoly != NULL)
                                //     site->SetCellPolygon(newPoly);
                            }
                        }
                    }
                }
            }
        }
    }

    void KiriPowerDiagram::PrintVoroSites()
    {
        KIRI_LOG_DEBUG("----------Voronoi Sites INFO----------");
        for (size_t i = 0; i < mVoroSites.size(); i++)
            mVoroSites[i]->Print();
        KIRI_LOG_DEBUG("------------------------------");
    }
}