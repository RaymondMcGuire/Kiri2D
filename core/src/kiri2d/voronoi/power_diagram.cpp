/***
 * @Author: Xu.WANG
 * @Date: 2021-05-20 21:44:20
 * @LastEditTime: 2021-10-06 11:24:47
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\power_diagram.cpp
 */

#include <kiri2d/voronoi/power_diagram.h>
#include <kiri2d/geo/geo_plane3.h>
#include <polyclipper2d/PolygonClipping.h>
#include <random>
#include <bop12/booleanop.h>

namespace KIRI
{

    void KiriPowerDiagram::ResetVoroSitesWeight()
    {
        for (size_t i = 0; i < mVoroSites.size(); i++)
            mVoroSites[i]->ResetWeight();
    }

    void KiriPowerDiagram::RemoveVoroSitesByIndex(UInt idx)
    {
        if (!mVoroSites.empty() && idx < mVoroSites.size())
        {
            mVoroSites[idx] = mVoroSites.back();
            mVoroSites.pop_back();
        }
    }

    void KiriPowerDiagram::removeVoroSitesByIndexArray(Vector<UInt> indexs)
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
                    if (mBoundaryPolygon2->contains(polyVertices[j]))
                    {
                        auto minDis = mBoundaryPolygon2->computeMinDisInPoly(polyVertices[j]);
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

    Vector<Vector4F> KiriPowerDiagram::ComputeDelaunayTriangulation()
    {
        Vector<Vector4F> dela_tri;
        if (!mVoroSites.empty())
        {
            for (size_t i = 0; i < mVoroSites.size(); i++)
            {
                auto posi = mVoroSites[i]->GetValue();
                auto neighbors = mVoroSites[i]->GetNeighborSites();
                for (size_t j = 0; j < neighbors.size(); j++)
                {
                    auto posj = neighbors[j]->GetValue();
                    dela_tri.emplace_back(Vector4F(posi.x, posi.y, posj.x, posj.y));
                }
            }
        }
        return dela_tri;
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
        mBoundaryPolygon2->updateBBox();
        mBoundaryPolygon2->ComputeVoroSitesList();
        auto bbox = mBoundaryPolygon2->bbox();

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
        std::random_device seed;
        std::default_random_engine engine(seed());
        std::uniform_int_distribution<> dist(0, num - 1);

        for (size_t i = 0; i < num; i++)
        {
            auto rndIdx = dist(engine);
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
            // prevEdge = prevEdge->GetTwinEdge()->GetPrevEdge();
            if (prevEdge->GetTwinEdgeId() == -1)
                break;

            prevEdge = edges[edges[prevEdge->GetTwinEdgeId()]->GetPrevEdgeId()];

            KiriVoroSitePtr neighborSite = std::dynamic_pointer_cast<KiriVoroSite>(prevEdge->GetOriginVertex());
            // add neighbor sites
            if (!neighborSite->IsBoundarySite())
                neighborSitesList.emplace_back(neighborSite);

            auto facet = mConvexHull->GetFacets()[prevEdge->id() / 3];
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

    void KiriPowerDiagram::reset()
    {
        mConvexHull->reset();
        mConvexClip->reset();
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
                if (!mBoundaryPolygon2->contains(Vector2F(pos.x, pos.y)))
                    mVoroSites[i]->ResetValue(mBoundaryPolygon2->rndInnerPoint());
                else
                {
                    std::random_device seed;
                    std::default_random_engine engine(seed());
                    std::uniform_real_distribution<float> dist(0.f, 0.1f);
                    mVoroSites[i]->ResetValue(Vector2F(pos.x, pos.y) + Vector2F(dist(engine), dist(engine)));

                    // KIRI_LOG_INFO("ReGenVoroSites::pos={0},{1},{2}", pos.x, pos.y, pos.z);
                }
            }
        }
        else
            KIRI_LOG_ERROR("ReGenVoroSites::No voro site!!");

        ComputeDiagram();
    }

    bool KiriPowerDiagram::move2Centroid()
    {
        bool outside = false;
        // move sites to centroid
        for (size_t j = 0; j < mVoroSites.size(); j++)
        {
            // auto is_frozen = mVoroSites[j]->GetIsFrozen();
            // if (is_frozen)
            //     continue;

            auto poly = mVoroSites[j]->GetCellPolygon();
            if (poly != NULL)
            {
                auto cen = poly->GetPolygonCentroid();
                if (mBoundaryPolygon2->contains(cen))
                    mVoroSites[j]->ResetValue(cen);
                else
                {
                    outside = true;
                    // KIRI_LOG_DEBUG("centroid is placed outside of polygon boundaries!! = {0},{1}", cen.x, cen.y);
                    //  auto rndPos = mBoundaryPolygon2->rndInnerPoint();
                    //  mVoroSites[j]->ResetValue(rndPos);
                    mVoroSites[j]->SetCellPolygon(NULL);
                    mVoroSites[j]->Disable();
                }
            }

            // KIRI_LOG_DEBUG("voro site idx={0}, value=({1},{2},{3})", mVoroSites[j]->GetIdx(), mVoroSites[j]->GetValue().x, mVoroSites[j]->GetValue().y, mVoroSites[j]->GetValue().z);
        }

        return outside;
    }

    bool KiriPowerDiagram::Move2CentroidDisableSite()
    {
        Vector<UInt> remove_voro_idxs;
        bool outside = false;
        // move sites to centroid
        for (size_t j = 0; j < mVoroSites.size(); j++)
        {
            // auto is_frozen = mVoroSites[j]->GetIsFrozen();
            // if (is_frozen)
            //     continue;

            auto poly = mVoroSites[j]->GetCellPolygon();
            if (poly != NULL)
            {
                auto cen = poly->GetPolygonCentroid();
                if (mBoundaryPolygon2->contains(cen))
                    mVoroSites[j]->ResetValue(cen);
                else
                {
                    outside = true;
                    // KIRI_LOG_DEBUG("centroid is placed outside of polygon boundaries!! = {0},{1}", cen.x, cen.y);
                    mVoroSites[j]->SetCellPolygon(NULL);
                    mVoroSites[j]->Disable();
                    remove_voro_idxs.emplace_back(mVoroSites[j]->GetIdx());
                }
            }

            // KIRI_LOG_DEBUG("voro site idx={0}, value=({1},{2},{3})", mVoroSites[j]->GetIdx(), mVoroSites[j]->GetValue().x, mVoroSites[j]->GetValue().y, mVoroSites[j]->GetValue().z);
        }

        if (!remove_voro_idxs.empty())
            removeVoroSitesByIndexArray(remove_voro_idxs);

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

            if (mBoundaryPolygon2->contains(newPos))
                mVoroSites[j]->ResetValue(newPos);
            else
            {
                outside = true;
                mVoroSites[j]->ResetValue(mBoundaryPolygon2->rndInnerPoint());
            }
        }

        return outside;
    }

    void KiriPowerDiagram::LloydRelaxation()
    {
        // TODO need optimize
        ComputeDiagram();

        for (size_t i = 0; i < mRelaxIterNumber; i++)
        {
            move2Centroid();

            ComputeDiagram();
        }
    }

    void KiriPowerDiagram::LloydIterate()
    {
        move2Centroid();

        ComputeDiagram();
    }

    bool KiriPowerDiagram::ComputeDiagram()
    {
        if (!mVoroSites.empty())
        {
            // PermutateVoroSites();

            reset();

            // remove disabled voro sites
            mVoroSites.erase(
                std::remove_if(mVoroSites.begin(), mVoroSites.end(),
                               [](const KiriVoroSitePtr &site)
                               { return !site->IsEnable(); }),
                mVoroSites.end());

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
    }

    KiriVoroCellPolygon2Ptr KiriPowerDiagram::VoroCellPolygonClip(const KiriVoroCellPolygon2Ptr &vcp1, const KiriVoroCellPolygon2Ptr &vcp2)
    {
        if (!vcp1->bbox().overlaps(vcp2->bbox()))
        {
            // vcp1->Print();
            // vcp2->Print();
            // KIRI_LOG_ERROR("VoroCellPolygonClip: vcp1 low={0},{1}; high={2},{3};vcp2 low={4},{5}; high={6},{7}",
            //                vcp1->bbox().LowestPoint.x,
            //                vcp1->bbox().LowestPoint.y,
            //                vcp1->bbox().HighestPoint.x,
            //                vcp1->bbox().HighestPoint.y,
            //                vcp2->bbox().LowestPoint.x,
            //                vcp2->bbox().LowestPoint.y,
            //                vcp2->bbox().HighestPoint.x,
            //                vcp2->bbox().HighestPoint.y);
            // KIRI_LOG_ERROR("VoroCellPolygonClip: Bounding box no overlap!!");
            return NULL;
        }

        if (vcp1->bbox().contains(vcp2->bbox()))
        {
            // KIRI_LOG_DEBUG("contains");
            return vcp2;
        }

        // vcp2->PrintPolyVertices();

        vcp1->ComputeVoroSitesList();
        vcp2->ComputeVoroSitesList();

        // vcp2->PrintPolyVertices();
        // vcp2->PrintVoroSitesList();

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

        // KIRI_LOG_DEBUG("Voro cell intersection size={0}", intersection->Size());
        // intersection->PrintVertexList();
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

            clipedPolygon->updateBBox();
            clipedPolygon->ComputeVoroSitesList();
            // clipedPolygon->Print();
            return clipedPolygon;
        }

        // no intersection between the two polygons, so check if one is inside the other
        if (vcp1->bbox().contains(vcp2->GetPolygonVertices()[0]))
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
                    // auto edge = facet->GetEdgesByIdx(e);
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

                        // edge->PrintEdgeInfo();

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
                            if (dx > std::numeric_limits<float>::epsilon() || dy > std::numeric_limits<float>::epsilon())
                            {
                                // KIRI_LOG_DEBUG("dual={0},{1}", dual.x, dual.y);
                                cellPoly->AddPolygonVertex2(dual);
                                lastX = dual.x;
                                lastY = dual.y;
                            }
                        }

                        if (!site->IsBoundarySite())
                        {
                            cellPoly->updateBBox();
                            cellPoly->ComputeVoroSitesList();

                            if (cellPoly->GetLength() > 2)
                            {
                                if (mBoundaryPolygon2->bbox().overlaps(cellPoly->bbox()))
                                {
                                    if (mBoundaryPolygon2->bbox().contains(cellPoly->bbox()))
                                    {
                                        site->SetCellPolygon(cellPoly);
                                    }
                                    else
                                    {
                                        mBoundaryPolygon2->ComputeVoroSitesList();
                                        cellPoly->ComputeVoroSitesList();

                                        auto A = mBoundaryPolygon2->GetPolygonVertices();
                                        auto B = cellPoly->GetPolygonVertices();

                                        std::vector<PolyClip::Point2d> polyA;
                                        std::vector<PolyClip::Point2d> polyB;

                                        for (size_t ai = 0; ai < A.size(); ai++)
                                            polyA.push_back(PolyClip::Point2d(A[ai].x, A[ai].y));

                                        for (size_t bi = 0; bi < B.size(); bi++)
                                            polyB.push_back(PolyClip::Point2d(B[bi].x, B[bi].y));

                                        // cbop::Polygon polyA, polyB;

                                        // cbop::Contour vert1, vert2;
                                        // for (size_t ai = 0; ai < A.size(); ai++)
                                        //     vert1.add(cbop::Point_2(A[ai].x, A[ai].y));
                                        // polyA.push_back(vert1);

                                        // for (size_t bi = 0; bi < B.size(); bi++)
                                        //     vert2.add(cbop::Point_2(B[bi].x, B[bi].y));
                                        // polyB.push_back(vert2);

                                        // cbop::BooleanOpType op = cbop::INTERSECTION;
                                        // cbop::Polygon result;
                                        // auto compute_result = cbop::compute(polyA, polyB, result, op);

                                        PolyClip::Polygon polygon1(polyA);
                                        PolyClip::Polygon polygon2(polyB);
                                        auto bintersection = PolyClip::PloygonOpration::DetectIntersection(polygon1, polygon2);
                                        std::vector<std::vector<PolyClip::Point2d>> possible_result;

                                        // if (!result.getContours().empty() && compute_result == true)

                                        if (bintersection && PolyClip::PloygonOpration::Mark(polygon1, polygon2, possible_result, PolyClip::MarkIntersection))
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

                                            // auto p = result.getContours()[0].getPoints();
                                            // for (int pp = 0; pp < p.size(); ++pp)
                                            // {
                                            //     clipedPolygon->AddPolygonVertex2(Vector2F(p[pp].x(), p[pp].y()));
                                            // }

                                            clipedPolygon->PopPolygonVertex2();
                                            clipedPolygon->ReversePolygonVertex2();

                                            clipedPolygon->updateBBox();
                                            clipedPolygon->ComputeVoroSitesList();
                                            // clipedPolygon->Print();

                                            site->SetCellPolygon(clipedPolygon);
                                        }
                                        else
                                        {
                                            if (mBoundaryPolygon2->bbox().contains(cellPoly->GetPolygonVertices()[0]))
                                            {
                                                site->SetCellPolygon(cellPoly);
                                            }
                                            else
                                            {
                                                site->SetCellPolygon(NULL);
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