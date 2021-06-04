/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-20 21:44:20
 * @LastEditTime: 2021-06-04 15:38:30
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\power_diagram.cpp
 */

#include <kiri2d/voronoi/power_diagram.h>
#include <kiri2d/geo/geo_plane3.h>
#include <random>

namespace KIRI
{
    void KiriPowerDiagram::SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary)
    {
        mBoundaryPolygon2 = boundary;
        mBoundaryPolygon2->UpdateBBox();
        mBoundaryPolygon2->ComputeVoroSitesList();
        auto bbox = mBoundaryPolygon2->GetBBox();

        //KIRI_LOG_DEBUG("bbox low={0},{1}, high={2},{3}, width={4}, height={5}", bbox.LowestPoint.x, bbox.LowestPoint.y, bbox.HighestPoint.x, bbox.HighestPoint.y, bbox.width(), bbox.height());

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
        do
        {
            prevEdge = prevEdge->GetTwinEdge()->GetPrevEdge();
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
        KIRI_LOG_ERROR("ReGenVoroSites::Start, size={0}", mVoroSites.size());
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
                    mVoroSites[j]->ResetValue(mBoundaryPolygon2->GetRndInnerPoint());
                }
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

    void KiriPowerDiagram::ComputeDiagram()
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

            mConvexHull->ComputeConvexHull();

            ComputeVoroCells();
        }

        //KIRI_LOG_INFO("Compute Power Diagram Finished!!");
    }

    KiriVoroCellPolygon2Ptr KiriPowerDiagram::VoroCellPolygonClip(const KiriVoroCellPolygon2Ptr &vcp1, const KiriVoroCellPolygon2Ptr &vcp2)
    {
        if (!vcp1->GetBBox().overlaps(vcp2->GetBBox()))
        {
            KIRI_LOG_ERROR("VoroCellPolygonClip: Bounding box no overlap!!");
            return NULL;
        }

        if (vcp1->GetBBox().contains(vcp2->GetBBox()))
        {
            KIRI_LOG_DEBUG("Contains");
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
            //clipedPolygon->Print();
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

        for (size_t i = 0; i < mConvexHull->GetNumOfFacets(); i++)
        {
            auto facet = mConvexHull->GetFacets()[i];
            if (facet->IsVisibleFromBelow())
            {
                for (size_t e = 0; e < 3; e++)
                {
                    auto edge = facet->GetEdgesByIdx(e);

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
                        //KIRI_LOG_DEBUG("{0}", mFacetsAroundVertex.size());

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

                        cellPoly->UpdateBBox();
                        //cellPoly->PrintPolyVertices();
                        // cellPoly->ComputeVoroSitesList();

                        if (!site->IsBoundarySite())
                        {
                            // KIRI_LOG_DEBUG("------------NEW SITE COMPUTE---------------");
                            // KIRI_LOG_DEBUG("site idx = {0}, ComputeVoroCells mFacetsAroundVertex={1}", site->GetIdx(), mFacetsAroundVertex.size());
                            //site->Print();
                            auto newPoly = VoroCellPolygonClip(mBoundaryPolygon2, cellPoly);
                            // newPoly->UpdateBBox();
                            // newPoly->ComputeVoroSitesList();

                            //newPoly->Print();

                            if (newPoly != NULL)
                                site->SetCellPolygon(newPoly);
                            // else
                            //     ReGenVoroSites();
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