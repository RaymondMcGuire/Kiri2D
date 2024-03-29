/***
 * @Author: Xu.WANG
 * @Date: 2021-07-22 11:03:44
 * @LastEditTime: 2021-07-25 13:27:24
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\straight_skeleton\sskel_slav.cpp
 */
#include <kiri2d/straight_skeleton/sskel_slav.h>

namespace KIRI2D::SSKEL
{

    Vector<SSkelEventPtr> SSkelSLAV::HandleEdgeEvent(const SSkelEdgeEventPtr &edgeEvent)
    {
        Vec_Vec2F sinks;
        Vector<SSkelEventPtr> events;

        auto lav = mLAVMaps[edgeEvent->vertA()->GetLAVId()];
        if (lav != nullptr)
        {
            if (edgeEvent->vertA()->prev == edgeEvent->vertB()->next.lock())
            {

                auto x = lav->head();
                auto len = lav->length();
                for (size_t i = 0; i < len; i++)
                {
                    if (x == nullptr)
                        break;

                    sinks.emplace_back(x->GetPoint());
                    x->SetInValid();
                    x = x->next.lock();
                }

                mLAVMaps.erase(lav->id());

                // //todo remove lav
                // if (head != nullptr)
                // {
                //     // KIRI_LOG_DEBUG("-----Edge-----");
                //     // lav->print();
                //     auto x = head;
                //     do
                //     {

                //         sinks.emplace_back(x->GetPoint());
                //         x->SetInValid();
                //         x = x->next.lock();

                //         if (x == head)
                //         {
                //             KIRI_LOG_DEBUG("-----x==head-----");
                //             x->print();
                //         }
                //     } while (x != head);
                // }
            }
            else
            {

                // edgeEvent->print();
                auto new_vertex = lav->unify(edgeEvent->vertA(), edgeEvent->vertB(), edgeEvent->intersectPoint());
                sinks.emplace_back(edgeEvent->vertA()->GetPoint());
                sinks.emplace_back(edgeEvent->vertB()->GetPoint());

                if ((lav->head() == edgeEvent->vertA()) || (lav->head() == edgeEvent->vertB()))
                    lav->setHead(new_vertex);

                auto new_event = lav->computeEventByVertex(new_vertex, mEdges);
                if (new_event != nullptr)
                    events.emplace_back(new_event);
            }
        }

        if (!sinks.empty())
        {
            auto skeleton = std::make_tuple(edgeEvent->intersectPoint(), sinks);
            mSkeletons.emplace_back(skeleton);
        }

        return events;
    }

    Vector<SSkelEventPtr> SSkelSLAV::HandleSplitEvent(const SSkelSplitEventPtr &splitEvent)
    {
        Vec_Vec2F sinks;
        Vector<SSkelEventPtr> events;
        sinks.emplace_back(splitEvent->vert()->GetPoint());

        auto opposite_edge = splitEvent->oppositeEdge();
        auto opposite_edge_dir = (Vector2F(opposite_edge.z, opposite_edge.w) - Vector2F(opposite_edge.x, opposite_edge.y)).normalized();

        SSkelVertexPtr right_vertex, left_vertex = nullptr;

        for (auto _lav : mLAVMaps)
        {
            if (_lav.second == nullptr)
                continue;

            auto x = _lav.second->head();
            auto len = _lav.second->length();
            for (size_t i = 0; i < len; i++)
            {
                if (x == nullptr)
                    break;

                auto edge_left = x->GetLeftEdge();
                auto edge_left_dir = (Vector2F(edge_left.z, edge_left.w) - Vector2F(edge_left.x, edge_left.y)).normalized();
                auto edge_right = x->GetRightEdge();
                auto edge_right_dir = (Vector2F(edge_right.z, edge_right.w) - Vector2F(edge_right.x, edge_right.y)).normalized();
                if ((opposite_edge_dir == edge_left_dir) && (Vector2F(edge_left.x, edge_left.y) == Vector2F(opposite_edge.x, opposite_edge.y)))
                {
                    right_vertex = x;
                    left_vertex = x->prev;
                }
                else if ((opposite_edge_dir == edge_right_dir) && (Vector2F(edge_right.x, edge_right.y) == Vector2F(opposite_edge.x, opposite_edge.y)))
                {
                    left_vertex = x;
                    right_vertex = x->next.lock();
                }

                if (right_vertex != nullptr)
                {
                    auto left_bi_norm = Vector2F(left_vertex->GetBisector().z, left_vertex->GetBisector().w).normalized();
                    auto right_bi_norm = Vector2F(right_vertex->GetBisector().z, right_vertex->GetBisector().w).normalized();
                    auto x_left = left_bi_norm.cross((splitEvent->intersectPoint() - left_vertex->GetPoint()).normalized()) > -std::numeric_limits<float>::epsilon();
                    auto x_right = right_bi_norm.cross((splitEvent->intersectPoint() - right_vertex->GetPoint()).normalized()) < std::numeric_limits<float>::epsilon();

                    if (x_left && x_right)
                        break;
                    else
                    {
                        left_vertex = nullptr;
                        right_vertex = nullptr;
                    }
                }

                x = x->next.lock();
            }

            // auto head = _lav.second->head();
            // if (head != nullptr)
            // {
            //     // KIRI_LOG_DEBUG("-----Split-----");
            //     // _lav.second->print();
            //     auto x = head;
            //     do
            //     {
            //         auto edge_left = x->GetLeftEdge();
            //         auto edge_left_dir = (Vector2F(edge_left.z, edge_left.w) - Vector2F(edge_left.x, edge_left.y)).normalized();
            //         auto edge_right = x->GetRightEdge();
            //         auto edge_right_dir = (Vector2F(edge_right.z, edge_right.w) - Vector2F(edge_right.x, edge_right.y)).normalized();
            //         if ((opposite_edge_dir == edge_left_dir) && (Vector2F(edge_left.x, edge_left.y) == Vector2F(opposite_edge.x, opposite_edge.y)))
            //         {
            //             right_vertex = x;
            //             left_vertex = x->prev;
            //         }
            //         else if ((opposite_edge_dir == edge_right_dir) && (Vector2F(edge_right.x, edge_right.y) == Vector2F(opposite_edge.x, opposite_edge.y)))
            //         {
            //             left_vertex = x;
            //             right_vertex = x->next.lock();
            //         }

            //         if (right_vertex != nullptr)
            //         {
            //             auto left_bi_norm = Vector2F(left_vertex->GetBisector().z, left_vertex->GetBisector().w).normalized();
            //             auto right_bi_norm = Vector2F(right_vertex->GetBisector().z, right_vertex->GetBisector().w).normalized();
            //             auto x_left = left_bi_norm.cross((splitEvent->intersectPoint() - left_vertex->GetPoint()).normalized()) > -std::numeric_limits<float>::epsilon();
            //             auto x_right = right_bi_norm.cross((splitEvent->intersectPoint() - right_vertex->GetPoint()).normalized()) < std::numeric_limits<float>::epsilon();

            //             if (x_left && x_right)
            //                 break;
            //             else
            //             {
            //                 left_vertex = nullptr;
            //                 right_vertex = nullptr;
            //             }
            //         }

            //         x = x->next.lock();
            //         if (x == head)
            //         {
            //             KIRI_LOG_DEBUG("-----x==head-----");
            //             x->print();
            //         }
            //     } while (x != head);
            //}
        }

        if (right_vertex == nullptr)
            return events;

        auto newVertex1 =
            std::make_shared<SSkelVertex>(
                mLAVCounter,
                splitEvent->vert()->GetLeftEdge(),
                splitEvent->intersectPoint(),
                splitEvent->oppositeEdge());

        auto newVertex2 =
            std::make_shared<SSkelVertex>(
                mLAVCounter + 1,
                splitEvent->oppositeEdge(),
                splitEvent->intersectPoint(),
                splitEvent->vert()->GetRightEdge());

        newVertex1->prev = splitEvent->vert()->prev;
        newVertex1->next = right_vertex;
        splitEvent->vert()->prev->next = newVertex1;
        right_vertex->prev = newVertex1;

        newVertex2->prev = left_vertex;
        newVertex2->next = splitEvent->vert()->next;
        splitEvent->vert()->next.lock()->prev = newVertex2;
        left_vertex->next = newVertex2;

        Vector<SSkelLAVPtr> new_lavs;
        // remove current lav
        auto cur_lav = mLAVMaps[splitEvent->vert()->GetLAVId()];

        if (cur_lav != nullptr)
            mLAVMaps.erase(splitEvent->vert()->GetLAVId());

        if (cur_lav == nullptr || (cur_lav->id() != right_vertex->GetLAVId()))
        {
            mLAVMaps.erase(right_vertex->GetLAVId());
            new_lavs.emplace_back(std::make_shared<SSkelLAV>(mLAVCounter++, newVertex1));
        }
        else
        {
            new_lavs.emplace_back(std::make_shared<SSkelLAV>(mLAVCounter++, newVertex1));
            new_lavs.emplace_back(std::make_shared<SSkelLAV>(mLAVCounter++, newVertex2));
        }

        Vector<SSkelVertexPtr> event_vertices;
        for (size_t i = 0; i < new_lavs.size(); i++)
        {
            auto elem_lav = new_lavs[i];
            auto elem_lav_head = elem_lav->head();
            if (elem_lav->length() > 2)
            {
                mLAVMaps[elem_lav->id()] = elem_lav;
                event_vertices.emplace_back(elem_lav_head);
            }
            else
            {
                sinks.emplace_back(elem_lav_head->next.lock()->GetPoint());

                if (elem_lav_head != nullptr)
                {

                    auto len = elem_lav->length();
                    auto x = elem_lav_head;
                    for (size_t i = 0; i < len; i++)
                    {
                        x->SetInValid();
                        x = x->next.lock();
                    }

                    // auto x = elem_lav_head;
                    // do
                    // {
                    //     x->SetInValid();
                    //     x = x->next.lock();
                    // } while (x != elem_lav_head && x != nullptr);
                }
            }
        }

        for (size_t i = 0; i < event_vertices.size(); i++)
        {
            auto vert_lav_id = event_vertices[i]->GetLAVId();
            auto new_event = mLAVMaps[vert_lav_id]->computeEventByVertex(event_vertices[i], mEdges);

            if (new_event != nullptr)
                events.emplace_back(new_event);
        }

        splitEvent->vert()->SetInValid();

        auto skeleton = std::make_tuple(splitEvent->intersectPoint(), sinks);
        mSkeletons.emplace_back(skeleton);

        return events;
    }

    void SSkelSLAV::HandleEvents()
    {
        // KIRI_LOG_DEBUG("-------------HandleEvents------------");

        while (!(mPriorityQueue.empty() || mLAVMaps.empty()))
        {

            auto event = mPriorityQueue.top();
            mPriorityQueue.pop();
            // event->print();

            Vector<SSkelEventPtr> new_events;
            if (IsInstanceOf<SSkelEdgeEvent>(event))
            {

                auto edge_event = std::dynamic_pointer_cast<SSkelEdgeEvent>(event);

                if (!edge_event->vertA()->GetIsValid() || !edge_event->vertB()->GetIsValid())
                    continue;

                auto edge_events = HandleEdgeEvent(edge_event);
                new_events.insert(new_events.end(), edge_events.begin(), edge_events.end());
            }
            else if (IsInstanceOf<SSkelSplitEvent>(event))
            {

                auto split_event = std::dynamic_pointer_cast<SSkelSplitEvent>(event);

                if (!split_event->vert()->GetIsValid())
                    continue;

                auto split_events = HandleSplitEvent(split_event);
                new_events.insert(new_events.end(), split_events.begin(), split_events.end());
            }

            for (size_t i = 0; i < new_events.size(); i++)
                mPriorityQueue.push(new_events[i]);
        }
    }
}