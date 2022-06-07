/***
 * @Author: Xu.WANG
 * @Date: 2021-07-22 11:03:44
 * @LastEditTime: 2021-10-06 19:43:31
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\straight_skeleton\sskel_lav.cpp
 */
#include <kiri2d/straight_skeleton/sskel_lav.h>
#include <kiri2d/straight_skeleton/sskel_utils.h>

namespace KIRI2D::SSKEL
{
    void SSkelLAV::push(const SSkelVertexPtr &x)
    {
        if (mHead == nullptr)
        {
            mHead = x;
            mHead->next = mHead->prev = mHead;
            mCounter++;
        }
        else
        {
            auto new_node = x;
            mHead->prev->next = new_node;
            new_node->prev = mHead->prev;
            new_node->next = mHead;
            mHead->prev = new_node;
            mCounter++;
        }
    }

    void SSkelLAV::print()
    {
        KIRI_LOG_DEBUG("----------SSkelLAV----------");
        KIRI_LOG_DEBUG("LAV list number={0}", this->length());

        if (mHead != nullptr)
        {
            auto x = mHead;
            do
            {
                x->print();
                x = x->next.lock();
            } while (x != mHead);
        }

        KIRI_LOG_DEBUG("--------------------------------------");
    }

    Vector<std::tuple<Vector4F, Vector4F, Vector4F>> SSkelLAV::computeEdgesData()
    {
        // edge/ bisector left/ right
        Vector<std::tuple<Vector4F, Vector4F, Vector4F>> edges;

        if (mHead != nullptr)
        {
            auto x = mHead;
            for (size_t i = 0; i < mCounter; i++)
            {
                if (x == nullptr)
                    break;

                auto prev_point = x->prev->GetPoint();
                auto curr_point = x->GetPoint();
                auto edge = std::make_tuple(
                    Vector4F(prev_point.x, prev_point.y, curr_point.x, curr_point.y),
                    x->prev->GetBisector(),
                    x->GetBisector());

                edges.emplace_back(edge);

                x = x->next.lock();
            }

            // auto x = mHead;
            // do
            // {
            //     auto prev_point = x->prev->GetPoint();
            //     auto curr_point = x->GetPoint();
            //     auto edge = std::make_tuple(
            //         Vector4F(prev_point.x, prev_point.y, curr_point.x, curr_point.y),
            //         x->prev->GetBisector(),
            //         x->GetBisector());

            //     edges.emplace_back(edge);

            //     x = x->next.lock();
            // } while (x != mHead);
        }
        return edges;
    }

    SSkelVertexPtr SSkelLAV::unify(SSkelVertexPtr va, SSkelVertexPtr vb, Vector2F mid)
    {
        auto dir_vbb = Vector2F(vb->GetBisector().z, vb->GetBisector().w).normalized();
        auto dir_vab = Vector2F(va->GetBisector().z, va->GetBisector().w).normalized();
        auto newVertex =
            std::make_shared<SSkelVertex>(
                mId,
                va->GetLeftEdge(),
                mid,
                vb->GetRightEdge(),
                Vector4F(dir_vbb.x, dir_vbb.y, dir_vab.x, dir_vab.y));

        if (mHead == va || mHead == vb)
            mHead = newVertex;

        va->prev->next = newVertex;
        newVertex->prev = va->prev;

        // FIXME
        if (vb->next.expired())
        {
            newVertex->next = va->prev;
        }
        else
        {
            vb->next.lock()->prev = newVertex;
            newVertex->next = vb->next;
        }

        va->SetInValid();
        vb->SetInValid();

        mCounter--;
        return newVertex;
    }

    Vector<SSkelEventPtr> SSkelLAV::computeSplitEventByVertex(const SSkelVertexPtr &vertex, Vector<std::tuple<Vector4F, Vector4F, Vector4F>> originalEdges)
    {
        Vector<SSkelEventPtr> events;
        if (!vertex->GetIsReflex())
            return events;

        for (size_t i = 0; i < originalEdges.size(); i++)
        {
            auto [edge, left_bisector, right_bisector] = originalEdges[i];

            auto left = vertex->GetLeftEdge();
            auto right = vertex->GetRightEdge();
            auto mid = vertex->GetPoint();

            auto prev_dir = (Vector2F(left.z, left.w) - Vector2F(left.x, left.y)).normalized();
            auto next_dir = (Vector2F(right.z, right.w) - Vector2F(right.x, right.y)).normalized();

            auto edge_dir = (Vector2F(edge.z, edge.w) - Vector2F(edge.x, edge.y)).normalized();
            auto left_dot = abs(prev_dir.dot(edge_dir));
            auto right_dot = abs(next_dir.dot(edge_dir));

            auto self_edge = (left_dot < right_dot) ? left : right;

            auto ist_edge = _intersect_line2_line2(
                Vector2F(self_edge.x, self_edge.y),
                (Vector2F(self_edge.z, self_edge.w) - Vector2F(self_edge.x, self_edge.y)).normalized(),
                Vector2F(edge.x, edge.y),
                (Vector2F(edge.z, edge.w) - Vector2F(edge.x, edge.y)).normalized());

            auto ist_edge2 = Vector2F(ist_edge.x, ist_edge.y);
            if ((ist_edge.z != 0.f) && !_approximately_equals_point2(ist_edge2, mid))
            {
                auto lin_vec = (mid - ist_edge2).normalized();
                auto ed_vec = Vector2F(edge_dir);
                auto lin_dot_ed = lin_vec.dot(ed_vec);

                // ANCHOR !!!! epsilon
                if (lin_dot_ed < 0.f && abs(lin_dot_ed) > 1e-6)
                    ed_vec = -ed_vec;

                auto bisec_vec = ed_vec + lin_vec;

                if (bisec_vec.length() == 0.f)
                    continue;

                auto vert_bisector = vertex->GetBisector();
                auto b = _intersect_line2_ray2(
                    Vector2F(ist_edge.x, ist_edge.y),
                    Vector2F(bisec_vec.x, bisec_vec.y),
                    Vector2F(vert_bisector.x, vert_bisector.y),
                    Vector2F(vert_bisector.z, vert_bisector.w));

                if (b.z == 0.f)
                    continue;

                auto b2 = Vector2F(b.x, b.y);
                auto x_left = Vector2F(left_bisector.z, left_bisector.w).cross((b2 - Vector2F(left_bisector.x, left_bisector.y)).normalized()) > -std::numeric_limits<float>::epsilon();
                auto x_right = Vector2F(right_bisector.z, right_bisector.w).cross((b2 - Vector2F(right_bisector.x, right_bisector.y)).normalized()) < std::numeric_limits<float>::epsilon();
                auto x_edge = edge_dir.cross((b2 - Vector2F(edge.x, edge.y)).normalized()) < std::numeric_limits<float>::epsilon();

                if (!(x_left && x_right && x_edge))
                    continue;

                // KIRI_LOG_DEBUG("----debug split event----");
                // KIRI_LOG_DEBUG("b={0},{1}", b.x, b.y);
                // KIRI_LOG_DEBUG("dis={0}", _distance_point2_line2(Vector2F(edge.x, edge.y), Vector2F(edge.z, edge.w), b2));

                auto dis = _distance_point2_line2(Vector2F(edge.x, edge.y), Vector2F(edge.z, edge.w), b2);
                auto split_event = std::make_shared<SSkelSplitEvent>(
                    dis,
                    b2,
                    vertex,
                    edge);
                // split_event->print();
                if (dis == dis)
                    events.emplace_back(split_event);
            }
        }

        return events;
    }

    SSkelEventPtr SSkelLAV::computeEventByVertex(const SSkelVertexPtr &vertex, Vector<std::tuple<Vector4F, Vector4F, Vector4F>> originalEdges)
    {
        Vector<SSkelEventPtr> events;

        auto split_event = computeSplitEventByVertex(vertex, originalEdges);
        if (!split_event.empty())
            events.insert(events.end(), split_event.begin(), split_event.end());

        // for edge event
        auto curr_bi = vertex->GetBisector();
        auto prev_bi = vertex->prev->GetBisector();
        auto next_bi = vertex->next.lock()->GetBisector();

        auto istbi_prev = _intersect_ray2_ray2(
            Vector2F(curr_bi.x, curr_bi.y),
            Vector2F(curr_bi.z, curr_bi.w),
            Vector2F(prev_bi.x, prev_bi.y),
            Vector2F(prev_bi.z, prev_bi.w));

        auto istbi_next = _intersect_ray2_ray2(
            Vector2F(curr_bi.x, curr_bi.y),
            Vector2F(curr_bi.z, curr_bi.w),
            Vector2F(next_bi.x, next_bi.y),
            Vector2F(next_bi.z, next_bi.w));

        // KIRI_LOG_DEBUG("debug!!!!!!!");
        // KIRI_LOG_DEBUG("prev={0},{1};next={2},{3}", istbi_prev.x, istbi_prev.y, istbi_next.x, istbi_next.y);

        if (istbi_prev.z != 0.f)
        {
            auto left_edge = vertex->GetLeftEdge();
            auto intersect = Vector2F(istbi_prev.x, istbi_prev.y);
            auto dis = _distance_point2_line2(Vector2F(left_edge.x, left_edge.y), Vector2F(left_edge.z, left_edge.w), intersect);
            auto event = std::make_shared<SSkelEdgeEvent>(
                dis,
                intersect,
                vertex->prev,
                vertex);

            // FIXME
            if (dis == dis)
                events.emplace_back(event);
        }

        if (istbi_next.z != 0.f)
        {
            auto right_edge = vertex->GetRightEdge();
            auto intersect = Vector2F(istbi_next.x, istbi_next.y);
            auto dis = _distance_point2_line2(Vector2F(right_edge.x, right_edge.y), Vector2F(right_edge.z, right_edge.w), intersect);
            auto event = std::make_shared<SSkelEdgeEvent>(
                dis,
                intersect,
                vertex,
                vertex->next.lock());

            // FIXME
            if (dis == dis)
                events.emplace_back(event);
        }

        if (!events.empty())
        {
            auto min = std::min_element(
                events.begin(), events.end(),
                [=](const SSkelEventPtr &event1, const SSkelEventPtr &event2)
                { return event1->intersectPoint().distanceTo(vertex->GetPoint()) < event2->intersectPoint().distanceTo(vertex->GetPoint()); });

            //(*min)->print();
            return (*min);
        }

        return nullptr;
    }

    Vector<SSkelEventPtr> SSkelLAV::computeEvents(Vector<std::tuple<Vector4F, Vector4F, Vector4F>> originalEdges)
    {
        Vector<SSkelEventPtr> events;

        if (mHead != nullptr)
        {

            auto x = mHead;
            for (size_t i = 0; i < mCounter; i++)
            {
                if (x == nullptr)
                    break;

                auto edge_event = this->computeEventByVertex(x, originalEdges);
                if (edge_event != nullptr)
                    events.emplace_back(edge_event);

                x = x->next.lock();
            }

            // auto x = mHead;
            // do
            // {
            //     auto edge_event = this->computeEventByVertex(x, originalEdges);
            //     if (edge_event != nullptr)
            //         events.emplace_back(edge_event);

            //     x = x->next.lock();
            // } while (x != mHead);
        }

        return events;
    }

}