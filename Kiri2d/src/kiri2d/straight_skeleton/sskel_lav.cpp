/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 11:03:44
 * @LastEditTime: 2021-07-23 14:38:48
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\straight_skeleton\sskel_lav.cpp
 */
#include <kiri2d/straight_skeleton/sskel_lav.h>
#include <kiri2d/straight_skeleton/sskel_utils.h>

namespace KIRI2D::SSKEL
{
    void SSkelLAV::Push(const SSkelVertexPtr &x)
    {
        if (mHead == NULL)
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

    void SSkelLAV::PrintSSkelLAV()
    {
        KIRI_LOG_DEBUG("----------SSkelLAV----------");
        KIRI_LOG_DEBUG("LAV list number={0}", this->Length());

        if (mHead != NULL)
        {
            auto x = mHead;
            do
            {
                x->Print();
                x = x->next;
            } while (x != mHead);
        }

        KIRI_LOG_DEBUG("--------------------------------------");
    }

    SSkelVertexPtr SSkelLAV::Unify(const SSkelVertexPtr &va, const SSkelVertexPtr &vb, Vector2F mid)
    {
        auto dir_vbb = Vector2F(vb->GetBisector().z, vb->GetBisector().w).normalized();
        auto dir_vab = Vector2F(va->GetBisector().z, va->GetBisector().w).normalized();
        auto newVertex =
            std::make_shared<SSkelVertex>(
                va->GetLeftPoint(),
                mid,
                vb->GetRightPoint(),
                Vector4F(dir_vbb.x, dir_vbb.y, dir_vab.x, dir_vab.y));
        if (mHead == va || mHead == vb)
            mHead = newVertex;

        va->prev->next = newVertex;
        vb->next->prev = newVertex;
        newVertex->prev = va->prev;
        newVertex->next = vb->next;

        va->SetInValid();
        vb->SetInValid();

        mCounter--;
        return newVertex;
    }

    SSkelEdgeEventPtr SSkelLAV::GenEdgeEventByVertex(const SSkelVertexPtr &vertex)
    {
        Vector<SSkelEdgeEventPtr> events;

        auto curr_bi = vertex->GetBisector();
        auto prev_bi = vertex->prev->GetBisector();
        auto next_bi = vertex->next->GetBisector();

        auto istbi_prev = _intersect_line2_line2(
            Vector2F(curr_bi.x, curr_bi.y),
            Vector2F(curr_bi.z, curr_bi.w),
            Vector2F(prev_bi.x, prev_bi.y),
            Vector2F(prev_bi.z, prev_bi.w));

        auto istbi_next = _intersect_line2_line2(
            Vector2F(curr_bi.x, curr_bi.y),
            Vector2F(curr_bi.z, curr_bi.w),
            Vector2F(next_bi.x, next_bi.y),
            Vector2F(next_bi.z, next_bi.w));

        if (istbi_prev.z != 0.f)
        {
            auto intersect = Vector2F(istbi_prev.x, istbi_prev.y);
            auto event = std::make_shared<SSkelEdgeEvent>(
                _distance_point2_line2(vertex->GetLeftPoint(), vertex->GetPoint(), intersect),
                intersect,
                vertex->prev,
                vertex);

            events.emplace_back(event);
        }

        if (istbi_next.z != 0.f)
        {
            auto intersect = Vector2F(istbi_next.x, istbi_next.y);
            auto event = std::make_shared<SSkelEdgeEvent>(
                _distance_point2_line2(vertex->GetPoint(), vertex->GetRightPoint(), intersect),
                intersect,
                vertex,
                vertex->next);
            events.emplace_back(event);
        }

        if (!events.empty())
        {
            auto min = std::min_element(
                events.begin(), events.end(),
                [](const SSkelEdgeEventPtr &event1, const SSkelEdgeEventPtr &event2)
                { return event1->GetDistance() < event2->GetDistance(); });

            return (*min);
        }

        return NULL;
    }

    void SSkelLAV::GenInitEvents()
    {
        // reset priority queue
        mPriorityQueue =
            std::priority_queue<
                SSkelEdgeEventPtr,
                Vector<SSkelEdgeEventPtr>,
                SSkelEdgeEventCmpDistance>();

        if (mHead != NULL)
        {
            auto x = mHead;
            do
            {
                auto edge_event = this->GenEdgeEventByVertex(x);
                if (edge_event != NULL)
                    mPriorityQueue.push(edge_event);

                x = x->next;
            } while (x != mHead);

            // debug prior queue
            // while (!mPriorityQueue.empty())
            // {

            //     auto p = mPriorityQueue.top();
            //     mPriorityQueue.pop();
            //     p->Print();
            // }
        }
    }

    Vector<SSkelEdgeEventPtr> SSkelLAV::HandleEdgeEvent(const SSkelEdgeEventPtr &edgeEvent)
    {
        Vec_Vec2F sinks;
        Vector<SSkelEdgeEventPtr> events;

        if (edgeEvent->GetVertA()->prev == edgeEvent->GetVertB()->next)
        {
            //todo remove lav
            if (mHead != NULL)
            {
                auto x = mHead;
                do
                {
                    sinks.emplace_back(x->GetPoint());
                    x->SetInValid();
                    x = x->next;
                } while (x != mHead);
            }
        }
        else
        {
            auto new_vertex = this->Unify(edgeEvent->GetVertA(), edgeEvent->GetVertB(), edgeEvent->GetIntersectPoint());
            sinks.emplace_back(edgeEvent->GetVertA()->GetPoint());
            sinks.emplace_back(edgeEvent->GetVertB()->GetPoint());

            auto edge_event = GenEdgeEventByVertex(new_vertex);
            if (edge_event != NULL)
                events.emplace_back(edge_event);
        }

        //
        auto skeleton = std::make_tuple(edgeEvent->GetIntersectPoint(), sinks);
        mSkeletons.emplace_back(skeleton);

        return events;
    }

    void SSkelLAV::HandleEvents()
    {
        //KIRI_LOG_DEBUG("-------------HandleEvents------------");
        while (!mPriorityQueue.empty())
        {
            auto event = mPriorityQueue.top();
            mPriorityQueue.pop();

            if (!event->GetVertA()->GetIsValid() || !event->GetVertB()->GetIsValid())
                continue;

            //event->Print();
            auto new_edge_events = HandleEdgeEvent(event);

            for (size_t i = 0; i < new_edge_events.size(); i++)
                mPriorityQueue.push(new_edge_events[i]);
        }
    }
}