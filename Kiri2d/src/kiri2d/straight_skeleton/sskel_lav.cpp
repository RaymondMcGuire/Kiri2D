/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 11:03:44
 * @LastEditTime: 2021-07-22 22:17:53
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\straight_skeleton\sskel_lav.cpp
 */
#include <kiri2d/straight_skeleton/sskel_lav.h>
#include <kiri2d/straight_skeleton/sskel_utils.h>

namespace KIRI2D::SSKEL
{
    void SSkelLAV::PrintSSkelLAV()
    {
        KIRI_LOG_DEBUG("----------SSkelLAV----------");
        KIRI_LOG_DEBUG("LAV list number={0}", this->Size());

        if (mHead != NULL)
        {
            auto x = mHead;
            do
            {
                x->value->Print();
                x = x->next;
            } while (x != mHead);
        }

        KIRI_LOG_DEBUG("--------------------------------------");
    }

    void SSkelLAV::GenEvents()
    {
        if (mHead != NULL)
        {
            auto x = mHead;
            do
            {
                Vector<SSkelEdgeEventPtr> events;

                auto curr = x->value;
                auto curr_bi = curr->GetBisector();
                auto prev_bi = x->prev->value->GetBisector();
                auto next_bi = x->next->value->GetBisector();

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
                        _distance_point2_line2(curr->GetLeftPoint(), curr->GetPoint(), intersect),
                        intersect,
                        x->prev->value,
                        curr);

                    events.emplace_back(event);
                }

                if (istbi_next.z != 0.f)
                {
                    auto intersect = Vector2F(istbi_next.x, istbi_next.y);
                    auto event = std::make_shared<SSkelEdgeEvent>(
                        _distance_point2_line2(curr->GetPoint(), curr->GetRightPoint(), intersect),
                        intersect,
                        curr,
                        x->next->value);
                    events.emplace_back(event);
                }

                if (!events.empty())
                {
                    auto min = std::min_element(
                        events.begin(), events.end(),
                        [](const SSkelEdgeEventPtr &event1, const SSkelEdgeEventPtr &event2)
                        { return event1->GetDistance() < event2->GetDistance(); });

                    (*min)->Print();
                }

                x = x->next;
            } while (x != mHead);
        }
    }
}