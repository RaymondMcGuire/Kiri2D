/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-10-16 15:11:38
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-16 15:22:15
 * @FilePath: \Kiri2D\core\src\kiri2d\geo\convex_clip2.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d/geo/convex_clip2.h>

namespace KIRI
{

    Int KiriConvexClip2::signedAreasOfTriangle2(const Vector2F &v1, const Vector2F &v2, const Vector2F &v3)
    {
        auto area = (v2.x - v1.x) * (v3.y - v1.y) - (v3.x - v1.x) * (v2.y - v1.y);
        if (area > std::numeric_limits<float>::epsilon())
            return 1;
        else if (area < -std::numeric_limits<float>::epsilon())
            return -1;
        else
            return 0;
    }

    bool KiriConvexClip2::isConvex(const KiriVector2ListPtr &list)
    {
        if (list->Size() < 3)
            return false;

        auto cur = list->head()->next;
        while (cur != list->head())
        {
            if (signedAreasOfTriangle2(cur->value, cur->next->value, cur->next->next->value) < 0)
                return false;

            cur = cur->next;
        }
        return true;
    }

    bool KiriConvexClip2::isBetween(const Vector2F &v1, const Vector2F &v2, const Vector2F &v3)
    {
        if (v1.x != v2.x)
            return (v3.x >= v1.x && v3.x <= v2.x) || (v3.x <= v1.x && v3.x >= v2.x);
        else
            return (v3.y >= v1.y && v3.y <= v2.y) || (v3.y <= v1.y && v3.y >= v2.y);
    }

    LineSegment KiriConvexClip2::computeLineIntersection(const Vector2F &a1, const Vector2F &a2, const Vector2F &b1, const Vector2F &b2)
    {
        auto val = ((b2.x - b1.x) * (a2.y - a1.y) - (a2.x - a1.x) * (b2.y - b1.y));
        if (val == 0.f)
        {
            if (signedAreasOfTriangle2(a1, a2, b1) == 0)
            {
                if (isBetween(a1, a2, b1) && isBetween(a1, a2, b2))
                {
                    return LineSegment(b1, b2, 'e');
                }
                if (isBetween(b1, b2, a1) && isBetween(b1, b2, a2))
                {
                    return LineSegment(a1, a2, 'e');
                }
                if (isBetween(a1, a2, b1) && isBetween(b1, b2, a2))
                {
                    return LineSegment(b1, a2, 'e');
                }
                if (isBetween(a1, a2, b1) && isBetween(b1, b2, a1))
                {
                    return LineSegment(b1, a1, 'e');
                }
                if (isBetween(a1, a2, b2) && isBetween(b1, b2, a2))
                {
                    return LineSegment(b2, a2, 'e');
                }
                if (isBetween(a1, a2, b2) && isBetween(b1, b2, a1))
                {
                    return LineSegment(b2, a1, 'e');
                }
            }
            return LineSegment(Vector2F(0.f), Vector2F(0.f), 'n');
        }

        auto t = (a1.x * (b2.y - b1.y) - a1.y * (b2.x - b1.x) + b1.y * (b2.x - b1.x) - b1.x * (b2.y - b1.y)) / val;
        auto s = ((a2.x - a1.x) * a1.y + b1.x * (a2.y - a1.y) - a1.x * (a2.y - a1.y) - b1.y * (a2.x - a1.x)) / -val;

        if (t >= 0.f && t <= 1.f && s >= 0.f && s <= 1.f)
            return LineSegment(Vector2F(a1.x + t * (a2.x - a1.x), a1.y + t * (a2.y - a1.y)), Vector2F(0.f), '1');

        return LineSegment(Vector2F(0.f), Vector2F(0.f), 'n');
    }

    void KiriConvexClip2::convexIntersection(const KiriVector2ListPtr &p, const KiriVector2ListPtr &q)
    {
        InsideFlag flag = IF_UNKNOWN;
        auto curP = p->head();
        auto curQ = q->head();

        auto firstPoint = true;
        auto advP = 0, advQ = 0;
        do
        {
            auto pqIntersec = computeLineIntersection(Vector2F(curP->prev->value), Vector2F(curP->value), Vector2F(curQ->prev->value), Vector2F(curQ->value));
            auto vQ = Vector2F(curQ->value.x - curQ->prev->value.x, curQ->value.y - curQ->prev->value.y);
            auto vP = Vector2F(curP->value.x - curP->prev->value.x, curP->value.y - curP->prev->value.y);

            auto cross = signedAreasOfTriangle2(Vector2F(0.f), Vector2F(vP), Vector2F(vQ));
            auto pInQ = signedAreasOfTriangle2(Vector2F(curQ->prev->value), Vector2F(curQ->value), Vector2F(curP->value));
            auto qInP = signedAreasOfTriangle2(Vector2F(curP->prev->value), Vector2F(curP->value), Vector2F(curQ->value));

            if (pqIntersec.code == '1')
            {
                if (flag == IF_UNKNOWN && firstPoint)
                {
                    firstPoint = false;
                    advP = advQ = 0;
                }

                mIntersectionList->push(Vector2F(pqIntersec.start));
                if (pInQ > 0)
                {
                    flag = IF_PINSIDE;
                }
                else if (qInP > 0)
                {
                    flag = IF_QINSIDE;
                }
            }

            if (pqIntersec.code == 'e' && vP.dot(vQ) < 0)
            {
                mIntersectionList->push(Vector2F(pqIntersec.start));
                mIntersectionList->push(Vector2F(pqIntersec.end));
                return;
            }

            if (cross == 0 && pInQ < 0 && qInP < 0)
            {
                return;
            }
            else if (cross == 0 && pInQ == 0 && qInP == 0)
            {
                if (flag == IF_PINSIDE)
                {
                    ++advQ;
                    curQ = curQ->next;
                }
                else
                {
                    ++advP;
                    curP = curP->next;
                }
            }
            else if (cross >= 0)
            {
                if (qInP > 0)
                {
                    if (flag == IF_PINSIDE)
                        mIntersectionList->push(Vector2F(curP->value));

                    ++advP;
                    curP = curP->next;
                }
                else
                {
                    if (flag == IF_QINSIDE)
                        mIntersectionList->push(Vector2F(curQ->value));
                    ++advQ;
                    curQ = curQ->next;
                }
            }
            else
            {
                if (pInQ > 0)
                {
                    if (flag == IF_QINSIDE)
                        mIntersectionList->push(Vector2F(curQ->value));
                    ++advQ;
                    curQ = curQ->next;
                }
                else
                {
                    if (flag == IF_PINSIDE)
                        mIntersectionList->push(Vector2F(curP->value));
                    ++advP;
                    curP = curP->next;
                }
            }

        } while (((advP < p->Size()) || (advQ < q->Size())) && (advP < 2 * p->Size()) && (advQ < 2 * q->Size()));
    }

    bool KiriConvexClip2::computeConvexPolygonIntersection(const KiriVector2ListPtr &a, const KiriVector2ListPtr &b)
    {
        auto p = std::make_shared<KiriVector2List>();
        auto q = std::make_shared<KiriVector2List>();
        a->clone(p);
        b->clone(q);

        if (!isConvex(p))
        {
            p->reverseVertexList();
            if (!isConvex(p))
            {
                // KIRI_LOG_ERROR("computeConvexPolygonIntersection: Polygons are not Convex");
                return false;
            }
        }
        if (!isConvex(q))
        {
            q->reverseVertexList();
            if (!isConvex(q))
            {
                // KIRI_LOG_ERROR("computeConvexPolygonIntersection: Polygons are not Convex");
                return false;
            }
        }

        mIntersectionList->removeAll();
        convexIntersection(p, q);

        return true;
    }
}