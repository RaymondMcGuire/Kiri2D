/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:33:21
 * @LastEditTime: 2021-06-02 23:41:21
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\geo\convex_clip2.cpp
 */

#include <kiri2d/geo/convex_clip2.h>

namespace KIRI
{

    Int KiriConvexClip2::GetSignedAreasOfTriangle2(const Vector2F &v1, const Vector2F &v2, const Vector2F &v3)
    {
        auto area = (v2.x - v1.x) * (v3.y - v1.y) - (v3.x - v1.x) * (v2.y - v1.y);
        if (area > MEpsilon<float>())
            return 1;
        else if (area < -MEpsilon<float>())
            return -1;
        else
            return 0;
    }

    bool KiriConvexClip2::IsConvex(const KiriVector2ListPtr &list)
    {
        if (list->Size() < 3)
            return false;

        auto cur = list->GetHead()->next;
        while (cur != list->GetHead())
        {
            if (GetSignedAreasOfTriangle2(cur->value, cur->next->value, cur->next->next->value) < 0)
                return false;

            cur = cur->next;
        }
        return true;
    }

    bool KiriConvexClip2::IsBetween(const Vector2F &v1, const Vector2F &v2, const Vector2F &v3)
    {
        if (v1.x != v2.x)
            return (v3.x >= v1.x && v3.x <= v2.x) || (v3.x <= v1.x && v3.x >= v2.x);
        else
            return (v3.y >= v1.y && v3.y <= v2.y) || (v3.y <= v1.y && v3.y >= v2.y);
    }

    LineSegment KiriConvexClip2::ComputeLineIntersection(const Vector2F &a1, const Vector2F &a2, const Vector2F &b1, const Vector2F &b2)
    {
        auto val = ((b2.x - b1.x) * (a2.y - a1.y) - (a2.x - a1.x) * (b2.y - b1.y));
        if (val == 0.f)
        {
            // collinear
            if (GetSignedAreasOfTriangle2(a1, a2, b1) == 0)
            {
                if (IsBetween(a1, a2, b1) && IsBetween(a1, a2, b2))
                {
                    return LineSegment(b1, b2, 'e');
                }
                if (IsBetween(b1, b2, a1) && IsBetween(b1, b2, a2))
                {
                    return LineSegment(a1, a2, 'e');
                }
                if (IsBetween(a1, a2, b1) && IsBetween(b1, b2, a2))
                {
                    return LineSegment(b1, a2, 'e');
                }
                if (IsBetween(a1, a2, b1) && IsBetween(b1, b2, a1))
                {
                    return LineSegment(b1, a1, 'e');
                }
                if (IsBetween(a1, a2, b2) && IsBetween(b1, b2, a2))
                {
                    return LineSegment(b2, a2, 'e');
                }
                if (IsBetween(a1, a2, b2) && IsBetween(b1, b2, a1))
                {
                    return LineSegment(b2, a1, 'e');
                }
            }
            // no intersection
            return LineSegment(Vector2F(0.f), Vector2F(0.f), 'n');
        }

        auto t = (a1.x * (b2.y - b1.y) - a1.y * (b2.x - b1.x) + b1.y * (b2.x - b1.x) - b1.x * (b2.y - b1.y)) / val;
        auto s = ((a2.x - a1.x) * a1.y + b1.x * (a2.y - a1.y) - a1.x * (a2.y - a1.y) - b1.y * (a2.x - a1.x)) / -val;

        // proper intersection
        if (t >= 0.f && t <= 1.f && s >= 0.f && s <= 1.f)
            return LineSegment(Vector2F(a1.x + t * (a2.x - a1.x), a1.y + t * (a2.y - a1.y)), Vector2F(0.f), '1');

        return LineSegment(Vector2F(0.f), Vector2F(0.f), 'n');
    }

    void KiriConvexClip2::ConvexIntersection(const KiriVector2ListPtr &p, const KiriVector2ListPtr &q)
    {
        InsideFlag flag = IF_UNKNOWN;
        auto curP = p->GetHead();
        auto curQ = q->GetHead();

        // flag whether first point or not
        auto firstPoint = true;

        // counter for the termination condition(advP = advance p,advQ = advance q)
        auto advP = 0, advQ = 0;
        do
        {
            // intersection of polygon p and q
            auto pqIntersec = ComputeLineIntersection(Vector2F(curP->prev->value), Vector2F(curP->value), Vector2F(curQ->prev->value), Vector2F(curQ->value));
            auto vQ = Vector2F(curQ->value.x - curQ->prev->value.x, curQ->value.y - curQ->prev->value.y);
            auto vP = Vector2F(curP->value.x - curP->prev->value.x, curP->value.y - curP->prev->value.y);

            // sign of cross product of vP and vQ
            auto cross = GetSignedAreasOfTriangle2(Vector2F(0.f), Vector2F(vP), Vector2F(vQ));

            // if curP is on the half plane of q
            auto pInQ = GetSignedAreasOfTriangle2(Vector2F(curQ->prev->value), Vector2F(curQ->value), Vector2F(curP->value));
            // if curQ is on the half plane of p
            auto qInP = GetSignedAreasOfTriangle2(Vector2F(curP->prev->value), Vector2F(curP->value), Vector2F(curQ->value));

            // proper intersection
            if (pqIntersec.code == '1')
            {
                if (flag == IF_UNKNOWN && firstPoint)
                {
                    firstPoint = false;
                    advP = advQ = 0;
                }

                // adding the intersection to the result
                mIntersectionList->Push(Vector2F(pqIntersec.start));
                //Flag update
                if (pInQ > 0)
                {
                    flag = IF_PINSIDE;
                }
                else if (qInP > 0)
                {
                    flag = IF_QINSIDE;
                }
            }

            // Advance Rules:
            // vP and vQ overlap and oppositely oriented
            if (pqIntersec.code == 'e' && vP.dot(vQ) < 0)
            {
                mIntersectionList->Push(Vector2F(pqIntersec.start));
                mIntersectionList->Push(Vector2F(pqIntersec.end));
                return;
            }
            // vP and vQ are parallel and separated. p and q are disjoint!
            if (cross == 0 && pInQ < 0 && qInP < 0)
            {
                return;
                //vP and vQ are collinear
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
                /* Generic cases
			 * cross  		halfplane condition   advance rule
			 *  >0				qInP == 1			 p
			 *  >0				qInP == 0			 q
			 *  <0				pInQ == 1			 q
			 *  <0				pInQ == 0			 p
			 *  if p is advanced and inside flag = p then add curP to result (equivalent to q)
			 */
            }
            else if (cross >= 0)
            {
                if (qInP > 0)
                {
                    if (flag == IF_PINSIDE)
                        mIntersectionList->Push(Vector2F(curP->value));

                    ++advP;
                    curP = curP->next;
                }
                else
                {
                    if (flag == IF_QINSIDE)
                        mIntersectionList->Push(Vector2F(curQ->value));
                    ++advQ;
                    curQ = curQ->next;
                }
            }
            else
            { //cross < 0
                if (pInQ > 0)
                {
                    if (flag == IF_QINSIDE)
                        mIntersectionList->Push(Vector2F(curQ->value));
                    ++advQ;
                    curQ = curQ->next;
                }
                else
                {
                    if (flag == IF_PINSIDE)
                        mIntersectionList->Push(Vector2F(curP->value));
                    ++advP;
                    curP = curP->next;
                }
            }
            /*
            * Termination condition: if ap >= n and aq >=m then both polygons traversed
            * if ap >= 2*n or aq >= 2*m then p or q cycled twice and there will not be another intersection.
            */
        } while (((advP < p->Size()) || (advQ < q->Size())) && (advP < 2 * p->Size()) && (advQ < 2 * q->Size()));
    }

    bool KiriConvexClip2::ComputeConvexPolygonIntersection(const KiriVector2ListPtr &a, const KiriVector2ListPtr &b)
    {
        auto p = std::make_shared<KiriVector2List>();
        auto q = std::make_shared<KiriVector2List>();
        a->Clone(p);
        b->Clone(q);

        //q->PrintVertexList();

        //KIRI_LOG_DEBUG("ComputeConvexPolygonIntersection: a size={0},b size={1},p size={2},q size={3}", a->Size(), b->Size(), p->Size(), q->Size());

        if (!IsConvex(p))
        {                           //Check for convexity
            p->ReverseVertexList(); //If list is not oriented counterclockwise
            if (!IsConvex(p))
            {
                KIRI_LOG_ERROR("ComputeConvexPolygonIntersection: Polygons are not Convex");
                return false;
            }
        }
        if (!IsConvex(q))
        {                           //Check for convexity
            q->ReverseVertexList(); //If list is not oriented counterclockwise
            if (!IsConvex(q))
            {
                KIRI_LOG_ERROR("ComputeConvexPolygonIntersection: Polygons are not Convex");
                return false;
            }
        }

        mIntersectionList->RemoveAll();
        ConvexIntersection(p, q);

        return true;
    }
}