/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-17 19:49:52
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_cell_polygon2.cpp
 */

#include <kiri2d/voronoi/voro_cell_polygon2.h>
#include <kiri2d/voronoi/voro_util.h>
#include <random>

namespace KIRI
{

    bool KiriVoroCellPolygon2::IsClockwise(const Vector<Vector4F> &poly)
    {
        auto a = 0.f;
        auto polySize = poly.size();
        for (size_t i = 0; i < polySize; i++)
        {
            auto s = Vector2F(poly[i].x, poly[i].y);
            auto e = Vector2F(poly[i].z, poly[i].w);
            a += (e.x - s.x) * (e.y + s.y);
        }

        return a < 0.f;
    }

    void KiriVoroCellPolygon2::ComputeBisectors(const Vector<Vector4F> &poly, float lambda)
    {
        auto cw = IsClockwise(poly);
        auto polySize = poly.size();

        mBisectors.clear();

        for (size_t i = polySize; i < polySize * 2; i++)
        {
            auto eprev = poly[(i - 1) % polySize];
            auto ecurr = poly[i % polySize];
            auto p1 = Vector2F(eprev.x, eprev.y);
            auto p2 = Vector2F(eprev.z, eprev.w);
            auto p3 = Vector2F(ecurr.z, ecurr.w);
            auto d = DirectionBetween2Edges2(p1, p2, p3, cw);
            mBisectors.emplace_back(d * lambda);
        }
    }

    IntersectionStatus KiriVoroCellPolygon2::ComputeShrink(const Vector<Vector4F> &poly, float lambda)
    {

        ComputeBisectors(poly, lambda);

        IntersectionStatus status;
        Vector<Vector4F> shrink;

        status.intersection = false;
        auto updateNextPoint = false;
        auto curPoint = Vector2F(0.f);
        auto nextPoint = Vector2F(0.f);

        auto polySize = poly.size();
        for (size_t i = 0; i < polySize; i++)
        {
            auto s = Vector2F(poly[i].x, poly[i].y);
            auto e = Vector2F(poly[i].z, poly[i].w);

            if (updateNextPoint)
                updateNextPoint = false;
            else
                curPoint = s + mBisectors[i];

            if (i == polySize - 1)
                nextPoint = Vector2F(shrink[0].x, shrink[0].y);
            else
                nextPoint = e + mBisectors[i + 1];

            auto intersection = IntersectionPoint2(s, curPoint, e, nextPoint);
            if (intersection.z == 1.f)
            {
                status.intersection = true;
                updateNextPoint = true;
                curPoint = nextPoint = Vector2F(intersection.x, intersection.y);

                if (shrink.size() > 0)
                    shrink[shrink.size() - 1] = Vector4F(shrink[shrink.size() - 1].x, shrink[shrink.size() - 1].y, curPoint.x, curPoint.y);
            }

            shrink.emplace_back(Vector4F(curPoint.x, curPoint.y, nextPoint.x, nextPoint.y));
        }

        status.shrink = shrink;
        return status;
    }

    void KiriVoroCellPolygon2::ComputeStraightSkeleton(float lambda)
    {
        Vector<Vector4F> poly;
        for (size_t i = 0; i < mPolygonVertices2.size(); i++)
        {
            auto v1 = mPolygonVertices2[i];
            auto v2 = mPolygonVertices2[(i + 1) % mPolygonVertices2.size()];
            poly.emplace_back(Vector4F(v1.x, v1.y, v2.x, v2.y));
        }

        if (IsClockwise(poly))
        {
            Vector<Vector2F> rPolyVert(mPolygonVertices2);
            std::reverse(rPolyVert.begin(), rPolyVert.end());

            poly.clear();
            for (size_t i = 0; i < rPolyVert.size(); i++)
            {
                auto v1 = rPolyVert[i];
                auto v2 = rPolyVert[(i + 1) % rPolyVert.size()];
                poly.emplace_back(Vector4F(v1.x, v1.y, v2.x, v2.y));
            }
        }

        Vector<Vector4F> oPoly(poly);
        Vector2F point = Vector2F(0.f);
        auto cnt = 0;
        while (cnt < 100000)
        {
            auto status = ComputeShrink(poly, lambda);
            auto shrink = status.shrink;
            mShrinks.insert(mShrinks.end(), shrink.begin(), shrink.end());

            Vector<Vector4F> newPoly(shrink);

            if (status.intersection)
            {
                auto removePoints = std::remove_if(newPoly.begin(),
                                                   newPoly.end(),
                                                   [&point](const Vector4F &elem)
                                                   {
                                                       auto res = IsApproxVec2(Vector2F(elem.x, elem.y), Vector2F(elem.z, elem.w), 0.01f);

                                                       if (res.z == 0.f)
                                                       {
                                                           point = Vector2F(res.x, res.y);
                                                           return false;
                                                       }
                                                       else
                                                           return true;
                                                   });

                newPoly.erase(removePoints, newPoly.end());
            }

            if (newPoly.size() < poly.size())
            {
                for (size_t i = 0; i < oPoly.size(); i++)
                    mSkeletons.emplace_back(Vector4F(oPoly[i].x, oPoly[i].y, shrink[i].x, shrink[i].y));

                oPoly = newPoly;
            }

            if (newPoly.size() == 2)
                if (IsApproxVec4(newPoly[0], newPoly[1], 0.01f))
                    newPoly.pop_back();

            if (newPoly.size() <= 1)
            {
                if (newPoly.size() == 1)
                    mSkeletons.emplace_back(newPoly[0]);
                else
                    for (size_t i = 0; i < oPoly.size(); i++)
                        mSkeletons.emplace_back(Vector4F(oPoly[i].x, oPoly[i].y, point.x, point.y));

                break;
            }
            poly = newPoly;
            cnt++;
        }
    }

    Vector3F KiriVoroCellPolygon2::ComputeMICByStraightSkeleton()
    {
        auto maxCirVec = Vector2F(0.f);
        auto maxCirRad = Tiny<float>();
        if (!mSkeletons.empty())
        {
            for (size_t i = 0; i < mSkeletons.size(); i++)
            {
                auto v1 = Vector2F(mSkeletons[i].x, mSkeletons[i].y);
                auto v2 = Vector2F(mSkeletons[i].z, mSkeletons[i].w);

                if (Contains(v1))
                {
                    auto minDis = ComputeMinDisInPoly(v1);
                    if (minDis > maxCirRad)
                    {
                        maxCirRad = minDis;
                        maxCirVec = v1;
                    }
                }

                if (Contains(v2))
                {
                    auto minDis = ComputeMinDisInPoly(v2);
                    if (minDis > maxCirRad)
                    {
                        maxCirRad = minDis;
                        maxCirVec = v2;
                    }
                }
            }
        }
        return Vector3F(maxCirVec.x, maxCirVec.y, maxCirRad);
    }

    bool KiriVoroCellPolygon2::CheckBBox()
    {
        if (mBBox2.isEmpty())
        {
            if (!mPolygonVertices2.empty())
            {
                UpdateBBox();
                return true;
            }
            else
            {
                KIRI_LOG_ERROR("Contains:: No polygon data!!");
                return false;
            }
        }
        return true;
    }

    Vector2F KiriVoroCellPolygon2::GetRndInnerPoint()
    {
        if (!CheckBBox())
        {
            KIRI_LOG_ERROR("GetRndInnerPoint:: Get inner point failed!!");
            return Vector2F(0.f);
        }

        Vector2F inner;
        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<> dist(0.f, 1.f);
        do
        {
            inner = mBBox2.LowestPoint + Vector2F(dist(rndEngine) * mBBox2.width(), dist(rndEngine) * mBBox2.height());
        } while (!Contains(inner));

        return inner;
    }

    bool KiriVoroCellPolygon2::Contains(const Vector2F &v)
    {
        if (!CheckBBox())
            return false;

        if (!mBBox2.contains(v))
            return false;

        bool contains = false;
        for (size_t i = 0, j = mPolygonVertices2.size() - 1; i < mPolygonVertices2.size(); j = i++)
        {
            auto verti = mPolygonVertices2[i];
            auto vertj = mPolygonVertices2[j];
            if ((((verti.y <= v.y) && (v.y < vertj.y) || ((vertj.y <= v.y) && (v.y < verti.y))) && (v.x < (vertj.x - verti.x) * (v.y - verti.y) / (vertj.y - verti.y) + verti.x)))
                contains = !contains;
        }
        return contains;
    }

    float KiriVoroCellPolygon2::GetPolygonArea()
    {
        auto area = 0.f;
        if (!mPolygonVertices2.empty())
            for (size_t i = 0; i < GetLength(); i++)
                area += mPolygonVertices2[i].cross(mPolygonVertices2[(i + 1) % GetLength()]);

        return std::abs(area * 0.5f);
    }

    Vector2F KiriVoroCellPolygon2::GetPolygonCentroid()
    {
        auto centroid = Vector2F(0.f);
        auto length = GetLength();
        if (!mPolygonVertices2.empty())
        {
            for (size_t i = 0; i < length; i++)
                centroid += (mPolygonVertices2[i] + mPolygonVertices2[(i + 1) % length]) * (mPolygonVertices2[i].cross(mPolygonVertices2[(i + 1) % length]));

            centroid /= 6.f * GetPolygonArea();
        }

        return centroid;
    }

    float KiriVoroCellPolygon2::ComputeMinDisInPoly(const Vector2F &p)
    {
        auto minDis = Huge<float>();
        for (size_t i = 0; i < mPolygonVertices2.size(); i++)
            minDis = std::min(minDis, MinDis2LineSegment2(mPolygonVertices2[i], mPolygonVertices2[(i + 1) % mPolygonVertices2.size()], p));
        return minDis;
    }

    void KiriVoroCellPolygon2::UpdateBBox()
    {
        for (size_t i = 0; i < mPolygonVertices2.size(); i++)
            mBBox2.merge(Vector2F(mPolygonVertices2[i]));
    }

    void KiriVoroCellPolygon2::ComputeVoroSitesList()
    {
        mVoroSitesList->RemoveAll();
        for (size_t i = 0; i < mPolygonVertices2.size(); i++)
            mVoroSitesList->Push(Vector2F(mPolygonVertices2[mPolygonVertices2.size() - 1 - i]));
    }

    void KiriVoroCellPolygon2::PrintPolyVertices()
    {
        KIRI_LOG_DEBUG("----------Voronoi Cell Polygon Vertices INFO----------");

        for (size_t i = 0; i < mPolygonVertices2.size(); i++)
            KIRI_LOG_DEBUG("vertex2=({0},{1})", mPolygonVertices2[i].x, mPolygonVertices2[i].y);

        KIRI_LOG_DEBUG("------------------------------");
    }

    void KiriVoroCellPolygon2::PrintVoroSitesList()
    {
        KIRI_LOG_DEBUG("----------Voronoi Sites List INFO----------");

        mVoroSitesList->PrintVertexList();

        KIRI_LOG_DEBUG("------------------------------");
    }

    void KiriVoroCellPolygon2::Print()
    {
        KIRI_LOG_DEBUG("----------Voronoi Cell Polygon INFO----------");
        KIRI_LOG_DEBUG("Voro cell polygon size={0}", mPolygonVertices2.size());
        KIRI_LOG_DEBUG("Voro cell mVoroSitesList size={0}", mVoroSitesList->Size());

        // for (size_t i = 0; i < mPolygonVertices2.size(); i++)
        //     KIRI_LOG_DEBUG("vertex2=({0},{1})", mPolygonVertices2[i].x, mPolygonVertices2[i].y);

        mVoroSitesList->PrintVertexList();

        KIRI_LOG_DEBUG("------------------------------");
    }
}