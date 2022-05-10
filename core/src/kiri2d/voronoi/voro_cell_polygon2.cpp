/***
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-10-07 02:16:06
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_cell_polygon2.cpp
 */

#include <kiri2d/voronoi/voro_cell_polygon2.h>
#include <kiri2d/voronoi/voro_util.h>
#include <random>
#include <kiri2d/straight_skeleton/sskel_slav.h>

#include <remove_at.hpp>
namespace KIRI
{

    bool KiriVoroCellPolygon2::isClockwise(const Vector<Vector4F> &poly)
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

    void KiriVoroCellPolygon2::computeSSkel1998Convex()
    {
        mSkeletons.clear();
        Vector<Vector4F> poly;
        for (size_t i = 0; i < mPolygonVertices2.size(); i++)
        {
            auto v1 = mPolygonVertices2[i];
            auto v2 = mPolygonVertices2[(i + 1) % mPolygonVertices2.size()];
            poly.emplace_back(Vector4F(v1.x, v1.y, v2.x, v2.y));
        }

        Vector<Vector2F> rPolyVert(mPolygonVertices2);
        if (isClockwise(poly))
            std::reverse(rPolyVert.begin(), rPolyVert.end());

        auto sskel_convex = std::make_shared<KIRI2D::SSKEL::SSkelSLAV>(rPolyVert);

        auto skeletons = sskel_convex->GetSkeletons();

        for (size_t i = 0; i < skeletons.size(); i++)
        {
            auto [intersect, sinks] = skeletons[i];
            for (size_t j = 0; j < sinks.size(); j++)
                mSkeletons.emplace_back(Vector4F(intersect.x, intersect.y, sinks[j].x, sinks[j].y));
        }
    }

    Vec_Vec3F KiriVoroCellPolygon2::ComputeAllCByStraightSkeleton()
    {
        // TODO remove same cir
        Vec_Vec3F all_circles;
        if (!mSkeletons.empty())
        {
            for (size_t i = 0; i < mSkeletons.size(); i++)
            {
                auto v1 = Vector2F(mSkeletons[i].x, mSkeletons[i].y);
                auto v2 = Vector2F(mSkeletons[i].z, mSkeletons[i].w);

                if (contains(v1))
                {
                    auto minDis = computeMinDisInPoly(v1);
                    all_circles.emplace_back(v1.x, v1.y, minDis);
                }

                if (contains(v2))
                {
                    auto minDis = computeMinDisInPoly(v2);
                    all_circles.emplace_back(v1.x, v1.y, minDis);
                }
            }
        }
        return all_circles;
    }

    Vec_Vec3F KiriVoroCellPolygon2::ComputeMICByStraightSkeletonTest()
    {
        Vec_Vec3F mic;
        Vector<int> removeMIC;
        if (!mSkeletons.empty())
        {
            for (size_t i = 0; i < mSkeletons.size(); i++)
            {
                auto v1 = Vector2F(mSkeletons[i].x, mSkeletons[i].y);

                if (contains(v1))
                {
                    auto minDis = computeMinDisInPoly(v1);
                    mic.emplace_back(Vector3F(v1.x, v1.y, minDis));
                }

                auto v2 = Vector2F(mSkeletons[i].z, mSkeletons[i].w);
                if (contains(v2))
                {
                    auto minDis = computeMinDisInPoly(v2);
                    mic.emplace_back(Vector3F(v2.x, v2.y, minDis));
                }
            }
        }

        if (mic.size() > 1)
        {
            for (size_t i = 0; i < mic.size() - 1; i++)
            {
                bool bRemoveContain = false;
                for (size_t j = i + 1; j < mic.size(); j++)
                {
                    auto dist_ij = (Vector2F(mic[i].x, mic[i].y) - Vector2F(mic[j].x, mic[j].y)).length();

                    if ((mic[j].z > (mic[i].z + dist_ij)) || (mic[i] == mic[j]))
                    {
                        bRemoveContain = true;
                        break;
                    }
                }

                if (bRemoveContain)
                    removeMIC.emplace_back(static_cast<int>(i));
            }

            // KIRI_LOG_DEBUG("o size={0}, re size={1}", mic.size(), removeMIC.size());
        }

        if (!removeMIC.empty())
        {
            remove_at<Vector3F>(mic, removeMIC);
        }

        // KIRI_LOG_DEBUG("current o size={0}", mic.size());

        return mic;
    }

    Vector3F KiriVoroCellPolygon2::computeMICByStraightSkeleton()
    {
        auto maxCirVec = Vector2F(0.f);
        auto maxCirRad = Tiny<float>();
        if (!mSkeletons.empty())
        {
            for (size_t i = 0; i < mSkeletons.size(); i++)
            {
                auto v1 = Vector2F(mSkeletons[i].x, mSkeletons[i].y);
                auto v2 = Vector2F(mSkeletons[i].z, mSkeletons[i].w);

                if (contains(v1))
                {
                    auto minDis = computeMinDisInPoly(v1);
                    if (minDis > maxCirRad)
                    {
                        maxCirRad = minDis;
                        maxCirVec = v1;
                    }
                }

                if (contains(v2))
                {
                    auto minDis = computeMinDisInPoly(v2);
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

    bool KiriVoroCellPolygon2::checkBBox()
    {
        if (mBBox2.isEmpty())
        {
            if (!mPolygonVertices2.empty())
            {
                updateBBox();
                return true;
            }
            else
            {
                KIRI_LOG_ERROR("contains:: No polygon data!!");
                return false;
            }
        }
        return true;
    }

    Vector2F KiriVoroCellPolygon2::rndInnerPoint()
    {
        if (!checkBBox())
        {
            KIRI_LOG_ERROR("rndInnerPoint:: Get inner point failed!!");
            return Vector2F(0.f);
        }

        Vector2F inner;
        std::random_device seed;
        std::default_random_engine engine(seed());
        std::uniform_real_distribution<> dist(0.f, 1.f);
        do
        {
            inner = mBBox2.LowestPoint + Vector2F(dist(engine) * mBBox2.width(), dist(engine) * mBBox2.height());
        } while (!contains(inner));

        return inner;
    }

    bool KiriVoroCellPolygon2::contains(const Vector2F &v)
    {
        if (!checkBBox())
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

            // if (((verti.y > v.y) != (vertj.y > v.y)) &&
            //     (v.x < (vertj.x - verti.x) * (v.y - verti.y) / (vertj.y - verti.y) + verti.x))
            //     contains = !contains;
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
        // auto centroid = Vector2F(0.f);
        // auto length = GetLength();
        // if (!mPolygonVertices2.empty())
        // {
        //     for (size_t i = 0; i < length; i++)
        //         centroid += (mPolygonVertices2[i] + mPolygonVertices2[(i + 1) % length]) * (mPolygonVertices2[i].cross(mPolygonVertices2[(i + 1) % length]));

        //     centroid /= 6.f * GetPolygonArea();
        // }
        Vector<Vector2F> tmpPolygonVertices(mPolygonVertices2);
        auto length = GetLength();

        auto first = tmpPolygonVertices[0], last = tmpPolygonVertices[tmpPolygonVertices.size() - 1];
        if (first.x != last.x || first.y != last.y)
            tmpPolygonVertices.emplace_back(first);

        auto twicearea = 0.f,
             x = 0.f, y = 0.f, f = 0.f;
        auto nPts = tmpPolygonVertices.size();
        Vector2F p1, p2;

        for (size_t i = 0, j = nPts - 1; i < nPts; j = i++)
        {
            p1 = tmpPolygonVertices[i];
            p2 = tmpPolygonVertices[j];
            f = p1.x * p2.y - p2.x * p1.y;
            twicearea += f;
            x += (p1.x + p2.x) * f;
            y += (p1.y + p2.y) * f;
        }
        f = twicearea * 3.f;

        return Vector2F(x / f, y / f);
    }

    float KiriVoroCellPolygon2::computeMinDisInPoly(const Vector2F &p)
    {
        auto minDis = Huge<float>();
        for (size_t i = 0; i < mPolygonVertices2.size(); i++)
            minDis = std::min(minDis, minDis2LineSegment2(mPolygonVertices2[i], mPolygonVertices2[(i + 1) % mPolygonVertices2.size()], p));
        return minDis;
    }

    void KiriVoroCellPolygon2::updateBBox()
    {
        for (size_t i = 0; i < mPolygonVertices2.size(); i++)
            mBBox2.merge(Vector2F(mPolygonVertices2[i]));
    }

    void KiriVoroCellPolygon2::ComputeVoroSitesList()
    {
        mVoroSitesList->removeAll();
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

        for (size_t i = 0; i < mPolygonVertices2.size(); i++)
            KIRI_LOG_DEBUG("vertex2=({0},{1}),size={2}", mPolygonVertices2[i].x, mPolygonVertices2[i].y, mPolygonVertices2.size());

        mVoroSitesList->PrintVertexList();

        KIRI_LOG_DEBUG("------------------------------");
    }
}