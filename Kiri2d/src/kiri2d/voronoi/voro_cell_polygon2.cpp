/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-06-08 16:23:24
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_cell_polygon2.cpp
 */

#include <kiri2d/voronoi/voro_cell_polygon2.h>
#include <kiri2d/voronoi/voro_util.h>
#include <random>

namespace KIRI
{

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