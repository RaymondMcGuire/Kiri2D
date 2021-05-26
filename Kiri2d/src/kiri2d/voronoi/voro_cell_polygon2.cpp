/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-05-26 17:54:42
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_cell_polygon2.cpp
 */

#include <kiri2d/voronoi/voro_cell_polygon2.h>
namespace KIRI
{
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