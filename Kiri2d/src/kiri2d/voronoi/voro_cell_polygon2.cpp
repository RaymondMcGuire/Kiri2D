/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-25 02:06:00
 * @LastEditTime: 2021-05-26 11:54:10
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\voronoi\voro_cell_polygon2.cpp
 */

#include <kiri2d/voronoi/voro_cell_polygon2.h>
namespace KIRI
{
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