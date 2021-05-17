/*** 
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:49:01
 * @LastEditTime: 2021-05-18 01:20:11
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\geo\convex_hull3.h
 */

#ifndef _KIRI_CONVEXHULL3_H_
#define _KIRI_CONVEXHULL3_H_

#pragma once

#include <kiri2d/data/vertex3.h>
#include <kiri2d/data/face3.h>

namespace KIRI2D
{
    class KiriConvexHull3
    {
    public:
        KiriConvexHull3()
        {
        }

        ~KiriConvexHull3()
        {
        }

        void AddVertex(Vector3F v3);
        void AddFacet(KiriFace3 &f3);

        void ComputeConvexHull();
        void PrintVertexInfo();
        void PrintCurFacetsInfo();

    private:
        Vector<KiriVertex3> mVertices;
        Vector<KiriEdge3> mEdges;
        Vector<KiriFace3> mCurFacets;
        Vector<KiriFace3> mVisFacets;
        Vector<KiriFace3> mCreatedFacets;

        void BuildTetrahedron();
    };
    typedef SharedPtr<KiriConvexHull3> KiriConvexHull3Ptr;
} // namespace KIRI

#endif /* _KIRI_CONVEXHULL3_H_ */