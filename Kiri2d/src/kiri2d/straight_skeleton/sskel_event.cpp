/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 11:03:44
 * @LastEditTime: 2021-10-04 20:19:32
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\straight_skeleton\sskel_event.cpp
 */

#include <kiri2d/straight_skeleton/sskel_event.h>
namespace KIRI2D::SSKEL
{
    void SSkelEdgeEvent::Print()
    {
        KIRI_LOG_DEBUG("Edge Event: distance={0}, intersect=({1},{2}), between VertA={3},{4} and VertB={5},{6}",
                       mDistance, mIntersect.x, mIntersect.y,
                       mVertA->GetPoint().x, mVertA->GetPoint().y,
                       mVertB->GetPoint().x, mVertB->GetPoint().y);
    }

    void SSkelSplitEvent::Print()
    {
        KIRI_LOG_DEBUG("Split Event: distance={0}, intersect=({1},{2}); Vert={3},{4} ; Opposite Edge=({5},{6})---({7},{8})",
                       mDistance, mIntersect.x, mIntersect.y,
                       mVert->GetPoint().x, mVert->GetPoint().y,
                       mOppositeEdge.x, mOppositeEdge.y,
                       mOppositeEdge.z, mOppositeEdge.w);
    }

}