/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:33:21
 * @LastEditTime: 2021-05-18 01:25:00
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\data\face3.cpp
 */

#include <kiri2d/data/face3.h>

namespace KIRI2D
{
    void KiriFace3::OrientFace(KiriVertex3 orient)
    {
        if (!CheckFaceDir(orient.GetValue()))
            ReverseFaceDir();
    }

    void KiriFace3::PrintFaceInfo()
    {
        KIRI_LOG_DEBUG("----------FACET INFO----------");
        KIRI_LOG_DEBUG("face info: v1=({0},{1},{2}),v2=({3},{4},{5}),v3=({6},{7},{8}),mNormal=({9},{10},{11})",
                       mVertices[0].GetValue().x, mVertices[0].GetValue().y, mVertices[0].GetValue().z,
                       mVertices[1].GetValue().x, mVertices[1].GetValue().y, mVertices[1].GetValue().z,
                       mVertices[2].GetValue().x, mVertices[2].GetValue().y, mVertices[2].GetValue().z,
                       mNormal.x, mNormal.y, mNormal.z);

        for (size_t i = 0; i < 3; i++)
            mEdges[i]->PrintEdgeInfo();

        KIRI_LOG_DEBUG("------------------------------");
    }

    void KiriFace3::ReverseFaceDir()
    {
        auto tmp = mVertices[1];
        mVertices[1] = mVertices[2];
        mVertices[2] = tmp;
        mNormal *= -1.f;
    }

    void KiriFace3::CreateEdges()
    {
        mEdges[0] = std::make_shared<KiriEdge3>(mIdx * 3 + 1, mVertices[0], mVertices[1]);
        mEdges[1] = std::make_shared<KiriEdge3>(mIdx * 3 + 2, mVertices[1], mVertices[2]);
        mEdges[2] = std::make_shared<KiriEdge3>(mIdx * 3 + 3, mVertices[2], mVertices[0]);

        mEdges[0]->SetNextEdge(mEdges[1]);
        mEdges[1]->SetNextEdge(mEdges[2]);
        mEdges[2]->SetNextEdge(mEdges[0]);

        mEdges[0]->SetPrevEdge(mEdges[2]);
        mEdges[1]->SetPrevEdge(mEdges[0]);
        mEdges[2]->SetPrevEdge(mEdges[1]);
    }

    const KiriEdge3Ptr &KiriFace3::GetEdge(KiriVertex3 a, KiriVertex3 b)
    {
        for (size_t i = 0; i < 3; i++)
        {
            if (mEdges[i]->IsEqual(a, b))
                return mEdges[i];
        }
        return NULL;
    }

    void KiriFace3::LinkFace(KiriFace3 f, KiriVertex3 a, KiriVertex3 b)
    {
        auto twin = f.GetEdge(a, b);
        if (twin == NULL)
        {
            KIRI_LOG_ERROR("LinkFace ERROR: Twin edge is not exist, cannot connect edges!");
            return;
        }
        auto cur_edge = this->GetEdge(a, b);
        twin->SetTwinEdge(cur_edge);
        cur_edge->SetTwinEdge(twin);
    }
}