/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-15 13:27:53
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_poropti_node.h
 */

#ifndef _KIRI_VORO_PORO_OPTI_NODE_H_
#define _KIRI_VORO_PORO_OPTI_NODE_H_

#pragma once

#include <kiri2d/voronoi/voro_poropti_core.h>

namespace KIRI
{

    class KiriVoroPoroOptiNode
    {
    public:
        explicit KiriVoroPoroOptiNode(Vector2F pos, float radius)
        {
            mCore = std::make_shared<KiriVoroPoroOptiCore>();
            mSite = std::make_shared<KiriVoroSite>(pos.x, pos.y);
            mSite->SetRadius(radius);
        }

        ~KiriVoroPoroOptiNode() noexcept {}

        void InitCore();
        const KiriVoroSitePtr &GetSite() const { return mSite; }
        void SetSite(const KiriVoroSitePtr &site) { mSite = site; }

        const KiriVoroPoroOptiCorePtr &GetCore() const { return mCore; }

        void ComputeIterate();

        Vector4F ComputeMaxInscribedCircle() const { return Vector4F(mCore->ComputeMaxInscribedCircle(), mSite->GetRadius()); };

    private:
        KiriVoroSitePtr mSite;
        KiriVoroPoroOptiCorePtr mCore;

        void InitChildSites();
    };

    typedef SharedPtr<KiriVoroPoroOptiNode> KiriVoroPoroOptiNodePtr;
}
#endif