/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-11 11:05:41
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
            mSite = std::make_shared<KiriVoroSite>(pos, radius);
            //mSite = std::make_shared<KiriVoroSite>(pos.x, pos.y, radius * radius + MEpsilon<float>(), radius);
        }

        ~KiriVoroPoroOptiNode() noexcept {}

        void InitCore();
        const KiriVoroSitePtr &GetSite() const { return mSite; }
        void SetSite(const KiriVoroSitePtr &site) { mSite = site; }

        const KiriVoroPoroOptiCorePtr &GetCore() const { return mCore; }

        void ComputeIterate();

        Vector4F ComputeMaxInscribedCircle() const { return Vector4F(mCore->ComputeMaxInscribedCircle(), mSite->GetPercentage()); };

    private:
        KiriVoroSitePtr mSite;
        KiriVoroPoroOptiCorePtr mCore;

        void InitChildSites();
    };

    typedef SharedPtr<KiriVoroPoroOptiNode> KiriVoroPoroOptiNodePtr;
}
#endif