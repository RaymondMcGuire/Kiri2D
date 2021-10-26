/***
 * @Author: Xu.WANG
 * @Date: 2021-08-23 18:20:43
 * @LastEditTime: 2021-10-26 15:46:10
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_split_test.h
 */

#ifndef _KIRI_VORO_SPLIT_H_
#define _KIRI_VORO_SPLIT_H_

#pragma once

#include <kiri2d/voronoi/power_diagram.h>
#include <kiri2d/voronoi/voro_group_site.h>
namespace KIRI
{

    class KiriVoroSplit
    {
    public:
        explicit KiriVoroSplit()
        {
            mPowerDiagram = std::make_shared<KiriPowerDiagram>();
        }

        ~KiriVoroSplit() noexcept {}

        void AddSite(const KiriVoroGroupSitePtr &site) { mPowerDiagram->AddVoroSite(site); }
        void SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary) { mPowerDiagram->SetBoundaryPolygon2(boundary); }

        void Init();
        void ComputeGroup();
        Vector<Vector4F> GetMICBySSkel();
        const Vector<KiriVoroSitePtr> &GetVoroSites() const { return mPowerDiagram->GetVoroSites(); }

    private:
        KiriPowerDiagramPtr mPowerDiagram;
        UInt mGroupCounter = 0;

        Vector<Vector4F> mMIC;

        Vec_Vec3F GetMIC(const KiriVoroSitePtr &site);
    };

    typedef SharedPtr<KiriVoroSplit> KiriVoroSplitPtr;
}
#endif