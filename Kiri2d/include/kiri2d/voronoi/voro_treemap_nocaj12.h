/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-05-30 00:42:32
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_treemap_nocaj12.h
 */

#ifndef _KIRI_VORO_TREEMAP_NOCAJ12_H_
#define _KIRI_VORO_TREEMAP_NOCAJ12_H_

#pragma once

#include <kiri2d/voronoi/power_diagram.h>

namespace KIRI
{

    class KiriVoroTreemapNocaj12
    {
    public:
        explicit KiriVoroTreemapNocaj12::KiriVoroTreemapNocaj12()
        {
            Reset();
            mPowerDiagram = std::make_shared<KiriPowerDiagram>();
        }

        ~KiriVoroTreemapNocaj12() noexcept {}

        void Compute();
        void ComputeIterate();

        void AddSite(const KiriVoroSitePtr &site) { mPowerDiagram->AddVoroSite(site); }
        void SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary);
        void SetMaxIterationNum(UInt num) { mMaxIterationNum = num; }
        void SetErrorThreshold(float error) { mErrorThreshold = error; }

        const Vector<KiriVoroSitePtr> &GetSites() const { return mPowerDiagram->GetVoroSites(); }

        constexpr UInt GetMaxIterationNum() const { return mMaxIterationNum; }
        constexpr float GetErrorThreshold() const { return mErrorThreshold; }

        void Init();

        void Reset();

    private:
        void ComputeBoundaryPolygonArea();
        void ReComputePercentage();

        float GetGlobalAvgDistance();
        float GetGlobalAreaError();
        float GetMaxAreaError();

        void Iterate();

        void CorrectWeights();
        void AdaptPositionsWeights();
        void AdaptWeights();

        KiriPowerDiagramPtr mPowerDiagram;

        float mCompleteArea;
        float mCurGlobalAreaError, mCurMaxAreaError, mErrorThreshold;
        UInt mCurIteration, mMaxIterationNum;
    };

    typedef SharedPtr<KiriVoroTreemapNocaj12> KiriVoroTreemapNocaj12Ptr;
}
#endif