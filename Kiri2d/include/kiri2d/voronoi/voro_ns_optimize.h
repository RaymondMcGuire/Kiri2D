/***
 * @Author: Xu.WANG
 * @Date: 2021-11-17 12:15:22
 * @LastEditTime: 2021-11-17 12:16:09
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_ns_optimize.h
 */

#ifndef _KIRI_VORO_NS_OPTIMIZE_H_
#define _KIRI_VORO_NS_OPTIMIZE_H_

#pragma once

#include <kiri2d/voronoi/power_diagram.h>

namespace KIRI
{

    class KiriVoroNSOptimize
    {
    public:
        explicit KiriVoroNSOptimize()
        {
            Reset();
            mPowerDiagram = std::make_shared<KiriPowerDiagram>();
        }

        ~KiriVoroNSOptimize() noexcept {}

        void ComputeLloyd(UInt num);
        float ComputeIterate();

        void AddSite(const KiriVoroGroupSitePtr &site)
        {
            mPowerDiagram->AddVoroSite(site);
            mMaxGroupNum++;
        }
        void SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary);
        void SetMaxIterationNum(UInt num) { mMaxIterationNum = num; }
        void SetErrorThreshold(float error) { mErrorThreshold = error; }

        const Vector<KiriVoroSitePtr> &GetVoroSites() const { return mPowerDiagram->GetVoroSites(); }

        constexpr UInt GetMaxIterationNum() const { return mMaxIterationNum; }
        constexpr float GetErrorThreshold() const { return mErrorThreshold; }
        constexpr float GetCurGlobalPorosity() const { return mCurGlobalPorosity; }

        void Init();

        void Reset();

        void RemoveVoroSitesByIndexArray(Vector<UInt> indexs) { mPowerDiagram->RemoveVoroSitesByIndexArray(indexs); }

        void SetMaxiumVorosite(UInt num) { mMaxiumNum = num; }

    private:
        void ComputeVoroSiteWeightError();
        void ComputeBoundaryPolygonArea();

        float GetGlobalAvgDistance();
        float GetGlobalAreaError();
        void Iterate();

        void CorrectVoroSitePos();
        void CorrectWeights();
        void AdaptPositionsWeights();
        void AdaptWeights();

        void RemoveNoiseVoroSites();

        void SplitEventProcess();

        KiriPowerDiagramPtr mPowerDiagram;

        float mCompleteArea;
        float mCurGlobalWeightError, mCurGlobalPorosity = 0.f;
        Vector<float> mVoroSitesWeightError, mVoroSitesWeightAbsError;

        Vector<float> mGlobalErrorArray, mGlobalPorosityArray;

        float mErrorThreshold;
        UInt mMaxiumNum = 1000;
        UInt mCurIteration, mMaxIterationNum;

        UInt mMaxGroupNum = 0;
    };

    typedef SharedPtr<KiriVoroNSOptimize> KiriVoroNSOptimizePtr;
}
#endif