/***
 * @Author: Xu.WANG
 * @Date: 2021-11-18 16:03:39
 * @LastEditTime: 2021-11-24 12:32:46
 * @LastEditors: Xu.WANG
 * @Description:
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
            reset();
            mPowerDiagram = std::make_shared<KiriPowerDiagram>();
        }

        ~KiriVoroNSOptimize() {}

        void ComputeLloyd(UInt num);
        float ComputeIterate();

        void addSite(const KiriVoroGroupSitePtr &site)
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

        void init();

        void reset();

        void removeVoroSitesByIndexArray(Vector<UInt> indexs) { mPowerDiagram->removeVoroSitesByIndexArray(indexs); }

        void SetMaxiumVorosite(UInt num) { mMaxiumNum = num; }

        Vector<KiriVoroCellPolygon2Ptr> GetFrozenPolygon();

    private:
        void ComputeVoroSiteWeightError();
        void ComputeBoundaryPolygonArea();

        float globalAvgDistance();
        float globalAreaError();
        void Iterate();

        void ComputeVoroSitePosConstrains();
        void CorrectVoroSitePos();

        void adaptPositionsWeights();
        void adaptWeights();

        void removeNoiseVoroSites();

        void SplitEventProcess();

        KiriPowerDiagramPtr mPowerDiagram;

        float mCompleteArea;
        float mCurGlobalWeightError, mCurGlobalPosConstainWeightError, mCurGlobalPorosity = 0.f;
        Vector<float> mVoroSitesWeightError, mVoroSitesWeightAbsError;
        Vector<float> mVoroPosConstainWeightError, mVoroPosConstainWeightAbsError;

        Vector<float> mGlobalErrorArray, mGlobalPorosityArray;

        Vector<KiriVoroCellPolygon2Ptr> mUnionPolygonArray;

        float mErrorThreshold;
        UInt mMaxiumNum = 1000;
        UInt mCurIteration, mMaxIterationNum;

        UInt mMaxGroupNum = 0;
    };

    typedef SharedPtr<KiriVoroNSOptimize> KiriVoroNSOptimizePtr;
}
#endif