/***
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-02 20:00:37
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_treemap_core.h
 */

#ifndef _KIRI_VORO_TREEMAP_CORE_H_
#define _KIRI_VORO_TREEMAP_CORE_H_

#pragma once

#include <kiri2d/voronoi/power_diagram.h>

namespace KIRI
{

    class KiriVoroTreeMapCore
    {
    public:
        explicit KiriVoroTreeMapCore()
        {
            reset();
            mPowerDiagram = std::make_shared<KiriPowerDiagram>();
        }

        ~KiriVoroTreeMapCore() {}

        void compute();
        void ComputeIterate();

        void addSite(const KiriVoroSitePtr &site) { mPowerDiagram->AddVoroSite(site); }
        void SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary);
        void SetMaxIterationNum(UInt num) { mMaxIterationNum = num; }
        void SetErrorThreshold(float error) { mErrorThreshold = error; }

        const Vector<KiriVoroSitePtr> &sites() const { return mPowerDiagram->GetVoroSites(); }

        constexpr UInt GetMaxIterationNum() const { return mMaxIterationNum; }
        constexpr float GetErrorThreshold() const { return mErrorThreshold; }

        void init();

        void reset();

    private:
        void ComputeBoundaryPolygonArea();
        void ReComputePercentage();

        float globalAvgDistance();
        float globalAreaError();
        float GetMaxAreaError();

        void Iterate();

        void CorrectVoroSitePos();
        void CorrectWeights();
        void adaptPositionsWeights();
        void adaptWeights();

        KiriPowerDiagramPtr mPowerDiagram;

        float mCompleteArea;
        float mCurGlobalAreaError, mCurMaxAreaError, mErrorThreshold;
        UInt mCurIteration, mMaxIterationNum;
    };

    typedef SharedPtr<KiriVoroTreeMapCore> KiriVoroTreeMapCorePtr;
}
#endif