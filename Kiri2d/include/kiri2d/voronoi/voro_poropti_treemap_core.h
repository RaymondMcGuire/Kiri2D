/***
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-23 17:13:39
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_poropti_treemap_core.h
 */

#ifndef _KIRI_VORO_PORO_OPTI_TREEMAP_CORE_H_
#define _KIRI_VORO_PORO_OPTI_TREEMAP_CORE_H_

#pragma once

#include <kiri2d/voronoi/power_diagram.h>

namespace KIRI
{

    class KiriVoroPoroOptiTreeMapCore
    {
    public:
        explicit KiriVoroPoroOptiTreeMapCore()
        {
            reset();
            mPowerDiagram = std::make_shared<KiriPowerDiagram>();
        }

        ~KiriVoroPoroOptiTreeMapCore() {}

        float ComputeIterate();

        void AddSite(const KiriVoroSitePtr &site) { mPowerDiagram->AddVoroSite(site); }
        void SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary);
        void SetMaxIterationNum(UInt num) { mMaxIterationNum = num; }
        void SetErrorThreshold(float error) { mErrorThreshold = error; }

        const Vector<KiriVoroSitePtr> &GetSites() const { return mPowerDiagram->GetVoroSites(); }

        constexpr UInt GetMaxIterationNum() const { return mMaxIterationNum; }
        constexpr float GetErrorThreshold() const { return mErrorThreshold; }

        void Init();

        void reset();

        Vector3F ComputeMaxInscribedCircle() const { return mPowerDiagram->ComputeMaxInscribedCircle(); };

        void RemoveVoroSitesByIndexArray(Vector<UInt> indexs) { mPowerDiagram->RemoveVoroSitesByIndexArray(indexs); }

    private:
        void ReComputePercentage();

        void ComputeVoroSiteWeightError();
        void ComputeBoundaryPolygonArea();

        float GetGlobalAvgDistance();
        float GetGlobalAreaError();
        float Iterate();

        void CorrectVoroSitePos();
        void CorrectWeights();
        void AdaptPositionsWeights();
        void AdaptWeights();

        KiriPowerDiagramPtr mPowerDiagram;

        float mCompleteArea;
        float mCurGlobalWeightError;
        Vector<float> mVoroSitesWeightError, mVoroSitesWeightAbsError;

        Vector<float> mGlobalErrorArray;

        float mErrorThreshold;
        UInt mCurIteration, mMaxIterationNum;
    };

    typedef SharedPtr<KiriVoroPoroOptiTreeMapCore> KiriVoroPoroOptiTreeMapCorePtr;
}
#endif