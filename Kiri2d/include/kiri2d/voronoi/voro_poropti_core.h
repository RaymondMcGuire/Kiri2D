/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-07-27 18:48:32
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_poropti_core.h
 */

#ifndef _KIRI_VORO_PORO_OPTI_CORE_H_
#define _KIRI_VORO_PORO_OPTI_CORE_H_

#pragma once

#include <kiri2d/voronoi/power_diagram.h>

namespace KIRI
{

    class KiriVoroPoroOptiCore
    {
    public:
        explicit KiriVoroPoroOptiCore(
            float targetPorosity = 0.f)
            : mTargetPorosity(targetPorosity),
              mCurrentPorosity(0.f)
        {
            Reset();
            mPowerDiagram = std::make_shared<KiriPowerDiagram>();
        }

        ~KiriVoroPoroOptiCore() noexcept {}

        void ComputeLloyd(UInt num);
        float ComputeIterate();

        void AddSite(const KiriVoroSitePtr &site) { mPowerDiagram->AddVoroSite(site); }
        void SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary);
        void SetMaxIterationNum(UInt num) { mMaxIterationNum = num; }
        void SetErrorThreshold(float error) { mErrorThreshold = error; }

        const Vector<KiriVoroSitePtr> &GetSites() const { return mPowerDiagram->GetVoroSites(); }

        constexpr UInt GetMaxIterationNum() const { return mMaxIterationNum; }
        constexpr float GetErrorThreshold() const { return mErrorThreshold; }
        constexpr float GetMiniumPorosity() const { return mCurrentPorosity; }

        void Init();

        void Reset();

        float ComputeMiniumPorosity();
        Vector<Vector4F> GetCellSSkel();
        Vector<Vector4F> GetMICBySSkel();

        Vector3F ComputeMaxInscribedCircle() const { return mPowerDiagram->ComputeMaxInscribedCircle(); };

        void RemoveVoroSitesByIndexArray(Vector<UInt> indexs) { mPowerDiagram->RemoveVoroSitesByIndexArray(indexs); }

    private:
        void ComputeVoroSiteWeightError();
        void ComputeBoundaryPolygonArea();

        float GetGlobalAvgDistance();
        float GetGlobalAreaError();
        float Iterate();

        void CorrectVoroSitePos();
        void CorrectWeights();
        void AdaptPositionsWeights();
        void AdaptWeights();
        void DynamicAddSites();

        KiriPowerDiagramPtr mPowerDiagram;

        float mCompleteArea;
        float mCurGlobalWeightError;
        Vector<float> mVoroSitesWeightError, mVoroSitesWeightAbsError;

        Vector<float> mGlobalErrorArray;

        float mTargetPorosity;
        float mCurrentPorosity;
        float mErrorThreshold;
        UInt mCurIteration, mMaxIterationNum;
    };

    typedef SharedPtr<KiriVoroPoroOptiCore> KiriVoroPoroOptiCorePtr;
}
#endif