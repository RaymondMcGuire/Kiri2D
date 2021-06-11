/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-10 22:46:24
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
        explicit KiriVoroPoroOptiCore()
        {
            Reset();
            mPowerDiagram = std::make_shared<KiriPowerDiagram>();
        }

        ~KiriVoroPoroOptiCore() noexcept {}

        void Compute();
        void ComputeDiagram();
        void ComputeLloyd(UInt num);
        float ComputeIterate();

        void AddSite(const KiriVoroSitePtr &site) { mPowerDiagram->AddVoroSite(site); }
        void SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary);
        void SetMaxIterationNum(UInt num) { mMaxIterationNum = num; }
        void SetErrorThreshold(float error) { mErrorThreshold = error; }

        const Vector<KiriVoroSitePtr> &GetSites() const { return mPowerDiagram->GetVoroSites(); }

        constexpr UInt GetMaxIterationNum() const { return mMaxIterationNum; }
        constexpr float GetErrorThreshold() const { return mErrorThreshold; }

        void Init();

        void Reset();

        Vector3F ComputeMaxInscribedCircle() const { return mPowerDiagram->ComputeMaxInscribedCircle(); };

    private:
        void ComputeVoroSiteMovement();
        void ComputeVoroSiteWeightError();

        void ComputeBoundaryPolygonArea();

        float GetGlobalAvgDistance();

        float Iterate();

        void CorrectVoroSitePos();
        void CorrectWeights();
        void AdaptPositionsWeights();
        void AdaptWeights();

        KiriPowerDiagramPtr mPowerDiagram;

        float mCurGlobalWeightError;
        Vector<float> mVoroSitesWeightError, mVoroSitesWeightAbsError;
        Vector<Vector2F> mVoroSitesMovemnet;

        float mErrorThreshold;
        UInt mCurIteration, mMaxIterationNum;
    };

    typedef SharedPtr<KiriVoroPoroOptiCore> KiriVoroPoroOptiCorePtr;
}
#endif