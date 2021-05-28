/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-05-28 17:01:57
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

        void SetMaxIterationNum(UInt num) { mMaxIterationNum = num; }
        void SetErrorThreshold(float error) { mErrorThreshold = error; }

        constexpr UInt GetMaxIterationNum() const { return mMaxIterationNum; }
        constexpr float GetErrorThreshold() const { return mErrorThreshold; }

    private:
        void Reset();

        float GetGlobalAvgDistance();
        float GetGlobalAreaError();
        float GetMaxAreaError();

        void SetBoundaryPolygon2(const KiriVoroCellPolygon2Ptr &boundary);

        void Iterate();

        void ComputeBoundaryPolygonArea();
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