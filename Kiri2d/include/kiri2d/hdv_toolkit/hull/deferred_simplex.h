/***
 * @Author: Xu.WANG
 * @Date: 2021-12-09 00:10:54
 * @LastEditTime: 2021-12-22 18:27:04
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_DEFERRED_SIMPLEX_H_
#define _HDV_DEFERRED_SIMPLEX_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/simplex_wrap.h>
namespace HDV::Hull
{
    class DeferredSimplex
    {
    public:
        explicit DeferredSimplex() {}

        virtual ~DeferredSimplex() {}

        void Clear()
        {
            Face = nullptr;
            Pivot = nullptr;
            OldFace = nullptr;
        }

        std::shared_ptr<SimplexWrap> Face;

        std::shared_ptr<SimplexWrap> Pivot;

        std::shared_ptr<SimplexWrap> OldFace;

        int FaceIndex;

        int PivotIndex;
    };

} // namespace HDV::Hull

#endif /* _HDV_DEFERRED_SIMPLEX_H_ */