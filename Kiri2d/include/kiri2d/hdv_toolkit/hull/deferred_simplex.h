/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 17:26:15
 * @LastEditTime: 2021-12-02 17:27:49
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_DEFERRED_SIMPLEX_H_
#define _HDV_DEFERRED_SIMPLEX_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/simplex_wrap.h>
namespace HDV::Hull
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class DeferredSimplex
    {
    public:
        explicit DeferredSimplex() {}

        virtual ~DeferredSimplex() noexcept {}

        std::shared_ptr<SimplexWrap<VERTEX>> Face;

        std::shared_ptr<SimplexWrap<VERTEX>> Pivot;

        std::shared_ptr<SimplexWrap<VERTEX>> OldFace;

        int FaceIndex;

        int PivotIndex;
    };

} // namespace HDV::Hull

#endif /* _HDV_DEFERRED_SIMPLEX_H_ */