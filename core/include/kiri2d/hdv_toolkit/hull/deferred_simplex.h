/***
 * @Author: Xu.WANG
 * @Date: 2022-05-02 13:22:27
 * @LastEditTime: 2022-05-03 17:37:13
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_DEFERRED_SIMPLEX_H_
#define _HDV_DEFERRED_SIMPLEX_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/simplex_node.h>
namespace HDV::Hull
{
    class DeferredSimplex
    {
    public:
        explicit DeferredSimplex()
        {
        }

        virtual ~DeferredSimplex()
        {
        }

        void clear()
        {
            Face = nullptr;
            Pivot = nullptr;
            OldFace = nullptr;
        }

        std::shared_ptr<SimplexNode> Face;

        std::shared_ptr<SimplexNode> Pivot;

        std::shared_ptr<SimplexNode> OldFace;

        int FaceIndex;

        int PivotIndex;
    };

} // namespace HDV::Hull

#endif /* _HDV_DEFERRED_SIMPLEX_H_ */