/***
 * @Author: Xu.WANG
 * @Date: 2021-12-01 19:04:56
 * @LastEditTime: 2021-12-01 19:05:44
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_VERTEX2_H_
#define _HDV_VERTEX2_H_

#pragma once

#include <kiri2d/hdv_toolkit/primitives/vertex.h>

namespace HDV::Primitives
{
    class Vertex2 : public Vertex
    {
    public:
        explicit Vertex2() : Vertex(2) {}
        explicit Vertex2(int id) : Vertex(2, id) {}
        explicit Vertex2(float x, float y) : Vertex(2)
        {
            mPosition[0] = x;
            mPosition[1] = y;
        }
        explicit Vertex2(float x, float y, int id) : Vertex(2, id)
        {
            mPosition[0] = x;
            mPosition[1] = y;
        }
        virtual ~Vertex2() noexcept {}

        void Set(float x, float y)
        {
            mPosition[0] = x;
            mPosition[1] = y;
        }

        float Distance(float px, float py)
        {
            return std::sqrtf(SqrDistance(px, py));
        }

        float SqrDistance(float px, float py)
        {
            auto x = mPosition[0] - px;
            auto y = mPosition[1] - py;

            return x * x + y * y;
        }
    };
    typedef std::shared_ptr<Vertex2> Vertex2Ptr;
} // namespace HDV::Primitives

#endif /* _HDV_VERTEX2_H_ */