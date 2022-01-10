/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2022-01-10 23:49:49
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
        explicit Vertex2(double x, double y) : Vertex(2)
        {
            mPosition[0] = x;
            mPosition[1] = y;
        }
        explicit Vertex2(double x, double y, int id) : Vertex(2, id)
        {
            mPosition[0] = x;
            mPosition[1] = y;
        }
        virtual ~Vertex2() noexcept {}

        void Set(double x, double y)
        {
            mPosition[0] = x;
            mPosition[1] = y;
        }

        double Distance(std::shared_ptr<Vertex2> v)
        {
            return std::sqrtf(SqrDistance(v->X(), v->Y()));
        }

        double Distance(double px, double py)
        {
            return std::sqrtf(SqrDistance(px, py));
        }

        double SqrDistance(double px, double py)
        {
            auto x = mPosition[0] - px;
            auto y = mPosition[1] - py;

            return x * x + y * y;
        }

        double X() const { return mPosition[0]; }
        double Y() const { return mPosition[1]; }
    };
    typedef std::shared_ptr<Vertex2> Vertex2Ptr;
} // namespace HDV::Primitives

#endif /* _HDV_VERTEX2_H_ */