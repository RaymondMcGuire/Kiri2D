/***
 * @Author: Xu.WANG
 * @Date: 2021-12-01 19:36:00
 * @LastEditTime: 2021-12-05 20:48:09
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_VERTEX3_H_
#define _HDV_VERTEX3_H_

#pragma once

#include <kiri2d/hdv_toolkit/primitives/vertex.h>

namespace HDV::Primitives
{
    class Vertex3 : public Vertex
    {
    public:
        explicit Vertex3() : Vertex(3) {}
        explicit Vertex3(int id) : Vertex(3, id) {}
        explicit Vertex3(float x, float y, float z) : Vertex(3)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
        }
        explicit Vertex3(float x, float y, float z, int id) : Vertex(3, id)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
        }
        virtual ~Vertex3() noexcept {}

        void Set(float x, float y, float z)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
        }

        float Distance(float px, float py, float pz)
        {
            return std::sqrtf(SqrDistance(px, py, pz));
        }

        float SqrDistance(float px, float py, float pz)
        {
            auto x = mPosition[0] - px;
            auto y = mPosition[1] - py;
            auto z = mPosition[2] - pz;

            return x * x + y * y + z * z;
        }
    };
    typedef std::shared_ptr<Vertex3> Vertex3Ptr;
} // namespace HDV::Primitives

#endif /* _HDV_Vertex3_H_ */