/***
 * @Author: Xu.WANG
 * @Date: 2021-12-01 19:36:02
 * @LastEditTime: 2021-12-01 19:36:26
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_Vertex4_H_
#define _HDV_Vertex4_H_

#pragma once

#include <kiri2d/hdv_toolkit/primitives/vertex.h>

namespace HDV::Primitives
{
    class Vertex4 : public Vertex
    {
    public:
        explicit Vertex4() : Vertex(4) {}
        explicit Vertex4(int id) : Vertex(4, id) {}
        explicit Vertex4(float x, float y, float z, float w) : Vertex(4)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
            mPosition[3] = w;
        }
        explicit Vertex4(float x, float y, float z, float w, int id) : Vertex(4, id)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
            mPosition[3] = w;
        }
        virtual ~Vertex4() noexcept {}

        void Set(float x, float y, float z, float w)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
            mPosition[3] = w;
        }

        float Distance(float px, float py, float pz, float pw)
        {
            return std::sqrtf(SqrDistance(px, py, pz, pw));
        }

        float SqrDistance(float px, float py, float pz, float pw)
        {
            auto x = mPosition[0] - px;
            auto y = mPosition[1] - py;
            auto z = mPosition[2] - pz;
            auto w = mPosition[3] - pw;

            return x * x + y * y + z * z + w * w;
        }
    };
} // namespace HDV::Primitives

#endif /* _HDV_Vertex4_H_ */