/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2021-12-29 14:58:29
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
        explicit Vertex4(double x, double y, double z, double w) : Vertex(4)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
            mPosition[3] = w;
        }
        explicit Vertex4(double x, double y, double z, double w, int id) : Vertex(4, id)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
            mPosition[3] = w;
        }
        virtual ~Vertex4() {}

        void Set(double x, double y, double z, double w)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
            mPosition[3] = w;
        }

        double Distance(double px, double py, double pz, double pw)
        {
            return std::sqrtf(SqrDistance(px, py, pz, pw));
        }

        double SqrDistance(double px, double py, double pz, double pw)
        {
            auto x = mPosition[0] - px;
            auto y = mPosition[1] - py;
            auto z = mPosition[2] - pz;
            auto w = mPosition[3] - pw;

            return x * x + y * y + z * z + w * w;
        }

        double X() const { return mPosition[0]; }
        double Y() const { return mPosition[1]; }
        double Z() const { return mPosition[2]; }
        double W() const { return mPosition[3]; }
    };

    typedef std::shared_ptr<Vertex4> Vertex4Ptr;
} // namespace HDV::Primitives

#endif /* _HDV_Vertex4_H_ */