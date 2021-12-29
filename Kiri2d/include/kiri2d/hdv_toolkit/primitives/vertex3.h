/*** 
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2021-12-29 14:58:25
 * @LastEditors: Xu.WANG
 * @Description: 
 */
/***
 * @Author: Xu.WANG
 * @Date: 2021-12-01 19:36:00
 * @LastEditTime: 2021-12-08 16:10:57
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
        explicit Vertex3(double x, double y, double z) : Vertex(3)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
        }
        explicit Vertex3(double x, double y, double z, int id) : Vertex(3, id)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
        }
        virtual ~Vertex3() noexcept {}

        void Set(double x, double y, double z)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
        }

        double Distance(double px, double py, double pz)
        {
            return std::sqrtf(SqrDistance(px, py, pz));
        }

        double SqrDistance(double px, double py, double pz)
        {
            auto x = mPosition[0] - px;
            auto y = mPosition[1] - py;
            auto z = mPosition[2] - pz;

            return x * x + y * y + z * z;
        }

        double X() const { return mPosition[0]; }
        double Y() const { return mPosition[1]; }
        double Z() const { return mPosition[2]; }
    };
    typedef std::shared_ptr<Vertex3> Vertex3Ptr;
} // namespace HDV::Primitives

#endif /* _HDV_Vertex3_H_ */