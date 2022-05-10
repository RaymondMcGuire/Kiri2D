/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2022-05-10 10:11:21
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
        explicit Vertex3()
            : Vertex(3)
        {
        }

        explicit Vertex3(int id)
            : Vertex(3, id)
        {
        }

        explicit Vertex3(double x, double y, double z)
            : Vertex(3)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
        }

        explicit Vertex3(double x, double y, double z, int id)
            : Vertex(3, id)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
        }

        virtual ~Vertex3()
        {
        }

        double x() const
        {
            return mPosition[0];
        }
        double y() const
        {
            return mPosition[1];
        }
        double z() const
        {
            return mPosition[2];
        }

        void set(double x, double y, double z)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
        }

        double distanceTo(std::shared_ptr<Vertex3> v) const
        {
            return std::sqrt(distanceSquaredTo(v->x(), v->y(), v->z()));
        }

        double distanceTo(double vx, double vy, double vz) const
        {
            return std::sqrt(distanceSquaredTo(vx, vy, vz));
        }

        double distanceSquaredTo(double vx, double vy, double vz) const
        {
            auto x = mPosition[0] - vx;
            auto y = mPosition[1] - vy;
            auto z = mPosition[2] - vz;

            return x * x + y * y + z * z;
        }
    };
    typedef std::shared_ptr<Vertex3> Vertex3Ptr;
} // namespace HDV::Primitives

#endif /* _HDV_Vertex3_H_ */