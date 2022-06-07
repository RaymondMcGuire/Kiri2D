/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-12-23 17:57:21
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 18:35:13
 * @FilePath: \core\include\kiri2d\hdv_toolkit\primitives\vertex4.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
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
        explicit Vertex4()
            : Vertex(4)
        {
        }

        explicit Vertex4(int id)
            : Vertex(4, id)
        {
        }

        explicit Vertex4(double x, double y, double z, double w)
            : Vertex(4)
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

        double w() const
        {
            return mPosition[3];
        }

        void set(double x, double y, double z, double w)
        {
            mPosition[0] = x;
            mPosition[1] = y;
            mPosition[2] = z;
            mPosition[3] = w;
        }

        double distanceTo(std::shared_ptr<Vertex4> v) const
        {
            return std::sqrt(distanceSquaredTo(v->x(), v->y(), v->z(), v->w()));
        }

        double distanceTo(double vx, double vy, double vz, double vw) const
        {
            return std::sqrt(distanceSquaredTo(vx, vy, vz, vw));
        }

        double distanceSquaredTo(double vx, double vy, double vz, double vw) const
        {
            auto x = mPosition[0] - vx;
            auto y = mPosition[1] - vy;
            auto z = mPosition[2] - vz;
            auto w = mPosition[3] - vw;

            return x * x + y * y + z * z + w * w;
        }
    };

    typedef std::shared_ptr<Vertex4> Vertex4Ptr;
} // namespace HDV::Primitives

#endif /* _HDV_Vertex4_H_ */