/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-12-23 17:57:21
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 18:35:49
 * @FilePath: \core\include\kiri2d\hdv_toolkit\primitives\vertex2.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
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
        explicit Vertex2()
            : Vertex(2)
        {
        }

        explicit Vertex2(int id)
            : Vertex(2, id)
        {
        }

        explicit Vertex2(double x, double y)
            : Vertex(2)
        {
            mPosition[0] = x;
            mPosition[1] = y;
        }

        explicit Vertex2(double x, double y, int id)
            : Vertex(2, id)
        {
            mPosition[0] = x;
            mPosition[1] = y;
        }

        virtual ~Vertex2()
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

        void set(double x, double y)
        {
            mPosition[0] = x;
            mPosition[1] = y;
        }

        double distanceTo(std::shared_ptr<Vertex2> v) const
        {
            return std::sqrt(distanceSquaredTo(v->x(), v->y()));
        }

        double distanceTo(double vx, double vy) const
        {
            return std::sqrt(distanceSquaredTo(vx, vy));
        }

        double distanceSquaredTo(double vx, double vy) const
        {
            auto x = mPosition[0] - vx;
            auto y = mPosition[1] - vy;

            return x * x + y * y;
        }
    };
    typedef std::shared_ptr<Vertex2> Vertex2Ptr;
} // namespace HDV::Primitives

#endif /* _HDV_VERTEX2_H_ */