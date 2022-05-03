/***
 * @Author: Xu.WANG
 * @Date: 2021-12-05 19:44:04
 * @LastEditTime: 2021-12-05 19:47:34
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_MATH_HELPER_H_
#define _HDV_MATH_HELPER_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/simplex_node.h>
namespace HDV::Hull
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr>
    class MathHelper
    {
    public:
        explicit MathHelper()
        {
        }

        virtual ~MathHelper()
        {
        }

        static double lengthSquared(std::vector<double> x)
        {
            auto norm = 0.0;
            for (int i = 0; i < x.size(); i++)
            {
                auto t = x[i];
                norm += t * t;
            }
            return norm;
        }

        static std::vector<double> subtractVectorD(std::vector<double> x, std::vector<double> y)
        {
            std::vector<double> target;
            auto d = x.size();
            target.assign(d, 0.0);
            for (auto i = 0; i < d; i++)
                target[i] = x[i] - y[i];

            return target;
        }

        static void findNormalVector4D(const std::vector<VERTEXPTR> &vertices, std::vector<double> &normal)
        {
            auto x = subtractVectorD(vertices[1]->positions(), vertices[0]->positions());
            auto y = subtractVectorD(vertices[2]->positions(), vertices[1]->positions());
            auto z = subtractVectorD(vertices[3]->positions(), vertices[2]->positions());

            auto nx = x[3] * (y[2] * z[1] - y[1] * z[2]) + x[2] * (y[1] * z[3] - y[3] * z[1]) + x[1] * (y[3] * z[2] - y[2] * z[3]);
            auto ny = x[3] * (y[0] * z[2] - y[2] * z[0]) + x[2] * (y[3] * z[0] - y[0] * z[3]) + x[0] * (y[2] * z[3] - y[3] * z[2]);
            auto nz = x[3] * (y[1] * z[0] - y[0] * z[1]) + x[1] * (y[0] * z[3] - y[3] * z[0]) + x[0] * (y[3] * z[1] - y[1] * z[3]);
            auto nw = x[2] * (y[0] * z[1] - y[1] * z[0]) + x[1] * (y[2] * z[0] - y[0] * z[2]) + x[0] * (y[1] * z[2] - y[2] * z[1]);

            auto norm = std::sqrt(nx * nx + ny * ny + nz * nz + nw * nw);

            auto f = 1.0 / norm;
            normal[0] = f * nx;
            normal[1] = f * ny;
            normal[2] = f * nz;
            normal[3] = f * nw;
        }

        static void findNormalVector3D(const std::vector<VERTEXPTR> &vertices, std::vector<double> &normal)
        {
            auto x = subtractVectorD(vertices[1]->positions(), vertices[0]->positions());
            auto y = subtractVectorD(vertices[2]->positions(), vertices[1]->positions());

            auto nx = x[1] * y[2] - x[2] * y[1];
            auto ny = x[2] * y[0] - x[0] * y[2];
            auto nz = x[0] * y[1] - x[1] * y[0];

            auto norm = std::sqrt(nx * nx + ny * ny + nz * nz);

            auto f = 1.0 / norm;
            normal[0] = f * nx;
            normal[1] = f * ny;
            normal[2] = f * nz;
        }

        static void findNormalVector2D(const std::vector<VERTEXPTR> &vertices, std::vector<double> &normal)
        {
            auto x = subtractVectorD(vertices[1]->positions(), vertices[0]->positions());

            auto nx = -x[1];
            auto ny = x[0];

            auto norm = std::sqrt(nx * nx + ny * ny);

            auto f = 1.0 / norm;
            normal[0] = f * nx;
            normal[1] = f * ny;
        }

        static void findNormalVector(const std::vector<VERTEXPTR> &vertices, std::vector<double> &normalData)
        {
            switch (vertices[0]->dimension())
            {
            case 2:
                findNormalVector2D(vertices, normalData);
                return;
            case 3:
                findNormalVector3D(vertices, normalData);
                return;
            case 4:
                findNormalVector4D(vertices, normalData);
                return;
            }
        }

        static double vertexDistanceToSimplex(const VERTEXPTR &v, const std::shared_ptr<SimplexNode> &f)
        {
            auto normal = f->normals();
            auto p = v->positions();
            auto distance = f->offset();
            for (auto i = 0; i < v->dimension(); i++)
                distance += normal[i] * p[i];

            return distance;
        }
    };

} // namespace HDV::Hull

#endif /* _HDV_MATH_HELPER_H_ */