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

#include <kiri2d/hdv_toolkit/hull/simplex_wrap.h>
namespace HDV::Hull
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class MathHelper
    {
    public:
        explicit MathHelper() {}

        virtual ~MathHelper() noexcept {}

        static float LengthSquared(std::vector<float> x)
        {
            auto norm = 0.f;
            for (int i = 0; i < x.size(); i++)
            {
                auto t = x[i];
                norm += t * t;
            }
            return norm;
        }

        static std::vector<float> SubtractFast(std::vector<float> x, std::vector<float> y)
        {
            std::vector<float> target;
            auto d = x.size();
            target.assign(d, 0.f);
            for (auto i = 0; i < d; i++)
                target[i] = x[i] - y[i];

            return target;
        }

        static void FindNormalVector4D(const std::vector<VERTEX> &vertices, std::vector<float> &normal)
        {
            auto x = SubtractFast(vertices[1]->GetPosition(), vertices[0]->GetPosition());
            auto y = SubtractFast(vertices[2]->GetPosition(), vertices[1]->GetPosition());
            auto z = SubtractFast(vertices[3]->GetPosition(), vertices[2]->GetPosition());

            auto nx = x[3] * (y[2] * z[1] - y[1] * z[2]) + x[2] * (y[1] * z[3] - y[3] * z[1]) + x[1] * (y[3] * z[2] - y[2] * z[3]);
            auto ny = x[3] * (y[0] * z[2] - y[2] * z[0]) + x[2] * (y[3] * z[0] - y[0] * z[3]) + x[0] * (y[2] * z[3] - y[3] * z[2]);
            auto nz = x[3] * (y[1] * z[0] - y[0] * z[1]) + x[1] * (y[0] * z[3] - y[3] * z[0]) + x[0] * (y[3] * z[1] - y[1] * z[3]);
            auto nw = x[2] * (y[0] * z[1] - y[1] * z[0]) + x[1] * (y[2] * z[0] - y[0] * z[2]) + x[0] * (y[1] * z[2] - y[2] * z[1]);

            auto norm = std::sqrtf(nx * nx + ny * ny + nz * nz + nw * nw);

            auto f = 1.f / norm;
            normal[0] = f * nx;
            normal[1] = f * ny;
            normal[2] = f * nz;
            normal[3] = f * nw;
        }

        static void FindNormalVector3D(const std::vector<VERTEX> &vertices, std::vector<float> &normal)
        {
            auto x = SubtractFast(vertices[1]->GetPosition(), vertices[0]->GetPosition());
            auto y = SubtractFast(vertices[2]->GetPosition(), vertices[1]->GetPosition());

            auto nx = x[1] * y[2] - x[2] * y[1];
            auto ny = x[2] * y[0] - x[0] * y[2];
            auto nz = x[0] * y[1] - x[1] * y[0];

            auto norm = std::sqrtf(nx * nx + ny * ny + nz * nz);

            auto f = 1.f / norm;
            normal[0] = f * nx;
            normal[1] = f * ny;
            normal[2] = f * nz;
        }

        static void FindNormalVector2D(const std::vector<VERTEX> &vertices, std::vector<float> &normal)
        {
            auto x = SubtractFast(vertices[1]->GetPosition(), vertices[0]->GetPosition());

            auto nx = -x[1];
            auto ny = x[0];

            auto norm = std::sqrtf(nx * nx + ny * ny);

            auto f = 1.f / norm;
            normal[0] = f * nx;
            normal[1] = f * ny;
        }

        static void FindNormalVector(const std::vector<VERTEX> &vertices, std::vector<float> &normalData)
        {
            switch (vertices[0]->GetDimension())
            {
            case 2:
                FindNormalVector2D(vertices, normalData);
                return;
            case 3:
                FindNormalVector3D(vertices, normalData);
                return;
            case 4:
                FindNormalVector4D(vertices, normalData);
                return;
            }
        }

        /// <summary>
        /// Check if the vertex is "visible" from the face.
        /// The vertex is "over face" if the return value is > Constants.PlaneDistanceTolerance.
        /// </summary>
        /// <returns>The vertex is "over face" if the result is positive.</returns>
        static float GetVertexDistance(VERTEX v, const std::shared_ptr<SimplexWrap<VERTEX>> &f)
        {
            auto normal = f->GetNormals();
            auto p = v->GetPosition();
            auto distance = f->GetOffset();
            for (auto i = 0; i < v->GetDimension(); i++)
                distance += normal[i] * p[i];
            return distance;
        }
    };

} // namespace HDV::Hull

#endif /* _HDV_MATH_HELPER_H_ */