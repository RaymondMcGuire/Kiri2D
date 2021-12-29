/***
 * @Author: Xu.WANG
 * @Date: 2021-12-01 18:20:37
 * @LastEditTime: 2021-12-08 14:53:06
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_VERTEX_H_
#define _HDV_VERTEX_H_

#pragma once

#include <kiri_pch.h>

namespace HDV::Primitives
{
    class Vertex
    {
    public:
        explicit Vertex() {}
        explicit Vertex(int dimension) { mPosition.assign(dimension, 0.0); }
        explicit Vertex(int dimension, int id) : Vertex(dimension) { mId = id; }
        virtual ~Vertex() noexcept {}

        int GetId() const { return mId; }
        int GetTag() const { return mTag; }
        int GetDimension() { return (mPosition.empty()) ? 0 : mPosition.size(); }
        double GetWeight() { return mWeight; }
        const std::vector<double> &GetPosition() const { return mPosition; }

        void SetId(int id) { mId = id; }
        void SetTag(int tag) { mTag = tag; }
        void SetWeight(double weight) { mWeight = weight; }

        const std::vector<double> &GetPosition() { return mPosition; }

        std::string GetString()
        {

            auto dim = GetDimension();
            std::string data = "";
            for (auto i = 0; i < dim; i++)
            {
                data += std::to_string(mPosition[i]);
                if (i != dim - 1)
                    data += ",";
            }

            return "[Vertex: Id=" + std::to_string(mId) + ",Tag=" + std::to_string(mTag) + ", Dimension=" + std::to_string(dim) + ",Data=" + data + "]";
        }

        std::vector<double> mPosition;

        void SetAsBoundaryVertex() { mIsBoundaryVertex = true; }
        bool GetIsBoundaryVertex() { return mIsBoundaryVertex; }

        double Magnitude()
        {
            return std::sqrtf(SqrMagnitude());
        }

        double SqrMagnitude()
        {

            auto sum = 0.0;
            auto dim = GetDimension();

            for (auto i = 0; i < dim; i++)
                sum += mPosition[i] * mPosition[i];

            return sum;
        }

        double Distance(Vertex v)
        {
            return std::sqrtf(SqrDistance(v));
        }

        double SqrDistance(Vertex v)
        {
            auto dim = std::min(GetDimension(), v.GetDimension());
            auto sum = 0.0;

            for (auto i = 0; i < dim; i++)
            {
                double x = mPosition[i] - v.GetPosition()[i];
                sum += x * x;
            }

            return sum;
        }

        void ToString()
        {
            auto dim = GetDimension();
            std::string data = "";
            for (auto i = 0; i < dim; i++)
            {
                data += std::to_string(mPosition[i]);
                if (i != dim - 1)
                    data += ",";
            }

            KIRI_LOG_DEBUG("[Vertex: Id={0}, Dimension={1},  Data={2}, Tag={3}]", mId, dim, data, std::to_string(mTag));
        }

        void Reset() { mTag = 0; }

    protected:
        int mId = -1;
        int mTag = 0;
        double mWeight = 0.0;
        bool mIsBoundaryVertex = false;
    };
    typedef std::shared_ptr<Vertex> VertexPtr;
} // namespace HDV::Primitives

#endif /* _HDV_VERTEX_H_ */