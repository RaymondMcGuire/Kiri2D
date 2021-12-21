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
        explicit Vertex(int dimension) { mPosition.assign(dimension, 0.f); }
        explicit Vertex(int dimension, int id) : Vertex(dimension) { mId = id; }
        virtual ~Vertex() noexcept {}

        int GetId() const { return mId; }
        int GetTag() const { return mTag; }
        int GetDimension() { return (mPosition.empty()) ? 0 : mPosition.size(); }
        float GetWeight() { return mWeight; }
        const std::vector<float> &GetPosition() const { return mPosition; }

        void SetId(int id) { mId = id; }
        void SetTag(int tag) { mTag = tag; }
        void SetWeight(float weight) { mWeight = weight; }

        const std::vector<float> &GetPosition() { return mPosition; }

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

        std::vector<float> mPosition;

        void SetAsBoundaryVertex() { mIsBoundaryVertex = true; }
        bool GetIsBoundaryVertex() { return mIsBoundaryVertex; }

        float Magnitude()
        {
            return std::sqrtf(SqrMagnitude());
        }

        float SqrMagnitude()
        {

            auto sum = 0.f;
            auto dim = GetDimension();

            for (auto i = 0; i < dim; i++)
                sum += mPosition[i] * mPosition[i];

            return sum;
        }

        float Distance(Vertex v)
        {
            return std::sqrtf(SqrDistance(v));
        }

        float SqrDistance(Vertex v)
        {
            auto dim = std::min(GetDimension(), v.GetDimension());
            auto sum = 0.f;

            for (auto i = 0; i < dim; i++)
            {
                float x = mPosition[i] - v.GetPosition()[i];
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

    protected:
        int mId = -1;
        int mTag = 0;
        float mWeight = 0.f;
        bool mIsBoundaryVertex = false;
    };
    typedef std::shared_ptr<Vertex> VertexPtr;
} // namespace HDV::Primitives

#endif /* _HDV_VERTEX_H_ */