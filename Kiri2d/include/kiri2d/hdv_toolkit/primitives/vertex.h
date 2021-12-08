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
        const std::vector<float> &GetPosition() const { return mPosition; }

        void SetId(int id) { mId = id; }
        void SetTag(int tag) { mTag = tag; }

        float Magnitude();
        float SqrMagnitude();
        float Distance(Vertex v);
        float SqrDistance(Vertex v);

        const std::vector<float> &GetPosition() { return mPosition; }

        void ToString();
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

    protected:
        int mId = -1;
        int mTag = 0;
    };
    typedef std::shared_ptr<Vertex> VertexPtr;
} // namespace HDV::Primitives

#endif /* _HDV_VERTEX_H_ */