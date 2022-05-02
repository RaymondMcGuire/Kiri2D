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
        virtual ~Vertex() {}

        int id() const { return mId; }
        int tag() const { return mTag; }
        int dimension() const { return (mPosition.empty()) ? 0 : mPosition.size(); }
        double weight() const { return mWeight; }
        double radius() const { return mRadius; }
        bool isBoundaryVertex() { return mIsBoundaryVertex; }
        std::vector<double> &positions() { return mPosition; }
        std::vector<std::shared_ptr<Vertex>> &neighbors() { return mNeighbors; }

        void setId(int id) { mId = id; }
        void setTag(int tag) { mTag = tag; }
        void setWeight(double weight) { mWeight = weight; }
        void setRadius(double radius) { mRadius = radius; }
        void setAsBoundaryVertex() { mIsBoundaryVertex = true; }

        double length() { return std::sqrt(lengthSquared()); }

        double lengthSquared()
        {
            auto sum = 0.0;
            auto dim = dimension();

            for (auto i = 0; i < dim; i++)
                sum += mPosition[i] * mPosition[i];

            return sum;
        }

        double distanceTo(std::shared_ptr<Vertex> v)
        {
            return std::sqrt(distanceSquaredTo(v));
        }

        double distanceSquaredTo(std::shared_ptr<Vertex> v)
        {
            auto dim = std::min(dimension(), v->dimension());
            auto sum = 0.0;

            for (auto i = 0; i < dim; i++)
            {
                double x = mPosition[i] - v->positions()[i];
                sum += x * x;
            }

            return sum;
        }

        void reset()
        {
            mTag = 0;
            mId = -1;
            mWeight = 0.0;
            mRadius = 0.0;
            mIsBoundaryVertex = false;
        }

        void toString()
        {
            auto dim = dimension();
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
        double mWeight = 0.0;
        double mRadius = 0.0;
        bool mIsBoundaryVertex = false;

        std::vector<double> mPosition;
        std::vector<std::shared_ptr<Vertex>> mNeighbors;
    };
    typedef std::shared_ptr<Vertex> VertexPtr;
} // namespace HDV::Primitives

#endif /* _HDV_VERTEX_H_ */