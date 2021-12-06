/***
 * @Author: Xu.WANG
 * @Date: 2021-12-01 20:39:30
 * @LastEditTime: 2021-12-01 20:39:30
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_SIMPLEX_H_
#define _HDV_SIMPLEX_H_

#pragma once

#include <kiri2d/hdv_toolkit/primitives/vertex.h>

namespace HDV::Primitives
{
    template <typename VERTEX = VertexPtr>
    class Simplex
    {
    public:
        explicit Simplex(int id, int dimension)
        {
            if (dimension < 2 || dimension > 4)
                throw std::invalid_argument("Invalid number of dimension for Simplex!");

            mId = id;
            mDimension = dimension;
            mVertices.assign(dimension, VERTEX());
            mNormal.assign(dimension, 0.f);
            mCentroid.assign(dimension, 0.f);
        }
        virtual ~Simplex() noexcept {}

        int GetId() const { return mTag; }
        int GetTag() const { return mTag; }
        int GetDimension() const { return mDimension; }
        float GetOffset() const { return mOffset; }
        bool GetNormalFlipped() const { return mIsNormalFlipped; }

        std::vector<float> GetNormals() { return mNormal; }
        const std::vector<VERTEX> &Vertices { return mVertices; }
        const std::vector<std::shared_ptr<Simplex<VERTEX>>> &GetAdjacent() { return mAdjacent; }

        void SetTag(int tag) { mTag = tag; }
        void SetOffset(float offset) { mOffset = offset; }
        void SetNormalFlipped(int normal_flipped) { mIsNormalFlipped = normal_flipped; }

        float Dot(VERTEX v)
        {
            auto dim = mDimension;
            if (v.GetDimension() != dim)
                return 0.f;

            auto dp = 0.0f;

            for (auto i = 0; i < dim; i++)
                dp += mNormal[i] * v->GetPosition()[i];

            return dp;
        }

        bool Remove(const std::shared_ptr<Simplex<VERTEX>> &simplex)
        {
            auto n = mAdjacent.size();

            for (auto i = 0; i < n; i++)
            {
                if (mAdjacent[i].GetId() == simplex.GetId())
                {
                    mAdjacent.erase(mAdjacent.begin() + i);
                    return true;
                }
            }

            return false;
        }

        void CalculateNormal()
        {

            switch (mDimension)
            {
            case 2:
                CalculateNormal2D();
                break;

            case 3:
                CalculateNormal3D();
                break;

            case 4:
                CalculateNormal4D();
                break;

            default:
                throw std::invalid_argument("Invalid number of dimension for Simplex!");
            }
        }

        void CalculateCentroid()
        {

            switch (mDimension)
            {
            case 2:
                CalculateCentroid2D();
                break;

            case 3:
                CalculateCentroid3D();
                break;

            case 4:
                CalculateCentroid4D();
                break;

            default:
                throw std::invalid_argument("Invalid number of dimension for Simplex!");
            }
        }

        void UpdateAdjacency(const std::shared_ptr<Simplex<VERTEX>> &simplex)
        {

            auto lv = mVertices;
            auto rv = simplex.Vertices;

            auto i = 0;
            auto dim = mDimension;

            // reset marks on the 1st face
            for (i = 0; i < dim; i++)
                lv[i].SetTag(0);

            // mark all vertices on the 2nd face
            for (i = 0; i < dim; i++)
                rv[i].SetTag(1);

            // find the 1st false index
            for (i = 0; i < dim; i++)
                if (lv[i].GetTag() == 0)
                    break;

            // no vertex was marked
            if (i == dim)
                return;

            // check if only 1 vertex wasn't marked
            for (auto j = i + 1; j < dim; j++)
                if (lv[j].GetTag() == 0)
                    return;

            // if we are here, the two faces share an edge
            if (i < mAdjacent.size())
                mAdjacent[i] = simplex;
            else if (i == mAdjacent.size())
                mAdjacent.emplace_back(simplex);
            else
                throw std::invalid_argument("Invalid index for add adjacent simplex 1!");

            // update the adj. face on the other face - find the vertex that remains marked
            for (i = 0; i < dim; i++)
                lv[i].SetTag(0);

            for (i = 0; i < dim; i++)
            {
                if (rv[i].GetTag() == 1)
                {
                    auto spx_adj = simplex.GetAdjacent();

                    if (i < spx_adj.size())
                        spx_adj[i] = this;
                    else if (i == spx_adj.size())
                        spx_adj.emplace_back(this);
                    else
                        throw std::invalid_argument("Invalid index for add adjacent simplex 2!");

                    break;
                }
            }
        }

        bool HasNullAdjacency()
        {

            if (mAdjacent.size() != mDimension)
                return true;

            return false;
        }

        bool HasAdjacency()
        {
            if (mAdjacent.empty())
                return false;

            return true;
        }

        void ToString()
        {

            std::string indexs = "";

            auto dim = mDimension;
            for (auto i = 0; i < dim; i++)
            {
                indexs += std::to_string(mVertices[i].GetId());
                if (i != dim - 1)
                    indexs += ",";
            }

            return KIRI_LOG_DEBUG("[Simplex: Dimension={0},  Vertices={1}]", mDimension, indexs);
        }

    private:
        int mId = -1;
        int mTag = -1;
        int mDimension;
        float mOffset;
        bool mIsNormalFlipped;

        std::vector<VERTEX> mVertices;
        std::vector<float> mNormal;
        std::vector<float> mCentroid;
        std::vector<std::shared_ptr<Simplex<VERTEX>>> mAdjacent;

        std::vector<float> Subtract(std::vector<float> x, std::vector<float> y)
        {
            std::vector<float> target;
            for (auto i = 0; i < mDimension; i++)
            {
                target[i] = x[i] - y[i];
            }
            return target;
        }

        void CalculateNormal2D()
        {

            if (mVertices.size() != 2)
                throw std::invalid_argument("Invalid dimension for Vertices!");

            auto ntX = Subtract(mVertices[0]->GetPosition(), mVertices[1]->GetPosition());

            auto nx = -ntX[1];
            auto ny = ntX[0];

            auto norm = std::sqrtf(nx * nx + ny * ny);

            auto f = 1.f / norm;
            mNormal[0] = f * nx;
            mNormal[1] = f * ny;
        }

        void CalculateNormal3D()
        {
            if (mVertices.size() != 3)
                throw std::invalid_argument("Invalid dimension for Vertices!");

            auto ntX = Subtract(mVertices[1]->GetPosition(), mVertices[0]->GetPosition());
            auto ntY = Subtract(mVertices[2]->GetPosition(), mVertices[1]->GetPosition());

            auto nx = ntX[1] * ntY[2] - ntX[2] * ntY[1];
            auto ny = ntX[2] * ntY[0] - ntX[0] * ntY[2];
            auto nz = ntX[0] * ntY[1] - ntX[1] * ntY[0];

            auto norm = std::sqrtf(nx * nx + ny * ny + nz * nz);

            auto f = 1.f / norm;
            mNormal[0] = f * nx;
            mNormal[1] = f * ny;
            mNormal[2] = f * nz;
        }

        void CalculateNormal4D()
        {
            if (mVertices.size() != 4)
                throw std::invalid_argument("Invalid dimension for Vertices!");

            auto ntX = Subtract(mVertices[1]->GetPosition(), mVertices[0]->GetPosition());
            auto ntY = Subtract(mVertices[2]->GetPosition(), mVertices[1]->GetPosition());
            auto ntZ = Subtract(mVertices[3]->GetPosition(), mVertices[2]->GetPosition());

            auto x = ntX;
            auto y = ntY;
            auto z = ntZ;

            // This was generated using Mathematica
            auto nx = x[3] * (y[2] * z[1] - y[1] * z[2]) + x[2] * (y[1] * z[3] - y[3] * z[1]) + x[1] * (y[3] * z[2] - y[2] * z[3]);
            auto ny = x[3] * (y[0] * z[2] - y[2] * z[0]) + x[2] * (y[3] * z[0] - y[0] * z[3]) + x[0] * (y[2] * z[3] - y[3] * z[2]);
            auto nz = x[3] * (y[1] * z[0] - y[0] * z[1]) + x[1] * (y[0] * z[3] - y[3] * z[0]) + x[0] * (y[3] * z[1] - y[1] * z[3]);
            auto nw = x[2] * (y[0] * z[1] - y[1] * z[0]) + x[1] * (y[2] * z[0] - y[0] * z[2]) + x[0] * (y[1] * z[2] - y[2] * z[1]);

            auto norm = std::sqrtf(nx * nx + ny * ny + nz * nz + nw * nw);

            auto f = 1.f / norm;
            Normal[0] = f * nx;
            Normal[1] = f * ny;
            Normal[2] = f * nz;
            Normal[3] = f * nw;
        }

        void CalculateCentroid2D()
        {
            if (mVertices.size() != 2)
                throw std::invalid_argument("Invalid dimension for Vertices!");
            mCentroid[0] = (mVertices[0]->GetPosition()[0] + mVertices[1]->GetPosition()[0]) / 2.f;
            mCentroid[1] = (mVertices[0]->GetPosition()[1] + mVertices[1]->GetPosition()[1]) / 2.f;
        }

        void CalculateCentroid3D()
        {
            if (mVertices.size() != 3)
                throw std::invalid_argument("Invalid dimension for Vertices!");
            mCentroid[0] = (mVertices[0]->GetPosition()[0] + mVertices[1]->GetPosition()[0] + mVertices[2]->GetPosition()[0]) / 3.f;
            mCentroid[1] = (mVertices[0]->GetPosition()[1] + mVertices[1]->GetPosition()[1] + mVertices[2]->GetPosition()[1]) / 3.f;
            mCentroid[2] = (mVertices[0]->GetPosition()[2] + mVertices[1]->GetPosition()[2] + mVertices[2]->GetPosition()[2]) / 3.f;
        }

        void CalculateCentroid4D()
        {
            if (mVertices.size() != 4)
                throw std::invalid_argument("Invalid dimension for Vertices!");
            mCentroid[0] = (mVertices[0]->GetPosition()[0] + mVertices[1]->GetPosition()[0] + mVertices[2]->GetPosition()[0] + mVertices[3]->GetPosition()[0]) / 4.f;
            mCentroid[1] = (mVertices[0]->GetPosition()[1] + mVertices[1]->GetPosition()[1] + mVertices[2]->GetPosition()[1] + mVertices[3]->GetPosition()[1]) / 4.f;
            mCentroid[2] = (mVertices[0]->GetPosition()[2] + mVertices[1]->GetPosition()[2] + mVertices[2]->GetPosition()[2] + mVertices[3]->GetPosition()[2]) / 4.f;
            mCentroid[3] = (mVertices[0]->GetPosition()[3] + mVertices[1]->GetPosition()[3] + mVertices[2]->GetPosition()[3] + mVertices[3]->GetPosition()[3]) / 4.f;
        }
    };
} // namespace HDV::Primitives

#endif /* _HDV_SIMPLEX_H_ */