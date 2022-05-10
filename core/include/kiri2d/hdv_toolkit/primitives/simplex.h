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
    template <typename VERTEXPTR = Vertex>
    class Simplex
    {
    public:
        explicit Simplex()
        {
        }

        explicit Simplex(int id, int dimension)
        {
            if (dimension < 2 || dimension > 4)
                throw std::invalid_argument("Invalid Number of Dimension for Simplex!");

            mId = id;
            mDimension = dimension;
            mVertices.assign(dimension, VERTEXPTR());
            mNormals.assign(dimension, 0.0);
            mAdjacent.assign(dimension, std::make_shared<Simplex<VERTEXPTR>>());
        }

        virtual ~Simplex()
        {
        }

        int id() const
        {
            return mId;
        }

        int tag() const
        {
            return mTag;
        }

        int dimension() const
        {
            return mDimension;
        }

        double offset() const
        {
            return mOffset;
        }

        bool normalFlipped() const
        {
            return mIsNormalFlipped;
        }

        std::vector<VERTEXPTR> &vertices()
        {
            return mVertices;
        }

        std::vector<double> &normals()
        {
            return mNormals;
        }

        std::vector<std::shared_ptr<Simplex<VERTEXPTR>>> &adjacents()
        {
            return mAdjacent;
        }

        void setTag(int tag)
        {
            mTag = tag;
        }

        void setOffset(double offset)
        {
            mOffset = offset;
        }

        void setNormalFlipped(int normalFlipped)
        {
            mIsNormalFlipped = normalFlipped;
        }

        void clear()
        {
            mVertices.clear();
            mNormals.clear();
            mAdjacent.clear();
        }

        bool remove(const std::shared_ptr<Simplex<VERTEXPTR>> &simplex)
        {
            auto n = mAdjacent.size();

            for (auto i = 0; i < n; i++)
            {
                if (mAdjacent[i]->id() == simplex->id())
                {
                    mAdjacent.erase(mAdjacent.begin() + i);
                    return true;
                }
            }

            return false;
        }

        void toString()
        {

            std::string indexs = "";

            auto dim = mDimension;
            for (auto i = 0; i < dim; i++)
            {
                indexs += std::to_string(mVertices[i]->id()) + "=";

                for (auto j = 0; j < dim; j++)
                {
                    indexs += std::to_string(mVertices[i]->positions()[j]);

                    if (j != dim - 1)
                        indexs += ",";
                }

                if (i != dim - 1)
                    indexs += ";";
            }

            return KIRI_LOG_DEBUG("[Simplex: Dimension={0},  mVertices:{1}]", mDimension, indexs);
        }

    private:
        int mId = -1;
        int mTag = 0;
        int mDimension;
        double mOffset;
        bool mIsNormalFlipped;

        std::vector<VERTEXPTR> mVertices;
        std::vector<double> mNormals;
        std::vector<std::shared_ptr<Simplex<VERTEXPTR>>> mAdjacent;
    };
} // namespace HDV::Primitives

#endif /* _HDV_SIMPLEX_H_ */