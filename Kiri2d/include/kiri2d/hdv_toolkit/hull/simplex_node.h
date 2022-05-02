/***
 * @Author: Xu.WANG
 * @Date: 2022-05-02 13:22:27
 * @LastEditTime: 2022-05-02 15:18:31
 * @LastEditors: Xu.WANG
 * @Description:
 */
#ifndef _HDV_SIMPLEX_NODE_H_
#define _HDV_SIMPLEX_NODE_H_

#pragma once

#include <kiri_pch.h>
namespace HDV::Hull
{
    class SimplexNode
    {
    public:
        explicit SimplexNode()
            : Next{nullptr}, Prev{}
        {
        }

        explicit SimplexNode(
            int dimension,
            std::vector<int> &beyondList)
            : Next{nullptr}, Prev{}
        {
            mNormals.assign(dimension, 0.0);

            mVertices.assign(dimension, -1);
            mAdjacentFaces.assign(dimension, std::make_shared<SimplexNode>());
            mBeyondList = beyondList;
        }

        virtual ~SimplexNode()
        {
        }

        std::shared_ptr<SimplexNode> Next;
        std::weak_ptr<SimplexNode> Prev;

        int tag() const
        {
            return mTag;
        }

        int furthestVertex() const
        {
            return mFurthestVertex;
        }

        double offset() const
        {
            return mOffset;
        }

        bool isNormalFlipped() const
        {
            return mIsNormalFlipped;
        }

        bool inLinkList() const
        {
            return mInLinkList;
        }

        std::vector<int> &vertices()
        {
            return mVertices;
        }

        std::vector<double> &normals()
        {
            return mNormals;
        }

        std::vector<int> &beyondList()
        {
            return mBeyondList;
        }

        std::vector<int> &verticesBeyond()
        {
            return mVerticesBeyond;
        }

        std::vector<std::shared_ptr<SimplexNode>> &adjacentFaces()
        {
            return mAdjacentFaces;
        }

        void setTag(int tag)
        {
            mTag = tag;
        }

        void setFurthestVertex(int vertexId)
        {
            mFurthestVertex = vertexId;
        }

        void setOffset(double offset)
        {
            mOffset = offset;
        }

        void setIsNormalFlipped(bool isNormalFlipped)
        {
            mIsNormalFlipped = isNormalFlipped;
        }

        void setInLinkList(bool inLinkList)
        {
            mInLinkList = inLinkList;
        }

        void clear()
        {
            Next = nullptr;
            Prev.reset();

            mFurthestVertex = -1;
            mBeyondList.clear();
            mVerticesBeyond.clear();
            mVertices.clear();
            mAdjacentFaces.clear();
        }

        void toString()
        {
            std::string vertexStr = "";
            for (size_t i = 0; i < mVertices.size(); i++)
            {
                vertexStr += std::to_string(mVertices[i]) + "->";
            }

            KIRI_LOG_DEBUG("Tag = {0}; InList = {1}; Verts={2}; Normal={3},{4},{5}; mOffset={6}; mIsNormalFlipped={7}, mFurthestVertex={8}",
                           mTag,
                           mInLinkList,
                           vertexStr,
                           mNormals[0], mNormals[1], mNormals[2], mOffset, mIsNormalFlipped,
                           mFurthestVertex);
        }

    private:
        int mTag = 0;
        int mFurthestVertex = -1;
        double mOffset = 0.0;
        bool mInLinkList = false;
        bool mIsNormalFlipped = false;

        std::vector<int> mVertices;
        std::vector<double> mNormals;

        std::vector<int> mBeyondList;
        std::vector<int> mVerticesBeyond;
        std::vector<std::shared_ptr<SimplexNode>> mAdjacentFaces;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_NODE_H_ */