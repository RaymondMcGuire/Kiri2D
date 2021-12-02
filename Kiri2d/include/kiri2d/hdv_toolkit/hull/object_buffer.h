/***
 * @Author: Xu.WANG
 * @Date: 2021-12-02 22:01:05
 * @LastEditTime: 2021-12-02 22:13:13
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_OBJECT_BUFFER_H_
#define _HDV_OBJECT_BUFFER_H_

#pragma once

#include <kiri2d/hdv_toolkit/hull/simplex_list.h>
#include <kiri2d/hdv_toolkit/hull/connector_list.h>
#include <kiri2d/hdv_toolkit/hull/object_manager.h>
namespace HDV::Hull
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class ObjectBuffer
    {
    public:
        explicit ObjectBuffer() {}
        explicit ObjectBuffer(int dimension)
        {
            mDimension = dimension;

            mUpdateBuffer.assign(dimension, SimplexWrap<VERTEX>());
            mUpdateIndices.assign(dimension, -1);

            mObjectManager = std::make_shared<ObjectManager<VERTEX>>(dimension);
            mEmptyBuffer = std::make_shared<VertexBuffer<VERTEX>>();
            mBeyondBuffer = std::make_shared<VertexBuffer<VERTEX>>();

            mConnectorTable.assign(CONNECTOR_TABLE_SIZE, ConnectorList<VERTEX>());
        }

        virtual ~ObjectBuffer() noexcept {}

        void Clear()
        {

            mUpdateBuffer.assign(dimension, SimplexWrap<VERTEX>());
            mUpdateIndices.assign(dimension, -1);

            mInputVertices.clear();
            mCurrentVertex = nullptr;
            mFurthestVertex = nullptr;
            mMaxDistance = -std::numeric_limits<float>::max();

            mConvexSimplexs.clear();
            mAffectedFaceBuffer.clear();
            mTraverseStack.clear();
            mSingularVertices.clear();
            mConeFaceBuffer.clear();

            mObjectManager->Clear();
            mUnprocessedFaces->Clear();
            mEmptyBuffer->Clear();
            mBeyondBuffer->Clear();

            for (auto i = 0; i < CONNECTOR_TABLE_SIZE; i++)
                mConnectorTable[i]->Clear();
        }

        void AddInput(const std::vector<VERTEX> &input, bool assignIds, bool checkInput)
        {
            auto count = input.size();
            mInputVertices = input;

            if (assignIds)
            {
                for (auto i = 0; i < count; i++)
                    mInputVertices[i]->SetId(i);
            }

            if (checkInput)
            {
                std::unordered_set<int> set;

                for (auto i = 0; i < count; i++)
                {
                    if (input[i] == nullptr)
                        throw std::invalid_argument("Input has a null vertex!");

                    if (input[i]->GetDimension() != mDimension)
                        throw std::invalid_argument("Input vertex is not the correct dimension!", std::to_string(input[i]->GetDimension()));

                    if (set.count(input[i]->GetId()))
                        throw std::invalid_argument("Input vertex id is not unique!", std::to_string(input[i]->GetId()));
                    else
                        set.insert(input[i]->GetId());
                }
            }
        }

    private:
        const int CONNECTOR_TABLE_SIZE = 2017;

        int mDimension = -1;
        float mMaxDistance = -std::numeric_limits<float>::max();

        VERTEX mCurrentVertex;
        VERTEX mFurthestVertex;

        std::shared_ptr<SimplexList<VERTEX>> mUnprocessedFaces;
        std::shared_ptr<ObjectManager<VERTEX>> mObjectManager;

        std::shared_ptr<VertexBuffer<VERTEX>> mEmptyBuffer;
        std::shared_ptr<VertexBuffer<VERTEX>> mBeyondBuffer;

        std::vector<VERTEX> mInputVertices;
        std::vector<SimplexWrap<VERTEX>> mConvexSimplexs;
        std::vector<SimplexWrap<VERTEX>> mAffectedFaceBuffer;
        std::vector<SimplexWrap<VERTEX>> mTraverseStack;
        std::unordered_set<VERTEX> mSingularVertices;
        std::vector<DeferredSimplex<VERTEX>> mConeFaceBuffer;
        std::vector<SimplexWrap<VERTEX>> mUpdateBuffer;
        std::vector<int> mUpdateIndices;
        std::vector<ConnectorList<VERTEX>> mConnectorTable;
    };

} // namespace HDV::Hull

#endif /* _HDV_OBJECT_BUFFER_H_ */