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

            UpdateBuffer.assign(dimension, std::make_shared<SimplexWrap<VERTEX>>());
            UpdateIndices.assign(dimension, -1);

           ObjManager = std::make_shared<ObjectManager<VERTEX>>(dimension);
            EmptyBuffer = std::make_shared<VertexBuffer<VERTEX>>();
            BeyondBuffer = std::make_shared<VertexBuffer<VERTEX>>();

            ConnectorTable.assign(CONNECTOR_TABLE_SIZE, std::make_shared <ConnectorList<VERTEX>>());
        }

        virtual ~ObjectBuffer() noexcept {}

        const int CONNECTOR_TABLE_SIZE = 2017;

        VERTEX CurrentVertex;
        VERTEX FurthestVertex;
        float MaxDistance = -std::numeric_limits<float>::max();

        std::shared_ptr<SimplexList<VERTEX>> UnprocessedFaces;
        std::shared_ptr<ObjectManager<VERTEX>> ObjManager;

        std::shared_ptr<VertexBuffer<VERTEX>> EmptyBuffer;
        std::shared_ptr<VertexBuffer<VERTEX>> BeyondBuffer;

        std::vector<VERTEX> InputVertices;
        std::vector<std::shared_ptr<SimplexWrap<VERTEX>>> ConvexSimplexs;
        std::vector<std::shared_ptr<SimplexWrap<VERTEX>>> AffectedFaceBuffer;
        std::stack<std::shared_ptr<SimplexWrap<VERTEX>>> TraverseStack;
        std::unordered_set<VERTEX> SingularVertices;
        std::vector<std::shared_ptr<DeferredSimplex<VERTEX>>> ConeFaceBuffer;
        std::vector<std::shared_ptr<SimplexWrap<VERTEX>>> UpdateBuffer;
        std::vector<int> UpdateIndices;
        std::vector<std::shared_ptr<ConnectorList<VERTEX>>> ConnectorTable;

        void Clear()
        {

            UpdateBuffer.assign(mDimension, std::make_shared<SimplexWrap<VERTEX>>());
            UpdateIndices.assign(mDimension, -1);

            InputVertices.clear();
            CurrentVertex = nullptr;
            FurthestVertex = nullptr;
            MaxDistance = -std::numeric_limits<float>::max();

            ConvexSimplexs.clear();
            AffectedFaceBuffer.clear();
            TraverseStack = std::stack<std::shared_ptr<SimplexWrap<VERTEX>>>();
            SingularVertices.clear();
            ConeFaceBuffer.clear();

            ObjManager->Clear();
            UnprocessedFaces->Clear();
            EmptyBuffer->Clear();
            BeyondBuffer->Clear();

            for (auto i = 0; i < CONNECTOR_TABLE_SIZE; i++)
                ConnectorTable[i]->Clear();
        }

        void AddInput(const std::vector<VERTEX> &input, bool assignIds, bool checkInput)
        {
            auto count = input.size();
            InputVertices = input;

            if (assignIds)
            {
                for (auto i = 0; i < count; i++)
                    InputVertices[i]->SetId(i);
            }

            if (checkInput)
            {
                std::unordered_set<int> set;

                for (auto i = 0; i < count; i++)
                {
                    if (input[i] == nullptr)
                        throw std::invalid_argument("Input has a null vertex!");

                    if (input[i]->GetDimension() != mDimension)
                        throw std::invalid_argument("Input vertex is not the correct dimension!");

                    if (set.count(input[i]->GetId()))
                        throw std::invalid_argument("Input vertex id is not unique!");
                    else
                        set.insert(input[i]->GetId());
                }
            }
        }

    private:
        int mDimension = -1;
    };

} // namespace HDV::Hull

#endif /* _HDV_OBJECT_BUFFER_H_ */