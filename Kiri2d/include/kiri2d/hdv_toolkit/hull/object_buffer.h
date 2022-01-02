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
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr>
    class ObjectBuffer
    {
    public:
        explicit ObjectBuffer() {}
        explicit ObjectBuffer(int dimension)
        {
            mDimension = dimension;

            UpdateBuffer.assign(dimension, std::make_shared<SimplexWrap<VERTEXPTR>>());
            UpdateIndices.assign(dimension, -1);

            ObjManager = std::make_shared<ObjectManager<VERTEXPTR>>(dimension);
            EmptyBuffer = std::make_shared<VertexBuffer<VERTEXPTR>>();
            BeyondBuffer = std::make_shared<VertexBuffer<VERTEXPTR>>();

            UnprocessedFaces = std::make_shared<SimplexList<VERTEXPTR>>();

            ConnectorTable.assign(CONNECTOR_TABLE_SIZE, std::make_shared<ConnectorList<VERTEXPTR>>());
        }

        virtual ~ObjectBuffer() noexcept {}

        const int CONNECTOR_TABLE_SIZE = 2017;

        // VERTEXPTR CurrentVertex;
        // VERTEXPTR FurthestVertex;
        int CurrentVertex;
        int FurthestVertex;

        double MaxDistance = -std::numeric_limits<double>::max();

        std::shared_ptr<SimplexList<VERTEXPTR>> UnprocessedFaces;
        std::shared_ptr<ObjectManager<VERTEXPTR>> ObjManager;

        std::shared_ptr<VertexBuffer<VERTEXPTR>> EmptyBuffer;
        std::shared_ptr<VertexBuffer<VERTEXPTR>> BeyondBuffer;

        std::vector<VERTEXPTR> InputVertices;
        std::vector<std::shared_ptr<SimplexWrap<VERTEXPTR>>> ConvexSimplexs;
        std::vector<std::shared_ptr<SimplexWrap<VERTEXPTR>>> AffectedFaceBuffer;
        std::stack<std::shared_ptr<SimplexWrap<VERTEXPTR>>> TraverseStack;

        // std::unordered_set<VERTEXPTR> SingularVertices;
        std::unordered_set<int> SingularVertices;

        std::vector<std::shared_ptr<DeferredSimplex<VERTEXPTR>>> ConeFaceBuffer;
        std::vector<std::shared_ptr<SimplexWrap<VERTEXPTR>>> UpdateBuffer;
        std::vector<int> UpdateIndices;
        std::vector<std::shared_ptr<ConnectorList<VERTEXPTR>>> ConnectorTable;

        void Clear()
        {

            // UpdateBuffer.assign(mDimension, std::make_shared<SimplexWrap<VERTEXPTR>>());
            // UpdateIndices.assign(mDimension, -1);

            for (auto i = 0; i < UpdateBuffer.size(); i++)
                UpdateBuffer[i]->Clear();

            for (auto i = 0; i < ConeFaceBuffer.size(); i++)
                ConeFaceBuffer[i]->Clear();

            for (auto i = 0; i < AffectedFaceBuffer.size(); i++)
                AffectedFaceBuffer[i]->Clear();

            for (auto i = 0; i < ConvexSimplexs.size(); i++)
                ConvexSimplexs[i]->Clear();

            for (auto i = 0; i < CONNECTOR_TABLE_SIZE; i++)
                ConnectorTable[i]->Clear();

            for (auto i = 0; i < TraverseStack.size(); i++)
            {
                TraverseStack.top()->Clear();
                TraverseStack.pop();
            }

            TraverseStack = std::stack<std::shared_ptr<SimplexWrap<VERTEXPTR>>>();

            UpdateBuffer.clear();
            UpdateIndices.clear();

            InputVertices.clear();
            MaxDistance = -std::numeric_limits<double>::max();

            ConvexSimplexs.clear();
            AffectedFaceBuffer.clear();
            SingularVertices.clear();
            ConeFaceBuffer.clear();

            ObjManager->Clear();
            UnprocessedFaces->Clear();
            EmptyBuffer->Clear();
            BeyondBuffer->Clear();

            CurrentVertex = -1;
            FurthestVertex = -1;

            ObjManager = nullptr;
            UnprocessedFaces = nullptr;
            EmptyBuffer = nullptr;
            BeyondBuffer = nullptr;
        }

        void AddInput(const std::vector<VERTEXPTR> &input, bool assignIds, bool checkInput)
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