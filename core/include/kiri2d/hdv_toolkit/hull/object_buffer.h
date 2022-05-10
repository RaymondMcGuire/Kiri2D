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

            UpdateBuffer.assign(dimension, std::make_shared<SimplexNode>());
            UpdateIndices.assign(dimension, -1);

            ObjManager = std::make_shared<ObjectManager>(dimension);
            UnprocessedFaces = std::make_shared<SimplexList>();

            ConnectorTable.assign(CONNECTOR_TABLE_SIZE, std::make_shared<ConnectorList>());
        }

        virtual ~ObjectBuffer() {}

        const int CONNECTOR_TABLE_SIZE = 2022;

        int CurrentVertex;
        int FurthestVertex;

        double MaxDistance = -std::numeric_limits<double>::max();

        std::shared_ptr<SimplexList> UnprocessedFaces;
        std::shared_ptr<ObjectManager> ObjManager;

        std::vector<int> EmptyBuffer;
        std::vector<int> BeyondBuffer;

        std::vector<VERTEXPTR> InputVertices;
        std::vector<std::shared_ptr<SimplexNode>> ConvexSimplexs;
        std::vector<std::shared_ptr<SimplexNode>> AffectedFaceBuffer;
        std::stack<std::shared_ptr<SimplexNode>> TraverseStack;

        std::unordered_set<int> SingularVertices;

        std::vector<std::shared_ptr<DeferredSimplex>> ConeFaceBuffer;
        std::vector<std::shared_ptr<SimplexNode>> UpdateBuffer;
        std::vector<int> UpdateIndices;
        std::vector<std::shared_ptr<ConnectorList>> ConnectorTable;

        void clear()
        {
            for (auto i = 0; i < UpdateBuffer.size(); i++)
                UpdateBuffer[i]->clear();

            for (auto i = 0; i < ConeFaceBuffer.size(); i++)
                ConeFaceBuffer[i]->clear();

            for (auto i = 0; i < AffectedFaceBuffer.size(); i++)
                AffectedFaceBuffer[i]->clear();

            for (auto i = 0; i < ConvexSimplexs.size(); i++)
                ConvexSimplexs[i]->clear();

            for (auto i = 0; i < CONNECTOR_TABLE_SIZE; i++)
                ConnectorTable[i]->clear();

            for (auto i = 0; i < TraverseStack.size(); i++)
            {
                TraverseStack.top()->clear();
                TraverseStack.pop();
            }

            TraverseStack = std::stack<std::shared_ptr<SimplexNode>>();

            UpdateBuffer.clear();
            UpdateIndices.clear();

            InputVertices.clear();
            MaxDistance = -std::numeric_limits<double>::max();

            ConvexSimplexs.clear();
            AffectedFaceBuffer.clear();
            SingularVertices.clear();
            ConeFaceBuffer.clear();

            ObjManager->clear();
            UnprocessedFaces->clear();
            EmptyBuffer.clear();
            BeyondBuffer.clear();

            CurrentVertex = -1;
            FurthestVertex = -1;

            ObjManager = nullptr;
            UnprocessedFaces = nullptr;
        }

        void addInput(const std::vector<VERTEXPTR> &input, bool assignIds, bool checkInput)
        {
            auto count = input.size();
            InputVertices = input;

            if (assignIds)
            {
                for (auto i = 0; i < count; i++)
                    InputVertices[i]->setId(i);
            }

            if (checkInput)
            {
                std::unordered_set<int> set;

                for (auto i = 0; i < count; i++)
                {
                    if (input[i] == nullptr)
                        throw std::invalid_argument("Input has a null vertex!");

                    if (input[i]->dimension() != mDimension)
                        throw std::invalid_argument("Input vertex is not the correct dimension!");

                    if (set.count(input[i]->id()))
                        throw std::invalid_argument("Input vertex id is not unique!");
                    else
                        set.insert(input[i]->id());
                }
            }
        }

    private:
        int mDimension = -1;
    };

} // namespace HDV::Hull

#endif /* _HDV_OBJECT_BUFFER_H_ */