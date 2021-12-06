/***
 * @Author: Xu.WANG
 * @Date: 2021-12-05 19:44:04
 * @LastEditTime: 2021-12-05 19:47:34
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_CONVEX_HULL_H_
#define _HDV_CONVEX_HULL_H_

#pragma once

#include<kiri2d/hdv_toolkit/primitives/vertex2.h>
#include <kiri2d/hdv_toolkit/primitives/simplex.h>
#include <kiri2d/hdv_toolkit/hull/math_helper.h>
#include <kiri2d/hdv_toolkit/hull/object_buffer.h>
namespace HDV::Hull
{
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class ConvexHull
    {
    public:
        explicit ConvexHull() {}
        explicit ConvexHull(int dimension)
        {
            mDimension = dimension;
            mCentroid.assign(dimension, 0.f);
        }

        virtual ~ConvexHull() noexcept {}

        void Clear()
        {
            mCentroid.assign(mDimension, 0.f);
            mSimplexs.clear();
            mVertices.clear();
        }

        bool Contains(VERTEX vertex)
        {
            auto count = mSimplexs.size();
            for (auto i = 0; i < count; i++)
            {
                if (MathHelper<VERTEX>().GetVertexDistance(vertex, mSimplexs[i]) >= PLANE_DISTANCE_TOLERANCE)
                    return false;
            }

            return true;
        }

        void Generate(const std::vector<VERTEX> &input, bool assignIds = true, bool checkInput = false)
        {

            Clear();

            mBuffer = std::make_shared<ObjectBuffer<VERTEX>>(mDimension);

            auto inputCount = input.size();
            if (inputCount < mDimension + 1)
                return;

            mBuffer->AddInput(input, assignIds, checkInput);

            InitConvexHull();

            // Expand the convex hull and faces.
            while (mBuffer->UnprocessedFaces->First != nullptr)
            {
                auto currentFace = mBuffer->UnprocessedFaces->First;
                mBuffer->CurrentVertex = currentFace->FurthestVertex;

                UpdateCenter();

                // The affected faces get tagged
                TagAffectedFaces(currentFace);

                // Create the cone from the currentVertex and the affected faces horizon.
                if ((mBuffer->SingularVertices.find(mBuffer->CurrentVertex) == mBuffer->SingularVertices.end()) && CreateCone())
                    CommitCone();
                else
                    HandleSingular();

                // Need to reset the tags
                auto count = mBuffer->AffectedFaceBuffer.size();
                for (auto i = 0; i < count; i++)
                    mBuffer->AffectedFaceBuffer[i]->SetTag(0);
            }

            for (auto i = 0; i < mBuffer->ConvexSimplexs.size(); i++)
            {
                auto wrap = mBuffer->ConvexSimplexs[i];
                wrap->SetTag(i);

                mSimplexs.emplace_back(std::make_shared<HDV::Primitives::Simplex<VERTEX>>(i, mDimension));
            }

            for (auto i = 0; i < mBuffer->ConvexSimplexs.size(); i++)
            {

                auto wrap = mBuffer->ConvexSimplexs[i];
                auto simplex = mSimplexs[i];

                simplex->SetNormalFlipped(wrap->IsNormalFlipped);
                simplex->SetOffset(wrap->Offset);

                for (auto j = 0; j < mDimension; j++)
                {
                    auto normals = simplex->GetNormals();
                    auto vertices = simplex->GetVertices();
                    normals[j] = wrap->Normals[j];
                    vertices[j] = wrap->Vertices[j];

                    auto adjacent_faces = simplex->GetAdjacent();
                    if (wrap->AdjacentFaces[j] != nullptr)
                        adjacent_faces[j] = mSimplexs[wrap->AdjacentFaces[j]->GetTag()];
                    else
                        adjacent_faces[j] = nullptr;
                }

                simplex->CalculateCentroid();
            }

            mBuffer->Clear();
            mBuffer = nullptr;
        }

        /// <summary>
        /// Check whether the vertex v is beyond the given face. If so, add it to beyondVertices.
        /// </summary>
        void IsBeyond(const std::shared_ptr<SimplexWrap<VERTEX>> &face, const std::shared_ptr<VertexBuffer<VERTEX>> &beyondVertices, const VERTEX &v)
        {
            auto distance = MathHelper<VERTEX>().GetVertexDistance(v, face);

            if (distance >= PLANE_DISTANCE_TOLERANCE)
            {
                if (distance > mBuffer->MaxDistance)
                {
                    mBuffer->MaxDistance = distance;
                    mBuffer->FurthestVertex = v;
                }
                beyondVertices->Add(v);
            }
        }

        /// <summary>
        /// Recalculates the centroid of the current hull.
        /// </summary>
        void UpdateCenter()
        {
            auto count = mVertices.size() + 1;

            for (auto i = 0; i < mDimension; i++)
                mCentroid[i] *= (count - 1);

            float f = 1.f / count;

            for (auto i = 0; i < mDimension; i++)
                mCentroid[i] = f * (mCentroid[i] + mBuffer->CurrentVertex->GetPosition()[i]);
        }

        /// <summary>
        /// Removes the last vertex from the center.
        /// </summary>
        void RollbackCenter()
        {
            auto count = mVertices.size() + 1;

            for (auto i = 0; i < mDimension; i++)
                mCentroid[i] *= count;

            auto f = 1.f / (count - 1);

            for (auto i = 0; i < mDimension; i++)
                mCentroid[i] = f * (mCentroid[i] - mBuffer->CurrentVertex->GetPosition()[i]);
        }

#pragma region Initilization

        /// <summary>
        /// Finds the extremes in all dimensions.
        /// </summary>
        std::vector<VERTEX> FindExtremes()
        {
            std::vector<VERTEX> extremes;
            extremes.assign(2 * mDimension, VERTEX());

            auto vCount = mBuffer->InputVertices.size();
            for (auto i = 0; i < mDimension; i++)
            {
                auto min = std::numeric_limits<float>::max(), max = std::numeric_limits<float>::min();
                auto minInd = 0, maxInd = 0;

                for (auto j = 0; j < vCount; j++)
                {
                    auto v = mBuffer->InputVertices[j]->GetPosition()[i];

                    if (v < min)
                    {
                        min = v;
                        minInd = j;
                    }
                    if (v > max)
                    {
                        max = v;
                        maxInd = j;
                    }
                }

                if (minInd != maxInd)
                {
                    extremes.emplace_back(mBuffer->InputVertices[minInd]);
                    extremes.emplace_back(mBuffer->InputVertices[maxInd]);
                }
                else
                    extremes.emplace_back(mBuffer->InputVertices[minInd]);
            }

            return extremes;
        }

        /// <summary>
        /// Computes the sum of square distances to the initial points.
        /// </summary>
        float GetSquaredDistanceSum(const VERTEX &pivot, const std::vector<VERTEX> &initialPoints)
        {
            auto initPtsNum = initialPoints.size();
            auto sum = 0.f;

            for (auto i = 0; i < initPtsNum; i++)
            {
                auto initPt = initialPoints[i];

                for (auto j = 0; j < mDimension; j++)
                {
                    auto t = (initPt->GetPosition()[j] - pivot->GetPosition()[j]);
                    sum += t * t;
                }
            }

            return sum;
        }

        /// <summary>
        /// Finds (dimension + 1) initial points.
        /// </summary>
        std::vector<VERTEX> FindInitialPoints(const std::vector<VERTEX> &extremes)
        {
            std::vector<VERTEX> initialPoints;

            VERTEX first = nullptr, second = nullptr;
            auto maxDist = 0.f;

            std::vector<float> temp;

            for (auto i = 0; i < extremes.size() - 1; i++)
            {
                auto a = extremes[i];
                for (auto j = i + 1; j < extremes.size(); j++)
                {
                    auto b = extremes[j];

                    temp = MathHelper<VERTEX>().SubtractFast(a->GetPosition(), b->GetPosition());

                    auto dist = MathHelper<VERTEX>().LengthSquared(temp);

                    if (dist > maxDist)
                    {
                        first = a;
                        second = b;
                        maxDist = dist;
                    }
                }
            }

            initialPoints.emplace_back(first);
            initialPoints.emplace_back(second);

            for (auto i = 2; i <= mDimension; i++)
            {
                auto maximum = 0.000001f;
                VERTEX maxPoint = nullptr;

                for (auto j = 0; j < extremes.size(); j++)
                {
                    auto extreme = extremes[j];

                    if (std::find(initialPoints.begin(), initialPoints.end(), extreme) != initialPoints.end())
                        continue;

                    auto val = GetSquaredDistanceSum(extreme, initialPoints);

                    if (val > maximum)
                    {
                        maximum = val;
                        maxPoint = extreme;
                    }
                }

                if (maxPoint != nullptr)
                {
                    initialPoints.emplace_back(maxPoint);
                }
                else
                {
                    auto vCount = mBuffer->InputVertices.size();
                    for (auto j = 0; j < vCount; j++)
                    {
                        auto point = mBuffer->InputVertices[j];
                        if (std::find(initialPoints.begin(), initialPoints.end(), point) != initialPoints.end())
                            continue;

                        auto val = GetSquaredDistanceSum(point, initialPoints);

                        if (val > maximum)
                        {
                            maximum = val;
                            maxPoint = point;
                        }
                    }

                    if (maxPoint != nullptr)
                        initialPoints.emplace_back(maxPoint);
                    else
                        throw std::invalid_argument("Singular input data error");
                }
            }

            return initialPoints;
        }

        /// <summary>
        /// Create the first faces from (dimension + 1) vertices.
        /// </summary>
        std::vector<std::shared_ptr<SimplexWrap<VERTEX>>> InitiateFaceDatabase()
        {
            std::vector<std::shared_ptr<SimplexWrap<VERTEX>>> faces;
            faces.assign(mDimension + 1, std::make_shared<SimplexWrap<VERTEX>>());

            for (auto i = 0; i < mDimension + 1; i++)
            {
                // Skips the i-th vertex
                std::vector<VERTEX> vertices;
                std::copy(mVertices.begin(), mVertices.end(), std::back_inserter(vertices));

                vertices.erase(vertices.begin() + i);
                auto newFace = std::make_shared<SimplexWrap<VERTEX>>(mDimension, std::make_shared<VertexBuffer<VERTEX>>());
                newFace->Vertices = vertices;

                std::sort(vertices.begin(), vertices.end(), [](const VERTEX &lhs, const VERTEX &rhs)
                          { return lhs->GetId() < rhs->GetId(); });

                CalculateFacePlane(newFace);
                faces[i] = newFace;
            }

            // update the adjacency (check all pairs of faces)
            for (auto i = 0; i < mDimension; i++)
            {
                for (auto j = i + 1; j < mDimension + 1; j++)
                    UpdateAdjacency(faces[i], faces[j]);
            }

            return faces;
        }

        /// <summary>
        /// Calculates the normal and offset of the hyper-plane given by the face's vertices.
        /// </summary>
        bool CalculateFacePlane(const std::shared_ptr<SimplexWrap<VERTEX>> &face)
        {
            auto vertices = face->Vertices;
            auto normal = face->Normals;
            MathHelper<VERTEX>().FindNormalVector(vertices, normal);

            if (normal[0] != normal[0])
            {
                return false;
            }

            auto offset = 0.f;
            auto centerDistance = 0.f;
            auto fi = vertices[0]->GetPosition();

            for (auto i = 0; i < mDimension; i++)
            {
                auto n = normal[i];
                offset += n * fi[i];
                centerDistance += n * mCentroid[i];
            }

            face->Offset = -offset;
            centerDistance -= offset;

            if (centerDistance > 0)
            {
                for (auto i = 0; i < mDimension; i++)
                    normal[i] = -normal[i];

                face->Offset = offset;
                face->IsNormalFlipped = true;
            }
            else
                face->IsNormalFlipped = false;

            return true;
        }

        /// <summary>
        /// Check if 2 faces are adjacent and if so, update their AdjacentFaces array.
        /// </summary>
        void UpdateAdjacency(const std::shared_ptr<SimplexWrap<VERTEX>> &l, const std::shared_ptr<SimplexWrap<VERTEX>> &r)
        {
            auto lv = l->Vertices;
            auto rv = r->Vertices;
            auto i = 0;

            // reset marks on the 1st face
            for (i = 0; i < mDimension; i++)
                lv[i]->SetTag(0);

            // mark all vertices on the 2nd face
            for (i = 0; i < mDimension; i++)
                rv[i]->SetTag(1);

            // find the 1st false index
            for (i = 0; i < mDimension; i++)
                if (lv[i]->GetTag() == 0)
                    break;

            // no vertex was marked
            if (i == mDimension)
                return;

            // check if only 1 vertex wasn't marked
            for (auto j = i + 1; j < mDimension; j++)
                if (lv[j]->GetTag() == 0)
                    return;

            // if we are here, the two faces share an edge
            l->AdjacentFaces[i] = r;

            // update the adj. face on the other face - find the vertex that remains marked
            for (i = 0; i < mDimension; i++)
                lv[i]->SetTag(0);
            for (i = 0; i < mDimension; i++)
            {
                if (rv[i]->GetTag() == 1)
                    break;
            }
            r->AdjacentFaces[i] = l;
        }

        /// <summary>
        /// Used in the "initialization" code.
        /// </summary>
        void FindBeyondVertices(const std::shared_ptr<SimplexWrap<VERTEX>> &face)
        {
            auto beyondVertices = face->VerticesBeyond;

            mBuffer->MaxDistance = -std::numeric_limits<float>::infinity();
            mBuffer->FurthestVertex = nullptr;

            auto count = mBuffer->InputVertices.size();

            for (auto i = 0; i < count; i++)
                IsBeyond(face, beyondVertices, mBuffer->InputVertices[i]);

            face->FurthestVertex = mBuffer->FurthestVertex;
        }

        /// <summary>
        /// Find the (dimension+1) initial points and create the simplexes.
        /// </summary>
        void InitConvexHull()
        {
            auto extremes = FindExtremes();
            auto initialPoints = FindInitialPoints(extremes);

            auto numPoints = initialPoints.size();

            // Add the initial points to the convex hull.
            for (auto i = 0; i < numPoints; i++)
            {
                mBuffer->CurrentVertex = initialPoints[i];
                // update center must be called before adding the vertex.
                UpdateCenter();
                mVertices.emplace_back(mBuffer->CurrentVertex);

                //! TODO need check
                mBuffer->InputVertices.erase(std::remove_if(mBuffer->InputVertices.begin(), mBuffer->InputVertices.end(),
                                                            [=](const VERTEX &v)
                                                            { return (initialPoints[i]->GetId() == v->GetId()); }),
                                             mBuffer->InputVertices.end());
                // mBuffer->InputVertices.remove(initialPoints[i]);

                // Because of the AklTou heuristic.
                //! TODO need check
                extremes.erase(std::remove_if(extremes.begin(), extremes.end(),
                                              [=](const VERTEX &v)
                                              { return (initialPoints[i]->GetId() == v->GetId()); }),
                               extremes.end());
                // extremes.remove(initialPoints[i]);
            }

            // Create the initial simplexes.
            auto faces = InitiateFaceDatabase();

            auto numFaces = faces.size();

            // Init the vertex beyond buffers.
            for (auto i = 0; i < numFaces; i++)
            {
                FindBeyondVertices(faces[i]);
                if (faces[i]->VerticesBeyond->GetCount() == 0)
                    mBuffer->ConvexSimplexs.emplace_back(faces[i]); // The face is on the hull
                else
                    mBuffer->UnprocessedFaces->Add(faces[i]);
            }
        }

#pragma endregion Initilization

#pragma region Process

        /// <summary>
        /// Tags all faces seen from the current vertex with 1.
        /// </summary>
        void TagAffectedFaces(const std::shared_ptr<SimplexWrap<VERTEX>> &currentFace)
        {
            mBuffer->AffectedFaceBuffer.clear();
            mBuffer->AffectedFaceBuffer.emplace_back(currentFace);
            TraverseAffectedFaces(currentFace);
        }

        /// <summary>
        /// Recursively traverse all the relevant faces.
        /// </summary>
        void TraverseAffectedFaces(const std::shared_ptr<SimplexWrap<VERTEX>> &currentFace)
        {

            mBuffer->TraverseStack = std::stack<std::shared_ptr<SimplexWrap<VERTEX>>>();
            mBuffer->TraverseStack.push(currentFace);
            currentFace->SetTag(1);

            while (mBuffer->TraverseStack.size() > 0)
            {
                auto top = mBuffer->TraverseStack.top();
                mBuffer->TraverseStack.pop();

                for (auto i = 0; i < mDimension; i++)
                {
                    auto adjFace = top->AdjacentFaces[i];

                    if (adjFace == nullptr)
                        throw std::invalid_argument("(2) Adjacent Face should never be nullptr");

                    if (adjFace->GetTag() == 0 && MathHelper<VERTEX>().GetVertexDistance(mBuffer->CurrentVertex, adjFace) >= PLANE_DISTANCE_TOLERANCE)
                    {
                        mBuffer->AffectedFaceBuffer.emplace_back(adjFace);
                        adjFace->SetTag(1);
                        mBuffer->TraverseStack.push(adjFace);
                    }
                }
            }
        }

        /// <summary>
        /// Removes the faces "covered" by the current vertex and adds the newly created ones.
        /// </summary>
        bool CreateCone()
        {
            auto currentVertexIndex = mBuffer->CurrentVertex->GetId();
            mBuffer->ConeFaceBuffer.clear();

            for (auto fIndex = 0; fIndex < mBuffer->AffectedFaceBuffer.size(); fIndex++)
            {
                auto oldFace = mBuffer->AffectedFaceBuffer[fIndex];

                // Find the faces that need to be updated
                auto updateCount = 0;
                for (auto i = 0; i < mDimension; i++)
                {
                    auto af = oldFace->AdjacentFaces[i];

                    if (af == nullptr)
                        throw std::invalid_argument("(3) Adjacent Face should never be nullptr");

                    if (af->GetTag() == 0) // Tag == 0 when oldFaces does not contain af
                    {
                        mBuffer->UpdateBuffer[updateCount] = af;
                        mBuffer->UpdateIndices[updateCount] = i;
                        ++updateCount;
                    }
                }

                for (auto i = 0; i < updateCount; i++)
                {
                    auto adjacentFace = mBuffer->UpdateBuffer[i];

                    auto oldFaceAdjacentIndex = 0;
                    auto adjFaceAdjacency = adjacentFace->AdjacentFaces;

                    for (auto j = 0; j < mDimension; j++)
                    {
                        //! TODO
                        if (oldFace == adjFaceAdjacency[j])
                        {
                            oldFaceAdjacentIndex = j;
                            break;
                        }
                    }

                    // Index of the face that corresponds to this adjacent face
                    auto forbidden = mBuffer->UpdateIndices[i];

                    auto newFace = mBuffer->ObjManager->GetFace();
                    auto vertices = newFace->Vertices;

                    for (auto j = 0; j < mDimension; j++)
                        vertices[j] = oldFace->Vertices[j];

                    auto oldVertexIndex = vertices[forbidden]->GetId();

                    auto orderedPivotIndex = 0;

                    // correct the ordering
                    if (currentVertexIndex < oldVertexIndex)
                    {
                        orderedPivotIndex = 0;
                        for (auto j = forbidden - 1; j >= 0; j--)
                        {
                            if (vertices[j]->GetId() > currentVertexIndex)
                                vertices[j + 1] = vertices[j];
                            else
                            {
                                orderedPivotIndex = j + 1;
                                break;
                            }
                        }
                    }
                    else
                    {
                        orderedPivotIndex = mDimension - 1;
                        for (auto j = forbidden + 1; j < mDimension; j++)
                        {
                            if (vertices[j]->GetId() < currentVertexIndex)
                                vertices[j - 1] = vertices[j];
                            else
                            {
                                orderedPivotIndex = j - 1;
                                break;
                            }
                        }
                    }

                    vertices[orderedPivotIndex] = mBuffer->CurrentVertex;

                    if (!CalculateFacePlane(newFace))
                    {
                        return false;
                    }

                    mBuffer->ConeFaceBuffer.emplace_back(MakeDeferredFace(newFace, orderedPivotIndex, adjacentFace, oldFaceAdjacentIndex, oldFace));
                }
            }

            return true;
        }

        /// <summary>
        /// Creates a new deferred face.
        /// </summary>

        std::shared_ptr<DeferredSimplex<VERTEX>> MakeDeferredFace(
            const std::shared_ptr<SimplexWrap<VERTEX>> &face,
            int faceIndex,
            const std::shared_ptr<SimplexWrap<VERTEX>> &pivot,
            int pivotIndex,
            const std::shared_ptr<SimplexWrap<VERTEX>> &oldFace)
        {
            auto ret = mBuffer->ObjManager->GetDeferredSimplex();

            ret->Face = face;
            ret->FaceIndex = faceIndex;
            ret->Pivot = pivot;
            ret->PivotIndex = pivotIndex;
            ret->OldFace = oldFace;

            return ret;
        }

        /// <summary>
        /// Commits a cone and adds a vertex to the convex hull.
        /// </summary>

        void CommitCone()
        {
            // Add the current vertex.
            mVertices.emplace_back(mBuffer->CurrentVertex);

            // Fill the adjacency.
            for (auto i = 0; i < mBuffer->ConeFaceBuffer.size(); i++)
            {
                auto face = mBuffer->ConeFaceBuffer[i];

                auto newFace = face->Face;
                auto adjacentFace = face->Pivot;
                auto oldFace = face->OldFace;
                auto orderedPivotIndex = face->FaceIndex;

                newFace->AdjacentFaces[orderedPivotIndex] = adjacentFace;
                adjacentFace->AdjacentFaces[face->PivotIndex] = newFace;

                // let there be a connection.
                for (auto j = 0; j < mDimension; j++)
                {
                    if (j == orderedPivotIndex)
                        continue;
                    auto connector = mBuffer->ObjManager->GetConnector();
                    connector->Update(newFace, j, mDimension);
                    ConnectFace(connector);
                }

                // This could slightly help...
                if (adjacentFace->VerticesBeyond->GetCount() < oldFace->VerticesBeyond->GetCount())
                {
                    FindBeyondVertices(newFace, adjacentFace->VerticesBeyond, oldFace->VerticesBeyond);
                }
                else
                {
                    FindBeyondVertices(newFace, oldFace->VerticesBeyond, adjacentFace->VerticesBeyond);
                }

                // This face will definitely lie on the hull
                if (newFace->VerticesBeyond->GetCount() == 0)
                {
                    mBuffer->ConvexSimplexs.emplace_back(newFace);
                    mBuffer->UnprocessedFaces->Remove(newFace);
                    mBuffer->ObjManager->DepositVertexBuffer(newFace->VerticesBeyond);
                    newFace->VerticesBeyond = mBuffer->EmptyBuffer;
                }
                else // Add the face to the list
                {
                    mBuffer->UnprocessedFaces->Add(newFace);
                }

                // recycle the object.
                mBuffer->ObjManager->DepositDeferredSimplex(face);
            }

            // Recycle the affected faces.
            for (auto fIndex = 0; fIndex < mBuffer->AffectedFaceBuffer.size(); fIndex++)
            {
                auto face = mBuffer->AffectedFaceBuffer[fIndex];

                mBuffer->UnprocessedFaces->Remove(face);
                mBuffer->ObjManager->DepositFace(face);
            }
        }

        /// <summary>
        /// Connect faces using a connector->
        /// </summary>
        void ConnectFace(const std::shared_ptr<SimplexConnector<VERTEX>> &connector)
        {
            auto index = connector->GetHashCode() % mBuffer->CONNECTOR_TABLE_SIZE;
            auto list = mBuffer->ConnectorTable[index];

            for (auto current = list->First; current != nullptr; current = current->Next)
            {
                if (SimplexConnector<VERTEX>().AreConnectable(connector, current, mDimension))
                {
                    list->Remove(current);
                    SimplexConnector<VERTEX>().Connect(current, connector);

                    mBuffer->ObjManager->DepositConnector(current);
                    mBuffer->ObjManager->DepositConnector(connector);
                    return;
                }
            }

            list->Add(connector);
        }

        /// <summary>
        /// Used by update faces.
        /// </summary>
        void FindBeyondVertices(
            const std::shared_ptr<SimplexWrap<VERTEX>> &face,
            const std::shared_ptr<VertexBuffer<VERTEX>> &beyond,
            const std::shared_ptr<VertexBuffer<VERTEX>> &beyond1)
        {
            auto beyondVertices = mBuffer->BeyondBuffer;

            mBuffer->MaxDistance = -std::numeric_limits<float>::infinity();
            mBuffer->FurthestVertex = nullptr;

            VERTEX v;

            auto count = beyond1->GetCount();

            for (auto i = 0; i < count; i++)
                beyond1->GetItem(i)->SetTag(1);

            mBuffer->CurrentVertex->SetTag(0);

            count = beyond->GetCount();
            for (auto i = 0; i < count; i++)
            {
                v = beyond->GetItem(i);

                //! TODO
                if (v == mBuffer->CurrentVertex)
                    continue;

                v->SetTag(0);
                IsBeyond(face, beyondVertices, v);
            }

            count = beyond1->GetCount();
            for (auto i = 0; i < count; i++)
            {
                v = beyond1->GetItem(i);
                if (v->GetTag() == 1)
                    IsBeyond(face, beyondVertices, v);
            }

            face->FurthestVertex = mBuffer->FurthestVertex;

            // Pull the old switch a roo
            auto temp = face->VerticesBeyond;
            face->VerticesBeyond = beyondVertices;
            if (temp->GetCount() > 0)
                temp->Clear();
            mBuffer->BeyondBuffer = temp;
        }

        /// <summary>
        /// Handles singular vertex.
        /// </summary>

        void HandleSingular()
        {
            RollbackCenter();
            mBuffer->SingularVertices.insert(mBuffer->CurrentVertex);

            // This means that all the affected faces must be on the hull and that all their "vertices beyond" are singular.
            for (auto fIndex = 0; fIndex < mBuffer->AffectedFaceBuffer.size(); fIndex++)
            {
                auto face = mBuffer->AffectedFaceBuffer[fIndex];
                auto vb = face->VerticesBeyond;
                for (auto i = 0; i < vb->GetCount(); i++)
                {
                    mBuffer->SingularVertices.insert(vb->GetItem(i));
                }

                mBuffer->ConvexSimplexs.emplace_back(face);
                mBuffer->UnprocessedFaces->Remove(face);
                mBuffer->ObjManager->DepositVertexBuffer(face->VerticesBeyond);
                face->VerticesBeyond = mBuffer->EmptyBuffer;
            }
        }

#pragma endregion Process

    private:
        const float PLANE_DISTANCE_TOLERANCE = 1e-7f;
        int mDimension;

        std::vector<VERTEX> mVertices;
        std::vector<std::shared_ptr<HDV::Primitives::Simplex<VERTEX>>> mSimplexs;
        std::vector<float> mCentroid;
        std::shared_ptr<ObjectBuffer<VERTEX>> mBuffer;
    };

    class ConvexHull2 : public ConvexHull<HDV::Primitives::Vertex2Ptr>
    {
    public:
        explicit ConvexHull2() : ConvexHull(2) {}

        ~ConvexHull2() noexcept {}
    };
} // namespace HDV::Hull

#endif /* _HDV_CONVEX_HULL_H_ */