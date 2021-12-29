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

#include <kiri2d/hdv_toolkit/primitives/vertex2.h>
#include <kiri2d/hdv_toolkit/primitives/vertex3.h>
#include <kiri2d/hdv_toolkit/primitives/vertex4.h>
#include <kiri2d/hdv_toolkit/primitives/simplex.h>
#include <kiri2d/hdv_toolkit/hull/math_helper.h>
#include <kiri2d/hdv_toolkit/hull/object_buffer.h>
namespace HDV::Hull
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr>
    class ConvexHull
    {
    public:
        explicit ConvexHull() {}
        explicit ConvexHull(int dimension)
        {
            mDimension = dimension;
            mCentroid.assign(dimension, 0.0);
        }

        virtual ~ConvexHull() noexcept {}

        void Clear()
        {
            for (size_t i = 0; i < mSimplexs.size(); i++)
                mSimplexs[i]->Clear();

            mCentroid.assign(mDimension, 0.0);
            mSimplexs.clear();
            mVertices.clear();
        }

        bool Contains(const VERTEXPTR &vertex)
        {
            auto count = mSimplexs.size();
            for (auto i = 0; i < count; i++)
            {
                if (MathHelper<VERTEXPTR>().GetVertexDistance(vertex, mSimplexs[i]) >= PLANE_DISTANCE_TOLERANCE)
                    return false;
            }

            return true;
        }

        std::vector<Vector4F> GetSortSimplexsList()
        {
            std::vector<Vector4F> pre_simplexs, unprocessed_simplexs, simplexs;
            for (auto i = 0; i < mSimplexs.size(); i++)
            {
                auto from_i = mSimplexs[i]->Vertices[0];
                auto to_i = mSimplexs[i]->Vertices[1];
                pre_simplexs.emplace_back(Vector4F(from_i->X(), from_i->Y(), to_i->X(), to_i->Y()));
            }

            std::vector<int> remove_idx;
            // preprocess
            for (auto i = 0; i < pre_simplexs.size() - 1; i++)
            {
                auto usi = pre_simplexs[i];
                for (auto j = i + 1; j < pre_simplexs.size(); j++)
                {
                    auto usj = pre_simplexs[j];
                    if (usi.x == usj.z && usi.y == usj.w && usi.z == usj.x && usi.w == usj.y)
                    {
                        remove_idx.emplace_back(i);
                        break;
                    }
                }
            }

            for (auto i = 0; i < pre_simplexs.size(); i++)
            {
                auto flag = true;
                for (auto j = 0; j < remove_idx.size(); j++)
                {
                    if (i == remove_idx[j])
                    {
                        flag = false;
                        break;
                    }
                }

                if (flag)
                    unprocessed_simplexs.emplace_back(pre_simplexs[i]);
            }

            // remove idx

            auto current = unprocessed_simplexs.back();
            simplexs.emplace_back(current);
            unprocessed_simplexs.pop_back();

            while (!unprocessed_simplexs.empty())
            {
                auto back = unprocessed_simplexs.back();

                for (auto j = 0; j < unprocessed_simplexs.size(); j++)
                {
                    auto simplex = unprocessed_simplexs[j];
                    if (current.z == simplex.x && current.w == simplex.y)
                    {
                        unprocessed_simplexs[unprocessed_simplexs.size() - 1] = Vector4F(simplex.x, simplex.y, simplex.z, simplex.w);
                        if (j != unprocessed_simplexs.size() - 1)
                            unprocessed_simplexs[j] = back;
                        break;
                    }

                    if (current.z == simplex.z && current.w == simplex.w)
                    {
                        unprocessed_simplexs[unprocessed_simplexs.size() - 1] = Vector4F(simplex.z, simplex.w, simplex.x, simplex.y);
                        if (j != unprocessed_simplexs.size() - 1)
                            unprocessed_simplexs[j] = back;
                        break;
                    }
                }

                current = unprocessed_simplexs.back();
                simplexs.emplace_back(current);
                unprocessed_simplexs.pop_back();
            }

            return simplexs;
        }

        void Generate(const std::vector<VERTEXPTR> &input, bool assignIds = true, bool checkInput = false)
        {

            Clear();

            mBuffer = std::make_shared<ObjectBuffer<VERTEXPTR>>(mDimension);

            auto inputCount = input.size();
            if (inputCount < mDimension + 1)
                return;

            mBuffer->AddInput(input, assignIds, checkInput);

            InitConvexHull();

            // Expand the convex hull and faces.
            while (mBuffer->UnprocessedFaces->First != nullptr)
            {
                mBuffer->CurrentVertex = mBuffer->UnprocessedFaces->First->FurthestVertex;

                UpdateCenter();

                // KIRI_LOG_DEBUG("centroid={0},{1}", mCentroid[0], mCentroid[1]);

                // The affected faces get tagged
                TagAffectedFaces(mBuffer->UnprocessedFaces->First);

                // mBuffer->UnprocessedFaces->First->ToString();

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
                mBuffer->ConvexSimplexs[i]->SetTag(i);
                mSimplexs.emplace_back(std::make_shared<HDV::Primitives::Simplex<VERTEXPTR>>(i, mDimension));
            }

            for (auto i = 0; i < mBuffer->ConvexSimplexs.size(); i++)
            {

                auto wrap = mBuffer->ConvexSimplexs[i];

                mSimplexs[i]->SetNormalFlipped(wrap->IsNormalFlipped);
                mSimplexs[i]->SetOffset(wrap->Offset);

                for (auto j = 0; j < mDimension; j++)
                {
                    mSimplexs[i]->Normals[j] = wrap->Normals[j];
                    mSimplexs[i]->Vertices[j] = wrap->Vertices[j];

                    if (wrap->AdjacentFaces[j] != nullptr)
                        mSimplexs[i]->Adjacent[j] = mSimplexs[wrap->AdjacentFaces[j]->GetTag()];
                    else
                        mSimplexs[i]->Adjacent[j] = nullptr;
                }

                mSimplexs[i]->CalculateCentroid();
            }

            // release memory
            mBuffer->Clear();
            mBuffer = nullptr;
        }

        /// <summary>
        /// Check whether the vertex v is beyond the given face. If so, add it to beyondVertices.
        /// </summary>
        void IsBeyond(const std::shared_ptr<SimplexWrap<VERTEXPTR>> &face, const std::shared_ptr<VertexBuffer<VERTEXPTR>> &beyondVertices, const VERTEXPTR &v)
        {
            // KIRI_LOG_DEBUG("-------GetVertexDistance Start-------");
            auto distance = MathHelper<VERTEXPTR>().GetVertexDistance(v, face);
            // KIRI_LOG_DEBUG("-------distance={0}, mBuffer->MaxDistance={1}", distance, mBuffer->MaxDistance);

            if (distance >= PLANE_DISTANCE_TOLERANCE)
            {
                if (distance > mBuffer->MaxDistance)
                {
                    mBuffer->MaxDistance = distance;
                    mBuffer->FurthestVertex = v;
                    // mBuffer->FurthestVertex->ToString();
                    //  KIRI_LOG_DEBUG("-------IsBeyond Finish-------");
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

            double f = 1.f / count;

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
        std::vector<VERTEXPTR> FindExtremes()
        {
            std::vector<VERTEXPTR> extremes;
            // extremes.assign(2 * mDimension, VERTEXPTR());

            auto vCount = mBuffer->InputVertices.size();
            // KIRI_LOG_DEBUG("vCOunt={0}", vCount);

            for (auto i = 0; i < mDimension; i++)
            {
                auto min = std::numeric_limits<double>::max();
                auto max = std::numeric_limits<double>::lowest();
                // KIRI_LOG_DEBUG(max);
                auto minInd = 0, maxInd = 0;

                for (auto j = 0; j < vCount; j++)
                {
                    auto v = mBuffer->InputVertices[j]->GetPosition()[i];

                    if (v < min)
                    {
                        min = v;
                        minInd = j;
                    }

                    // KIRI_LOG_DEBUG("v={0},max={1},v>max={2}", v, max, v > max);
                    if (v > max)
                    {
                        max = v;
                        maxInd = j;
                    }
                }

                // KIRI_LOG_DEBUG("min={0}, max={1}, minidx={2}, maxidx={3}", min, max, minInd, maxInd);

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
        double GetSquaredDistanceSum(const VERTEXPTR &pivot, const std::vector<VERTEXPTR> &initialPoints)
        {
            auto initPtsNum = initialPoints.size();
            auto sum = 0.0;

            for (auto i = 0; i < initPtsNum; i++)
            {
                auto initPt = initialPoints[i];
                if (initPt == nullptr)
                    continue;

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
        std::vector<VERTEXPTR> FindInitialPoints(const std::vector<VERTEXPTR> &extremes)
        {
            std::vector<VERTEXPTR> initialPoints;

            VERTEXPTR first = nullptr, second = nullptr;
            auto maxDist = 0.0;

            std::vector<double> temp;

            for (auto i = 0; i < extremes.size() - 1; i++)
            {
                auto a = extremes[i];
                for (auto j = i + 1; j < extremes.size(); j++)
                {
                    auto b = extremes[j];

                    temp = MathHelper<VERTEXPTR>().SubtractFast(a->GetPosition(), b->GetPosition());

                    auto dist = MathHelper<VERTEXPTR>().LengthSquared(temp);

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
                VERTEXPTR maxPoint = nullptr;

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
        std::vector<std::shared_ptr<SimplexWrap<VERTEXPTR>>> InitiateFaceDatabase()
        {
            std::vector<std::shared_ptr<SimplexWrap<VERTEXPTR>>> faces;
            faces.assign(mDimension + 1, std::make_shared<SimplexWrap<VERTEXPTR>>());

            for (auto i = 0; i < mDimension + 1; i++)
            {
                // Skips the i-th vertex
                std::vector<VERTEXPTR> vertices;
                std::copy(mVertices.begin(), mVertices.end(), std::back_inserter(vertices));
                vertices.erase(vertices.begin() + i);

                std::sort(vertices.begin(), vertices.end(), [](const VERTEXPTR &lhs, const VERTEXPTR &rhs)
                          { return lhs->GetId() < rhs->GetId(); });

                auto newFace = std::make_shared<SimplexWrap<VERTEXPTR>>(mDimension, std::make_shared<VertexBuffer<VERTEXPTR>>());
                newFace->Vertices = vertices;

                // KIRI_LOG_DEBUG("nfv data={0},{1}; face vert data={2},{3}", vertices[0], vertices[1], newFace->Vertices[0], newFace->Vertices[1]);

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
        bool CalculateFacePlane(const std::shared_ptr<SimplexWrap<VERTEXPTR>> &face)
        {
            auto vertices = face->Vertices;
            auto normal = face->Normals;

            MathHelper<VERTEXPTR>().FindNormalVector(vertices, normal);

            face->Normals = normal;

            if (normal[0] != normal[0])
            {
                return false;
            }

            auto offset = 0.0;
            auto centerDistance = 0.0;
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
                    face->Normals[i] = -face->Normals[i];

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
        void UpdateAdjacency(const std::shared_ptr<SimplexWrap<VERTEXPTR>> &l, const std::shared_ptr<SimplexWrap<VERTEXPTR>> &r)
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
        void FindBeyondVertices(const std::shared_ptr<SimplexWrap<VERTEXPTR>> &face)
        {
            mBuffer->MaxDistance = std::numeric_limits<double>::lowest();
            mBuffer->FurthestVertex = nullptr;

            auto count = mBuffer->InputVertices.size();

            for (auto i = 0; i < count; i++)
                IsBeyond(face, face->VerticesBeyond, mBuffer->InputVertices[i]);

            face->FurthestVertex = mBuffer->FurthestVertex;

            // if (face->FurthestVertex != nullptr)
            //     face->FurthestVertex->ToString();
        }

        /// <summary>
        /// Find the (dimension+1) initial points and create the simplexes.
        /// </summary>
        void InitConvexHull()
        {
            auto extremes = FindExtremes();

            // for (size_t i = 0; i < extremes.size(); i++)
            // {
            //     KIRI_LOG_DEBUG("data={0}", extremes[i]->GetString());
            // }
            // KIRI_LOG_DEBUG("---extremes.size={0}", extremes.size());

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
                                                            [=](const VERTEXPTR &v)
                                                            { return (initialPoints[i]->GetId() == v->GetId()); }),
                                             mBuffer->InputVertices.end());

                // Because of the AklTou heuristic.
                //! TODO need check
                extremes.erase(std::remove_if(extremes.begin(), extremes.end(),
                                              [=](const VERTEXPTR &v)
                                              { return (initialPoints[i]->GetId() == v->GetId()); }),
                               extremes.end());
                // extremes.remove(initialPoints[i]);
            }

            // Create the initial simplexes.
            auto faces = InitiateFaceDatabase();

            auto numFaces = faces.size();

            // KIRI_LOG_DEBUG("numFaces={0}", numFaces);

            // Init the vertex beyond buffers.
            for (auto i = 0; i < numFaces; i++)
            {
                FindBeyondVertices(faces[i]);

                // KIRI_LOG_DEBUG("faces[i]->VerticesBeyond->GetCount()={0}", faces[i]->VerticesBeyond->GetCount());
                if (faces[i]->VerticesBeyond->GetCount() == 0)
                    mBuffer->ConvexSimplexs.emplace_back(faces[i]); // The face is on the hull
                else
                    mBuffer->UnprocessedFaces->Add(faces[i]);

                // faces[i]->ToString();
            }
        }

#pragma endregion Initilization

#pragma region Process

        /// <summary>
        /// Tags all faces seen from the current vertex with 1.
        /// </summary>
        void TagAffectedFaces(const std::shared_ptr<SimplexWrap<VERTEXPTR>> &currentFace)
        {
            mBuffer->AffectedFaceBuffer.clear();
            mBuffer->AffectedFaceBuffer.emplace_back(currentFace);
            TraverseAffectedFaces(currentFace);
        }

        /// <summary>
        /// Recursively traverse all the relevant faces.
        /// </summary>
        void TraverseAffectedFaces(const std::shared_ptr<SimplexWrap<VERTEXPTR>> &currentFace)
        {

            mBuffer->TraverseStack = std::stack<std::shared_ptr<SimplexWrap<VERTEXPTR>>>();
            mBuffer->TraverseStack.push(currentFace);
            currentFace->SetTag(1);

            // KIRI_LOG_DEBUG("travse top tag={0}", mBuffer->TraverseStack.top()->GetTag());

            while (mBuffer->TraverseStack.size() > 0)
            {
                auto top = mBuffer->TraverseStack.top();
                mBuffer->TraverseStack.pop();

                for (auto i = 0; i < mDimension; i++)
                {
                    auto adjFace = top->AdjacentFaces[i];
                    // adjFace->ToString();

                    if (adjFace == nullptr)
                        throw std::invalid_argument("(2) Adjacent Face should never be nullptr");

                    if (adjFace->GetTag() == 0 && MathHelper<VERTEXPTR>().GetVertexDistance(mBuffer->CurrentVertex, adjFace) >= PLANE_DISTANCE_TOLERANCE)
                    {
                        mBuffer->AffectedFaceBuffer.emplace_back(top->AdjacentFaces[i]);
                        top->AdjacentFaces[i]->SetTag(1);
                        mBuffer->TraverseStack.push(top->AdjacentFaces[i]);
                    }
                }

                // KIRI_LOG_DEBUG("adjFace-----: traverse size={0}", mBuffer->TraverseStack.size());
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
                        mBuffer->UpdateBuffer[updateCount] = mBuffer->AffectedFaceBuffer[fIndex]->AdjacentFaces[i];
                        mBuffer->UpdateIndices[updateCount] = i;
                        ++updateCount;
                    }
                }

                for (auto i = 0; i < updateCount; i++)
                {
                    // auto adjacentFace = mBuffer->UpdateBuffer[i];

                    auto oldFaceAdjacentIndex = 0;
                    // auto adjFaceAdjacency = adjacentFace->AdjacentFaces;

                    for (auto j = 0; j < mDimension; j++)
                    {
                        //! TODO
                        if (mBuffer->AffectedFaceBuffer[fIndex] == mBuffer->UpdateBuffer[i]->AdjacentFaces[j])
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
                        vertices[j] = mBuffer->AffectedFaceBuffer[fIndex]->Vertices[j];

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
                    newFace->Vertices = vertices;

                    if (!CalculateFacePlane(newFace))
                    {
                        return false;
                    }

                    mBuffer->ConeFaceBuffer.emplace_back(MakeDeferredFace(newFace, orderedPivotIndex, mBuffer->UpdateBuffer[i], oldFaceAdjacentIndex, mBuffer->AffectedFaceBuffer[fIndex]));
                }
            }

            return true;
        }

        /// <summary>
        /// Creates a new deferred face.
        /// </summary>

        std::shared_ptr<DeferredSimplex<VERTEXPTR>> MakeDeferredFace(
            const std::shared_ptr<SimplexWrap<VERTEXPTR>> &face,
            int faceIndex,
            const std::shared_ptr<SimplexWrap<VERTEXPTR>> &pivot,
            int pivotIndex,
            const std::shared_ptr<SimplexWrap<VERTEXPTR>> &oldFace)
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

            //  Add the current vertex.
            mVertices.emplace_back(mBuffer->CurrentVertex);

            // Fill the adjacency.
            for (auto i = 0; i < mBuffer->ConeFaceBuffer.size(); i++)
            {
                // auto face = mBuffer->ConeFaceBuffer[i];

                // auto newFace = face->Face;
                // auto adjacentFace = face->Pivot;
                // auto oldFace = face->OldFace;
                auto orderedPivotIndex = mBuffer->ConeFaceBuffer[i]->FaceIndex;

                mBuffer->ConeFaceBuffer[i]->Face->AdjacentFaces[orderedPivotIndex] = mBuffer->ConeFaceBuffer[i]->Pivot;

                mBuffer->ConeFaceBuffer[i]->Pivot->AdjacentFaces[mBuffer->ConeFaceBuffer[i]->PivotIndex] = mBuffer->ConeFaceBuffer[i]->Face;

                // KIRI_LOG_DEBUG("---Face->AdjacentFaces[orderedPivotIndex]---");
                // mBuffer->ConeFaceBuffer[i]->Face->AdjacentFaces[orderedPivotIndex]->ToString();

                // KIRI_LOG_DEBUG("---orderedPivotIndex={0}---", orderedPivotIndex);
                //  let there be a connection.
                for (auto j = 0; j < mDimension; j++)
                {
                    if (j == orderedPivotIndex)
                        continue;
                    auto connector = mBuffer->ObjManager->GetConnector();
                    connector->Update(mBuffer->ConeFaceBuffer[i]->Face, j, mDimension);
                    // connector->ToString();

                    ConnectFace(connector);
                }

                // KIRI_LOG_DEBUG("(1)newFace->VerticesBeyond->GetCount()={0}", mBuffer->ConeFaceBuffer[i]->Face->VerticesBeyond->GetCount());
                //  KIRI_LOG_DEBUG("adjacentFace->VerticesBeyond->GetCount() < oldFace->VerticesBeyond->GetCount()={0}", adjacentFace->VerticesBeyond->GetCount() < oldFace->VerticesBeyond->GetCount());
                //   This could slightly help...
                if (mBuffer->ConeFaceBuffer[i]->Pivot->VerticesBeyond->GetCount() < mBuffer->ConeFaceBuffer[i]->OldFace->VerticesBeyond->GetCount())
                {
                    FindBeyondVertices(mBuffer->ConeFaceBuffer[i]->Face, mBuffer->ConeFaceBuffer[i]->Pivot->VerticesBeyond, mBuffer->ConeFaceBuffer[i]->OldFace->VerticesBeyond);
                }
                else
                {
                    FindBeyondVertices(mBuffer->ConeFaceBuffer[i]->Face, mBuffer->ConeFaceBuffer[i]->OldFace->VerticesBeyond, mBuffer->ConeFaceBuffer[i]->Pivot->VerticesBeyond);
                }

                // KIRI_LOG_DEBUG("(xxx)newFace->VerticesBeyond->GetCount()={0}", mBuffer->ConeFaceBuffer[i]->Face->VerticesBeyond->GetCount());

                // mBuffer->UnprocessedFaces->ToString();

                // This face will definitely lie on the hull
                if (mBuffer->ConeFaceBuffer[i]->Face->VerticesBeyond->GetCount() == 0)
                {
                    mBuffer->ConvexSimplexs.emplace_back(mBuffer->ConeFaceBuffer[i]->Face);
                    mBuffer->UnprocessedFaces->Remove(mBuffer->ConeFaceBuffer[i]->Face);
                    mBuffer->ObjManager->DepositVertexBuffer(mBuffer->ConeFaceBuffer[i]->Face->VerticesBeyond);

                    mBuffer->ConeFaceBuffer[i]->Face->VerticesBeyond = mBuffer->EmptyBuffer;
                }
                else // Add the face to the list
                {
                    // KIRI_LOG_DEBUG("unprocessed face add!!!!!");
                    // mBuffer->ConeFaceBuffer[i]->Face->ToString();
                    mBuffer->UnprocessedFaces->Add(mBuffer->ConeFaceBuffer[i]->Face);

                    // mBuffer->UnprocessedFaces->ToString();
                }

                // recycle the object.
                mBuffer->ObjManager->DepositDeferredSimplex(mBuffer->ConeFaceBuffer[i]);
            }

            // Recycle the affected faces.
            for (auto fIndex = 0; fIndex < mBuffer->AffectedFaceBuffer.size(); fIndex++)
            {
                mBuffer->UnprocessedFaces->Remove(mBuffer->AffectedFaceBuffer[fIndex]);
                mBuffer->ObjManager->DepositFace(mBuffer->AffectedFaceBuffer[fIndex]);
            }
        }

        /// <summary>
        /// Connect faces using a connector->
        /// </summary>
        void ConnectFace(const std::shared_ptr<SimplexConnector<VERTEXPTR>> &connector)
        {
            auto index = connector->GetHashCode() % mBuffer->CONNECTOR_TABLE_SIZE;

            for (auto current = mBuffer->ConnectorTable[index]->First; current != nullptr; current = current->Next)
            {
                if (SimplexConnector<VERTEXPTR>().AreConnectable(connector, current, mDimension))
                {
                    mBuffer->ConnectorTable[index]->Remove(current);
                    SimplexConnector<VERTEXPTR>().Connect(current, connector);

                    mBuffer->ObjManager->DepositConnector(current);
                    mBuffer->ObjManager->DepositConnector(connector);
                    return;
                }
            }

            mBuffer->ConnectorTable[index]->Add(connector);
        }

        /// <summary>
        /// Used by update faces.
        /// </summary>
        void FindBeyondVertices(
            const std::shared_ptr<SimplexWrap<VERTEXPTR>> &face,
            const std::shared_ptr<VertexBuffer<VERTEXPTR>> &beyond,
            const std::shared_ptr<VertexBuffer<VERTEXPTR>> &beyond1)
        {
            auto beyondVertices = mBuffer->BeyondBuffer;

            mBuffer->MaxDistance = std::numeric_limits<double>::lowest();
            mBuffer->FurthestVertex = nullptr;

            VERTEXPTR v;

            auto count = beyond1->GetCount();

            for (auto i = 0; i < count; i++)
                beyond1->GetItem(i)->SetTag(1);

            mBuffer->CurrentVertex->SetTag(0);

            count = beyond->GetCount();
            for (auto i = 0; i < count; i++)
            {
                v = beyond->GetItem(i);

                //! TODO
                if (v->GetId() == mBuffer->CurrentVertex->GetId())
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
            // KIRI_LOG_DEBUG("---HandleSingular--");

            RollbackCenter();
            mBuffer->SingularVertices.insert(mBuffer->CurrentVertex);

            // This means that all the affected faces must be on the hull and that all their "vertices beyond" are singular.
            for (auto fIndex = 0; fIndex < mBuffer->AffectedFaceBuffer.size(); fIndex++)
            {
                for (auto i = 0; i < mBuffer->AffectedFaceBuffer[fIndex]->VerticesBeyond->GetCount(); i++)
                {
                    mBuffer->SingularVertices.insert(mBuffer->AffectedFaceBuffer[fIndex]->VerticesBeyond->GetItem(i));
                }

                mBuffer->ConvexSimplexs.emplace_back(mBuffer->AffectedFaceBuffer[fIndex]);
                mBuffer->UnprocessedFaces->Remove(mBuffer->AffectedFaceBuffer[fIndex]);
                mBuffer->ObjManager->DepositVertexBuffer(mBuffer->AffectedFaceBuffer[fIndex]->VerticesBeyond);

                mBuffer->AffectedFaceBuffer[fIndex]->VerticesBeyond = mBuffer->EmptyBuffer;
            }
        }

#pragma endregion Process

        std::vector<std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>>> GetSimplexs()
        {
            return mSimplexs;
        }

        std::vector<VERTEXPTR> GetVertices()
        {
            return mVertices;
        }

        std::vector<double> GetCentroid()
        {
            return mCentroid;
        }

    private:
        const double PLANE_DISTANCE_TOLERANCE = 1e-10;
        int mDimension;

        std::vector<VERTEXPTR> mVertices;
        std::vector<std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>>> mSimplexs;
        std::vector<double> mCentroid;
        std::shared_ptr<ObjectBuffer<VERTEXPTR>> mBuffer;
    };

    class ConvexHull2 : public ConvexHull<HDV::Primitives::Vertex2Ptr>
    {
    public:
        explicit ConvexHull2() : ConvexHull(2) {}

        ~ConvexHull2() noexcept {}
    };

    class ConvexHull3 : public ConvexHull<HDV::Primitives::Vertex3Ptr>
    {
    public:
        explicit ConvexHull3() : ConvexHull(3) {}

        ~ConvexHull3() noexcept {}
    };
} // namespace HDV::Hull

#endif /* _HDV_CONVEX_HULL_H_ */