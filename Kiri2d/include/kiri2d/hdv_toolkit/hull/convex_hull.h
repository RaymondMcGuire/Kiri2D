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
        const std::vector<VERTEX> &FindExtremes()
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
                    auto v = mBuffer->InputVertices[j].GetPosition()[i];

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
        const std::vector<VERTEX> &FindInitialPoints(const std::vector<VERTEX> &extremes)
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

                    temp = MathHelper<VERTEX>.SubtractFast(a->GetPosition(), b->GetPosition());

                    auto dist = MathHelper<VERTEX>.LengthSquared(temp);

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
                newFace.Vertices = vertices;

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
            auto vertices = face.Vertices;
            auto normal = face.Normal;
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
                centerDistance += n * Centroid[i];
            }

            face.Offset = -offset;
            centerDistance -= offset;

            if (centerDistance > 0)
            {
                for (auto i = 0; i < mDimension; i++)
                    normal[i] = -normal[i];

                face.Offset = offset;
                face.IsNormalFlipped = true;
            }
            else
                face.IsNormalFlipped = false;

            return true;
        }

        /// <summary>
        /// Check if 2 faces are adjacent and if so, update their AdjacentFaces array.
        /// </summary>
        void UpdateAdjacency(const std::shared_ptr<SimplexWrap<VERTEX>> &l, const std::shared_ptr<SimplexWrap<VERTEX>> &r)
        {
            auto lv = l.Vertices;
            auto rv = r.Vertices;
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
            l.AdjacentFaces[i] = r;

            // update the adj. face on the other face - find the vertex that remains marked
            for (i = 0; i < mDimension; i++)
                lv[i]->SetTag(0);
            for (i = 0; i < mDimension; i++)
            {
                if (rv[i]->GetTag() == 1)
                    break;
            }
            r->GetAdjacentFaces()[i] = l;
        }

        /// <summary>
        /// Used in the "initialization" code.
        /// </summary>

        void FindBeyondVertices(const std::shared_ptr<SimplexWrap<VERTEX>> face)
        {
            auto beyondVertices = face->GetVerticesBeyond();

            mBuffer->MaxDistance = -std::numeric_limits<float>::infinity();
            mBuffer->FurthestVertex = nullptr;

            auto count = mBuffer->InputVertices.size();

            for (auto i = 0; i < count; i++)
                IsBeyond(face, beyondVertices, mBuffer->InputVertices[i]);

            face.FurthestVertex = mBuffer->FurthestVertex;
        }

#pragma endregion Initilization

    private:
        const float PLANE_DISTANCE_TOLERANCE = 1e-7f;
        int mDimension;

        std::vector<VERTEX> mVertices;
        std::vector<std::shared_ptr<HDV::Primitives::Simplex<VERTEX>>> mSimplexs;
        std::vector<float> mCentroid;
        std::shared_ptr<ObjectBuffer<VERTEX>> mBuffer;
    };

} // namespace HDV::Hull

#endif /* _HDV_CONVEX_HULL_H_ */