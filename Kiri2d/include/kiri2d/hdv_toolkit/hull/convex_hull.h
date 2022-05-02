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

#include <experimental/vector>

#ifdef KIRI_WINDOWS
#include <omp.h>
#endif

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

            mAxisMinma.assign(dimension, 0.0);
            mAxisMaxma.assign(dimension, 0.0);
            mShiftAmount.assign(dimension, 0.0);

            mBoundingBoxPoints.assign(dimension, std::vector<int>());

            mPositions.clear();
        }

        virtual ~ConvexHull() {}

        void SetForPowerDiagram(bool enable)
        {
            mForPowerDiagram = enable;
        }

        double GetCoordinate(int index, int dimension)
        {
            return mPositions[index * mDimension + dimension];
        }

        void Clear()
        {
            for (auto i = 0; i < mSimplexs.size(); i++)
                mSimplexs[i]->Clear();

            mCentroid.assign(mDimension, 0.0);
            mSimplexs.clear();
            mVertices.clear();
            mPositions.clear();
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

            // preprocess
            std::vector<int> remove_idx;
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

        void SerializeVerticesToPositions(const std::vector<VERTEXPTR> &input)
        {
            mInput = input;
            mNumberOfVertices = input.size();
            mVertexVisited.assign(mNumberOfVertices, false);

            //! TODO for power diagram

            for (auto i = 0; i < mNumberOfVertices; i++)
            {
                auto vi = input[i];
                for (auto j = 0; j < vi->GetDimension(); j++)
                    mPositions.emplace_back(vi->mPosition[j]);
            }

            // KIRI_LOG_DEBUG("positions size={0}", mPositions.size());
        }

        void FindBoundingBoxPoints()
        {
            mIndexOfDimensionWithLeastExtremes = -1;
            auto minNumExtremes = std::numeric_limits<int>::max();

            for (auto i = 0; i < mDimension; i++)
            {
                std::vector<int> minIndices;
                std::vector<int> maxIndices;

                auto min = std::numeric_limits<double>::max();
                auto max = std::numeric_limits<double>::lowest();

                for (auto j = 0; j < mNumberOfVertices; j++)
                {
                    auto v = GetCoordinate(j, i);
                    auto difference = min - v;
                    if (difference >= PLANE_DISTANCE_TOLERANCE)
                    {
                        // you found a better solution than before, clear out the list and store new value
                        min = v;
                        minIndices.clear();
                        minIndices.emplace_back(j);
                    }
                    else if (difference > 0)
                    {
                        // you found a solution slightly better than before, clear out those that are no longer on the list and store new value
                        min = v;

                        //! TODO need confirm
                        std::experimental::erase_if(minIndices, [=](int index)
                                                    { return (min - GetCoordinate(index, i)) > PLANE_DISTANCE_TOLERANCE; });

                        minIndices.emplace_back(j);
                    }
                    else if (difference > -PLANE_DISTANCE_TOLERANCE)
                    {
                        // same or almost as good as current limit, so store it
                        minIndices.emplace_back(j);
                    }
                    difference = v - max;
                    if (difference >= PLANE_DISTANCE_TOLERANCE)
                    {
                        // you found a better solution than before, clear out the list and store new value
                        max = v;
                        maxIndices.clear();
                        maxIndices.emplace_back(j);
                    }
                    else if (difference > 0)
                    {
                        // you found a solution slightly better than before, clear out those that are no longer on the list and store new value
                        max = v;

                        //! TODO need confirm
                        std::experimental::erase_if(maxIndices, [=](int index)
                                                    { return (min - GetCoordinate(index, i)) > PLANE_DISTANCE_TOLERANCE; });

                        maxIndices.emplace_back(j);
                    }
                    else if (difference > -PLANE_DISTANCE_TOLERANCE)
                    {
                        // same or almost as good as current limit, so store it
                        maxIndices.emplace_back(j);
                    }
                }

                mAxisMinma[i] = min;
                mAxisMaxma[i] = max;

                minIndices.insert(minIndices.end(), maxIndices.begin(), maxIndices.end());

                if (minIndices.size() < minNumExtremes)
                {
                    minNumExtremes = minIndices.size();
                    mIndexOfDimensionWithLeastExtremes = i;
                }

                mBoundingBoxPoints[i] = minIndices;
            }
        }

        void ShiftAndScalePositions()
        {
            if (mForPowerDiagram)
            {

                auto absSum = [](const std::vector<double> &v) -> double
                {
                    double sum{};
                    for (const auto &x : v)
                        sum += std::abs(x);
                    return sum;
                };

                auto origNumDim = mDimension - 1;
                auto parabolaScale = 2 / (absSum(mAxisMinma) + absSum(mAxisMaxma) - std::abs(mAxisMaxma[origNumDim]) - std::abs(mAxisMinma[origNumDim]));
                // the parabola scale is 1 / average of the sum of the other dimensions.
                // multiplying this by the parabola will scale it back to be on near similar size to the
                // other dimensions. Without this, the term is much larger than the others, which causes
                // problems for roundoff error and finding the normal of faces.
                mAxisMinma[origNumDim] *= parabolaScale; // change the extreme values as well
                mAxisMaxma[origNumDim] *= parabolaScale;
                // it is done here because
                for (int i = origNumDim; i < mPositions.size(); i += mDimension)
                    mPositions[i] *= parabolaScale;
            }

            for (auto i = 0; i < mDimension; i++)
            {
                if (mAxisMinma[i] == mAxisMaxma[i])
                    mShiftAmount[i] = 0.0;
                else
                    mShiftAmount[i] = mAxisMaxma[i] - mAxisMinma[i] - mAxisMinma[i];
            }

            for (auto i = 0; i < mPositions.size(); i++)
                mPositions[i] += mShiftAmount[i % mDimension];
        }

        std::vector<double> VectorBetweenVertices(int toIndex, int fromIndex)
        {
            std::vector<double> target;
            target.assign(mDimension, 0.0);

            int u = toIndex * mDimension, v = fromIndex * mDimension;
            for (auto i = 0; i < mDimension; i++)
            {
                target[i] = mPositions[u + i] - mPositions[v + i];
            }

            return target;
        }

        static void LUFactor(std::vector<double> &data, int order, std::vector<int> &ipiv, std::vector<double> &vecLUcolj)
        {
            // Initialize the pivot matrix to the identity permutation.
            for (auto i = 0; i < order; i++)
            {
                ipiv[i] = i;
            }

            // Outer loop.
            for (auto j = 0; j < order; j++)
            {
                auto indexj = j * order;
                auto indexjj = indexj + j;

                // Make a copy of the j-th column to localize references.
                for (auto i = 0; i < order; i++)
                {
                    vecLUcolj[i] = data[indexj + i];
                }

                // Apply previous transformations.
                for (auto i = 0; i < order; i++)
                {
                    // Most of the time is spent in the following dot product.
                    auto kmax = std::min(i, j);
                    auto s = 0.0;
                    for (auto k = 0; k < kmax; k++)
                    {
                        s += data[k * order + i] * vecLUcolj[k];
                    }

                    data[indexj + i] = vecLUcolj[i] -= s;
                }

                // Find pivot and exchange if necessary.
                auto p = j;
                for (auto i = j + 1; i < order; i++)
                {
                    if (std::abs(vecLUcolj[i]) > std::abs(vecLUcolj[p]))
                    {
                        p = i;
                    }
                }

                if (p != j)
                {
                    for (auto k = 0; k < order; k++)
                    {
                        auto indexk = k * order;
                        auto indexkp = indexk + p;
                        auto indexkj = indexk + j;
                        auto temp = data[indexkp];
                        data[indexkp] = data[indexkj];
                        data[indexkj] = temp;
                    }

                    ipiv[j] = p;
                }

                // Compute multipliers.
                if (j < order & data[indexjj] != 0.0)
                {
                    for (auto i = j + 1; i < order; i++)
                    {
                        data[indexj + i] /= data[indexjj];
                    }
                }
            }
        }

        double Determinant(std::vector<double> A)
        {
            switch (mDimension)
            {
            case 0:
                return 0.0;
            case 1:
                return A[0];
            case 2:
                return A[0] * A[3] - A[1] * A[2];
            case 3:
                return A[0] * A[4] * A[8] + A[1] * A[5] * A[6] + A[2] * A[3] * A[7] - A[0] * A[5] * A[7] - A[1] * A[3] * A[8] - A[2] * A[4] * A[6];
            default:
            {
                std::vector<int> iPiv;
                std::vector<double> helper;
                iPiv.assign(mDimension, 0);
                helper.assign(mDimension, 0.0);

                LUFactor(A, mDimension, iPiv, helper);
                auto det = 1.0;
                for (auto i = 0; i < iPiv.size(); i++)
                {
                    det *= A[mDimension * i + i];
                    if (iPiv[i] != i)
                        det *= -1; // the determinant sign changes on row swap.
                }
                return det;
            }
            }
        }

        void FindNormalVector(std::vector<int> vertices, std::vector<double> &normalData)
        {
            switch (mDimension)
            {
            case 2:
                FindNormalVector2D(vertices, normalData);
                break;
            case 3:
                FindNormalVector3D(vertices, normalData);
                break;
            case 4:
                FindNormalVector4D(vertices, normalData);
                break;
            default:
                FindNormalVectorND(vertices, normalData);
                break;
            }
        }
        /// <summary>
        /// Finds 2D normal vector.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="normal">The normal.</param>
        void FindNormalVector2D(std::vector<int> vertices, std::vector<double> &normal)
        {
            auto ntX = VectorBetweenVertices(vertices[1], vertices[0]);

            auto nx = -ntX[1];
            auto ny = ntX[0];

            auto norm = std::sqrt(nx * nx + ny * ny);

            auto f = 1.0 / norm;
            normal[0] = f * nx;
            normal[1] = f * ny;
        }
        /// <summary>
        /// Finds 3D normal vector.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="normal">The normal.</param>
        void FindNormalVector3D(std::vector<int> vertices, std::vector<double> &normal)
        {
            auto ntX = VectorBetweenVertices(vertices[1], vertices[0]);
            auto ntY = VectorBetweenVertices(vertices[2], vertices[1]);

            auto nx = ntX[1] * ntY[2] - ntX[2] * ntY[1];
            auto ny = ntX[2] * ntY[0] - ntX[0] * ntY[2];
            auto nz = ntX[0] * ntY[1] - ntX[1] * ntY[0];

            auto norm = std::sqrt(nx * nx + ny * ny + nz * nz);

            auto f = 1.0 / norm;
            normal[0] = f * nx;
            normal[1] = f * ny;
            normal[2] = f * nz;
        }
        /// <summary>
        /// Finds 4D normal vector.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="normal">The normal.</param>
        void FindNormalVector4D(std::vector<int> vertices, std::vector<double> &normal)
        {
            auto ntX = VectorBetweenVertices(vertices[1], vertices[0]);
            auto ntY = VectorBetweenVertices(vertices[2], vertices[1]);
            auto ntZ = VectorBetweenVertices(vertices[3], vertices[2]);

            auto x = ntX;
            auto y = ntY;
            auto z = ntZ;

            // This was generated using Mathematica
            auto nx = x[3] * (y[2] * z[1] - y[1] * z[2]) + x[2] * (y[1] * z[3] - y[3] * z[1]) + x[1] * (y[3] * z[2] - y[2] * z[3]);
            auto ny = x[3] * (y[0] * z[2] - y[2] * z[0]) + x[2] * (y[3] * z[0] - y[0] * z[3]) + x[0] * (y[2] * z[3] - y[3] * z[2]);
            auto nz = x[3] * (y[1] * z[0] - y[0] * z[1]) + x[1] * (y[0] * z[3] - y[3] * z[0]) + x[0] * (y[3] * z[1] - y[1] * z[3]);
            auto nw = x[2] * (y[0] * z[1] - y[1] * z[0]) + x[1] * (y[2] * z[0] - y[0] * z[2]) + x[0] * (y[1] * z[2] - y[2] * z[1]);

            auto norm = std::sqrt(nx * nx + ny * ny + nz * nz + nw * nw);

            auto f = 1.0 / norm;
            normal[0] = f * nx;
            normal[1] = f * ny;
            normal[2] = f * nz;
            normal[3] = f * nw;
        }

        /// <summary>
        /// Finds the normal vector nd.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="normal">The normal.</param>
        void FindNormalVectorND(std::vector<int> vertices, std::vector<double> &normal)
        {
            /* We need to solve the matrix A n = B where
             *  - A contains coordinates of vertices as columns
             *  - B is vector with all 1's. Really, it should be the distance of
             *      the plane from the origin, but - since we're not worried about that
             *      here and we will normalize the normal anyway - all 1's suffices.
             */
            std::vector<int> iPiv;
            std::vector<double> data;
            std::vector<double> nDNormalHelperVector;

            iPiv.assign(mDimension, -1);
            data.assign(mDimension * mDimension, 0.0);
            nDNormalHelperVector.assign(mDimension, 0.0);

            auto norm = 0.0;

            // Solve determinants by replacing x-th column by all 1.
            for (auto x = 0; x < mDimension; x++)
            {
                for (auto i = 0; i < mDimension; i++)
                {
                    auto offset = vertices[i] * mDimension;
                    for (auto j = 0; j < mDimension; j++)
                    {
                        // maybe I got the i/j mixed up here regarding the representation Math.net uses...
                        // ...but it does not matter since Det(A) = Det(Transpose(A)).
                        data[mDimension * i + j] = j == x ? 1.0 : mPositions[offset + j];
                    }
                }
                LUFactor(data, mDimension, iPiv, nDNormalHelperVector);
                auto coord = 1.0;
                for (auto i = 0; i < mDimension; i++)
                {
                    if (iPiv[i] != i)
                        coord *= -data[mDimension * i + i]; // the determinant sign changes on row swap.
                    else
                        coord *= data[mDimension * i + i];
                }
                normal[x] = coord;
                norm += coord * coord;
            }

            // Normalize the result
            auto f = 1.0 / std::sqrt(norm);
            for (auto i = 0; i < normal.size(); i++)
                normal[i] *= f;
        }

        double GetSimplexVolume(std::vector<std::vector<double>> edgeVectors, int lastIndex, double bigNumber)
        {
            std::vector<double> A;
            A.assign(mDimension * mDimension + 1, 0.0);

            auto index = 0;
            for (auto i = 0; i < mDimension; i++)
                for (auto j = 0; j < mDimension; j++)
                    if (i <= lastIndex)
                        A[index++] = edgeVectors[i][j];
                    else
                        A[index] = (std::pow(-1, index) * index++) / bigNumber;
            // this last term is used for all the vertices in the comparison for the yet determined vertices
            // the idea is to come up with sets of numbers that are orthogonal so that an non-zero value will result
            // and to choose smallish numbers since the choice of vectors will affect what the end volume is.
            // A better way (todo?) is to solve a smaller matrix. However, cases were found in which the obvious smaller vector
            // (the upper left) had too many zeros. So, one would need to find the right subset. Indeed choosing a subset
            // biases the first dimensions of the others. Perhaps a larger volume would be created from a different vertex
            // if another subset of dimensions were used.
            return std::abs(Determinant(A));
        }

        void RandomOffsetToLift(int index, double maxHeight)
        {

            std::random_device seedGen;
            std::default_random_engine rndEngine(seedGen());
            std::uniform_real_distribution<double> dist(0.0, 1.0);

            auto liftIndex = (index * mDimension) + mDimension - 1;
            mPositions[liftIndex] += 0.0001 * maxHeight * (dist(rndEngine) - 0.5);
        }

        std::vector<int> FindInitialPoints()
        {
            auto bigNumber = std::accumulate(mAxisMaxma.begin(), mAxisMaxma.end(), decltype(mAxisMaxma)::value_type(0)) * mDimension * mNumberOfVertices;
            // KIRI_LOG_DEBUG("big number ={0}", bigNumber);

            auto vertex1 = mBoundingBoxPoints[mIndexOfDimensionWithLeastExtremes].front(); // these are min and max vertices along
            auto vertex2 = mBoundingBoxPoints[mIndexOfDimensionWithLeastExtremes].back();  // the dimension that had the fewest points
            mBoundingBoxPoints[mIndexOfDimensionWithLeastExtremes].erase(mBoundingBoxPoints[mIndexOfDimensionWithLeastExtremes].begin() + 0);
            mBoundingBoxPoints[mIndexOfDimensionWithLeastExtremes].erase(mBoundingBoxPoints[mIndexOfDimensionWithLeastExtremes].begin() + mBoundingBoxPoints[mIndexOfDimensionWithLeastExtremes].size() - 1);

            // KIRI_LOG_DEBUG("vertex1={0}, vertex2={1},", vertex1, vertex2);
            std::vector<int> initialPoints{vertex1, vertex2};

            mVertexVisited[vertex1] = mVertexVisited[vertex2] = true;
            mBuffer->CurrentVertex = vertex1;
            UpdateCenter();
            mBuffer->CurrentVertex = vertex2;
            UpdateCenter();
            // KIRI_LOG_DEBUG("centroid={0},{1},{2}", mCentroid[0], mCentroid[1], mCentroid[2]);

            std::vector<std::vector<double>> edgeVectors;
            edgeVectors.assign(mDimension, std::vector<double>());
            edgeVectors[0] = VectorBetweenVertices(vertex2, vertex1);
            // KIRI_LOG_DEBUG("edgeVectors[0]={0},{1},{2}", edgeVectors[0][0], edgeVectors[0][1], edgeVectors[0][2]);

            std::vector<int> extremes;
            std::for_each(mBoundingBoxPoints.begin(), mBoundingBoxPoints.end(),
                          [&](std::vector<int> elem)
                          { extremes.insert(extremes.end(), elem.begin(), elem.end()); });

            // otherwise find the remaining points by maximizing the initial simplex volume
            auto index = 1;
            while (index < mDimension && !extremes.empty())
            {
                auto bestVertex = -1;
                std::vector<double> bestEdgeVector;
                auto maxVolume = PLANE_DISTANCE_TOLERANCE;

                extremes.erase(std::remove_if(extremes.begin(), extremes.end(),
                                              [=](int elem)
                                              { if(std::find(initialPoints.begin(), initialPoints.end(), elem) != initialPoints.end()) return true;else return false; }),
                               extremes.end());

                for (auto i = 0; i < extremes.size(); i++)
                {
                    // count backwards in order to remove potential duplicates
                    auto vIndex = extremes[i];

                    edgeVectors[index] = VectorBetweenVertices(vIndex, vertex1);
                    auto volume = GetSimplexVolume(edgeVectors, index, bigNumber);
                    if (maxVolume < volume)
                    {
                        maxVolume = volume;
                        bestVertex = vIndex;
                        bestEdgeVector = edgeVectors[index];
                    }
                }

                extremes.erase(std::remove_if(extremes.begin(), extremes.end(),
                                              [=](int elem)
                                              { if(elem == bestVertex) return true;else return false; }),
                               extremes.end());

                if (bestVertex == -1)
                    break;
                initialPoints.emplace_back(bestVertex);
                edgeVectors[index++] = bestEdgeVector;
                mBuffer->CurrentVertex = bestVertex;
                UpdateCenter();
            }

            if (initialPoints.size() <= mDimension && !mForPowerDiagram)
            {
                // KIRI_LOG_DEBUG("initial points not enough!!");

                std::vector<int> allVertices(mNumberOfVertices);
                std::iota(allVertices.begin(), allVertices.end(), 0);
                while (index < mDimension && !allVertices.empty())
                {
                    auto bestVertex = -1;
                    std::vector<double> bestEdgeVector;
                    auto maxVolume = 0.0;

                    allVertices.erase(std::remove_if(allVertices.begin(), allVertices.end(),
                                                     [=](int elem)
                                                     { if(std::find(initialPoints.begin(), initialPoints.end(), elem) != initialPoints.end()) return true;else return false; }),
                                      allVertices.end());

                    for (auto i = 0; i < allVertices.size(); i++)
                    {
                        auto vIndex = allVertices[i];

                        edgeVectors[index] = VectorBetweenVertices(vIndex, vertex1);
                        auto volume = GetSimplexVolume(edgeVectors, index, bigNumber);
                        if (maxVolume < volume)
                        {
                            maxVolume = volume;
                            bestVertex = vIndex;
                            bestEdgeVector = edgeVectors[index];
                        }
                    }

                    allVertices.erase(std::remove_if(allVertices.begin(), allVertices.end(),
                                                     [=](int elem)
                                                     { if(elem == bestVertex) return true;else return false; }),
                                      allVertices.end());

                    if (bestVertex == -1)
                        break;
                    initialPoints.emplace_back(bestVertex);
                    edgeVectors[index++] = bestEdgeVector;
                    mBuffer->CurrentVertex = bestVertex;
                    UpdateCenter();
                }
            }

            if (initialPoints.size() <= mDimension && mForPowerDiagram)
            {
                // KIRI_LOG_DEBUG("initial points not enough!!");

                std::vector<int> allVertices(mNumberOfVertices);
                std::iota(allVertices.begin(), allVertices.end(), 0);
                while (index < mDimension && !allVertices.empty())
                {
                    auto bestVertex = -1;
                    std::vector<double> bestEdgeVector;
                    auto maxVolume = 0.0;

                    allVertices.erase(std::remove_if(allVertices.begin(), allVertices.end(),
                                                     [=](int elem)
                                                     { if(std::find(initialPoints.begin(), initialPoints.end(), elem) != initialPoints.end()) return true;else return false; }),
                                      allVertices.end());

                    for (auto i = 0; i < allVertices.size(); i++)
                    {
                        auto vIndex = allVertices[i];

                        RandomOffsetToLift(vIndex, mAxisMaxma.back() - mAxisMinma.back());
                        edgeVectors[index] = VectorBetweenVertices(vIndex, vertex1);
                        auto volume = GetSimplexVolume(edgeVectors, index, bigNumber);
                        if (maxVolume < volume)
                        {
                            maxVolume = volume;
                            bestVertex = vIndex;
                            bestEdgeVector = edgeVectors[index];
                        }
                    }

                    allVertices.erase(std::remove_if(allVertices.begin(), allVertices.end(),
                                                     [=](int elem)
                                                     { if(elem == bestVertex) return true;else return false; }),
                                      allVertices.end());

                    if (bestVertex == -1)
                        break;
                    initialPoints.emplace_back(bestVertex);
                    edgeVectors[index++] = bestEdgeVector;
                    mBuffer->CurrentVertex = bestVertex;
                    UpdateCenter();
                }
            }

            return initialPoints;
        }

        void GetHullVertices()
        {
            mVertices.clear();

            auto cellCount = mBuffer->ConvexSimplexs.size();
            auto hullVertexCount = 0;

            for (auto i = 0; i < mNumberOfVertices; i++)
                mVertexVisited[i] = false;

            for (auto i = 0; i < cellCount; i++)
            {
                // auto myid = omp_get_thread_num();
                // KIRI_LOG_DEBUG("thread id={0},cell count={1}", myid, cellCount);

                auto vs = mBuffer->ConvexSimplexs[i]->Vertices;
                for (auto j = 0; j < vs.size(); j++)
                {
                    auto v = vs[j];
                    if (!mVertexVisited[v])
                    {
                        mVertexVisited[v] = true;
                        hullVertexCount++;
                    }
                }
            }

            mVertices.assign(hullVertexCount, VERTEXPTR());

            auto counter = 0;
            for (auto i = 0; i < mNumberOfVertices; i++)
            {
                if (mVertexVisited[i])
                    mVertices[--hullVertexCount] = mInput[i];
            }
        }

        void CreateInitialSimplex()
        {
            auto initialPoints = FindInitialPoints();

            std::vector<std::shared_ptr<SimplexWrap>> faces;
            faces.assign(mDimension + 1, std::make_shared<SimplexWrap>());

            for (auto i = 0; i < mDimension + 1; i++)
            {
                std::vector<int> vertices;
                vertices.assign(mDimension, -1);

                for (auto j = 0, k = 0; j <= mDimension; j++)
                {
                    if (i != j)
                        vertices[k++] = initialPoints[j];
                }

                auto newFace = std::make_shared<SimplexWrap>(mDimension, std::vector<int>());

                std::sort(vertices.begin(), vertices.end());
                newFace->Vertices = vertices;

                CalculateFacePlane(newFace);
                faces[i] = newFace;
            }

            // update the adjacency (check all pairs of faces)
            for (auto i = 0; i < mDimension; i++)
            {
                for (auto j = i + 1; j < mDimension + 1; j++)
                    UpdateAdjacency(faces[i], faces[j]);
            }

            auto numFaces = faces.size();
            // Init the vertex beyond buffers.
            for (auto i = 0; i < numFaces; i++)
            {
                FindBeyondVertices(faces[i]);

                // KIRI_LOG_DEBUG("faces[i]->VerticesBeyond.size()={0}", faces[i]->VerticesBeyond.size());
                if (faces[i]->VerticesBeyond.size() == 0)
                    mBuffer->ConvexSimplexs.emplace_back(faces[i]); // The face is on the hull
                else
                    mBuffer->UnprocessedFaces->Add(faces[i]);

                // faces[i]->ToString();
            }

            // Set all vertices to false (unvisited).
            for (auto i = 0; i < initialPoints.size(); i++)
            {
                mVertexVisited[initialPoints[i]] = false;
            }
        }

        void Generate(const std::vector<VERTEXPTR> &input, bool assignIds = true, bool checkInput = false)
        {
            Clear();

            mBuffer = std::make_shared<ObjectBuffer<VERTEXPTR>>(mDimension);

            auto inputCount = input.size();
            if (inputCount < mDimension + 1)
                return;

            SerializeVerticesToPositions(input);

            FindBoundingBoxPoints();

            ShiftAndScalePositions();

            CreateInitialSimplex();

            while (mBuffer->UnprocessedFaces->First != nullptr)
            {
                mBuffer->CurrentVertex = mBuffer->UnprocessedFaces->First->FurthestVertex;

                UpdateCenter();

                // KIRI_LOG_DEBUG("centroid={0},{1},{2}", mCentroid[0], mCentroid[1], mCentroid[2]);

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

                    if (wrap->IsNormalFlipped)
                        mSimplexs[i]->Vertices[j] = mInput[wrap->Vertices[mDimension - j - 1]];
                    else
                        mSimplexs[i]->Vertices[j] = mInput[wrap->Vertices[j]];

                    if (wrap->AdjacentFaces[j] != nullptr)
                    {
                        if (wrap->IsNormalFlipped)
                            mSimplexs[i]->Adjacent[j] = mSimplexs[wrap->AdjacentFaces[mDimension - j - 1]->GetTag()];
                        else
                            mSimplexs[i]->Adjacent[j] = mSimplexs[wrap->AdjacentFaces[j]->GetTag()];
                    }
                }

                // mSimplexs[i]->CalculateCentroid();
            }

            GetHullVertices();

            // release memory
            mBuffer->Clear();
            mBuffer = nullptr;
        }

        double GetVertexDistance(int v, const std::shared_ptr<SimplexWrap> &f)
        {
            auto normal = f->Normals;
            auto x = v * mDimension;
            auto distance = f->Offset;
            for (auto i = 0; i < normal.size(); i++)
                distance += normal[i] * mPositions[x + i];
            return distance;
        }

        int CompareTo(double a, double b)
        {
            if (a > b)
                return 1;
            else if (b > a)
                return -1;
            else
                return 0;
        }

        /// <summary>
        /// Compares the values of two vertices. The return value (-1, 0 or +1) are found
        /// by first checking the first coordinate and then progressing through the rest.
        /// In this way {2, 8} will be a "-1" (less than) {3, 1}.
        /// </summary>
        /// <param name="u">The base vertex index, u.</param>
        /// <param name="v">The compared vertex index, v.</param>
        /// <returns>System.Int32.</returns>
        int LexCompare(int u, int v)
        {
            int uOffset = u * mDimension, vOffset = v * mDimension;
            for (auto i = 0; i < mDimension; i++)
            {
                double x = mPositions[uOffset + i], y = mPositions[vOffset + i];
                auto comp = CompareTo(x, y);
                if (comp != 0)
                    return comp;
            }
            return 0;
        }

        void IsBeyond(const std::shared_ptr<SimplexWrap> &face, std::vector<int> &beyondVertices, int v)
        {
            auto distance = GetVertexDistance(v, face);
            if (distance >= PLANE_DISTANCE_TOLERANCE)
            {
                if (distance > mBuffer->MaxDistance)
                {
                    // If it's within the tolerance distance, use the lex. larger point
                    if ((distance - mBuffer->MaxDistance) < PLANE_DISTANCE_TOLERANCE)
                    { // todo: why is this LexCompare necessary. Would seem to favor x over y over z (etc.)?
                        if (LexCompare(v, mBuffer->FurthestVertex) > 0)
                        {
                            mBuffer->MaxDistance = distance;
                            mBuffer->FurthestVertex = v;
                        }
                    }
                    else
                    {
                        mBuffer->MaxDistance = distance;
                        mBuffer->FurthestVertex = v;
                    }
                }
                beyondVertices.emplace_back(v);
            }
        }

        // Recalculates the centroid of the current hull.
        void UpdateCenter()
        {
            for (auto i = 0; i < mDimension; i++)
                mCentroid[i] *= mConvexHullSize;
            mConvexHullSize++;
            auto f = 1.0 / mConvexHullSize;
            auto co = mBuffer->CurrentVertex * mDimension;
            for (auto i = 0; i < mDimension; i++)
                mCentroid[i] = f * (mCentroid[i] + mPositions[co + i]);
        }

        void RollbackCenter()
        {
            for (auto i = 0; i < mDimension; i++)
                mCentroid[i] *= mConvexHullSize;
            mConvexHullSize -= 1;
            auto f = mConvexHullSize > 0 ? 1.0 / mConvexHullSize : 0.0;
            auto co = mBuffer->CurrentVertex * mDimension;
            for (auto i = 0; i < mDimension; i++)
                mCentroid[i] = f * (mCentroid[i] - mPositions[co + i]);
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
                mAxisMinma[i] = min;
                mAxisMaxma[i] = max;

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
        /// Calculates the normal and offset of the hyper-plane given by the face's vertices.
        /// </summary>
        bool CalculateFacePlane(const std::shared_ptr<SimplexWrap> &face)
        {
            auto vertices = face->Vertices;
            auto normal = face->Normals;

            FindNormalVector(vertices, normal);

            face->Normals = normal;

            if (normal[0] != normal[0])
            {
                return false;
            }

            auto offset = 0.0;
            auto centerDistance = 0.0;

            auto fi = vertices[0] * mDimension;
            for (auto i = 0; i < mDimension; i++)
            {
                auto n = normal[i];
                offset += n * mPositions[fi + i];
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
        void UpdateAdjacency(const std::shared_ptr<SimplexWrap> &l, const std::shared_ptr<SimplexWrap> &r)
        {
            auto lv = l->Vertices;
            auto rv = r->Vertices;
            auto i = 0;

            // reset marks on the 1st face
            for (i = 0; i < lv.size(); i++)
                mVertexVisited[lv[i]] = false;

            // mark all vertices on the 2nd face
            for (i = 0; i < rv.size(); i++)
                mVertexVisited[rv[i]] = true;

            // find the 1st false index
            for (i = 0; i < lv.size(); i++)
                if (!mVertexVisited[lv[i]])
                    break;

            // no vertex was marked
            if (i == mDimension)
                return;

            // check if only 1 vertex wasn't marked
            for (auto j = i + 1; j < lv.size(); j++)
                if (!mVertexVisited[lv[j]])
                    return;

            // if we are here, the two faces share an edge
            l->AdjacentFaces[i] = r;

            // update the adj. face on the other face - find the vertex that remains marked
            for (i = 0; i < lv.size(); i++)
                mVertexVisited[lv[i]] = false;
            for (i = 0; i < rv.size(); i++)
            {
                if (mVertexVisited[rv[i]])
                    break;
            }
            r->AdjacentFaces[i] = l;
        }

        /// <summary>
        /// Used in the "initialization" code.
        /// </summary>
        void FindBeyondVertices(const std::shared_ptr<SimplexWrap> &face)
        {
            mBuffer->MaxDistance = std::numeric_limits<double>::lowest();
            mBuffer->FurthestVertex = 0;

            for (auto i = 0; i < mNumberOfVertices; i++)
            {
                if (mVertexVisited[i])
                    continue;
                IsBeyond(face, face->VerticesBeyond, i);
            }

            face->FurthestVertex = mBuffer->FurthestVertex;
        }

#pragma endregion Initilization

#pragma region Process

        /// <summary>
        /// Tags all faces seen from the current vertex with 1.
        /// </summary>
        void TagAffectedFaces(const std::shared_ptr<SimplexWrap> &currentFace)
        {
            mBuffer->AffectedFaceBuffer.clear();
            mBuffer->AffectedFaceBuffer.emplace_back(currentFace);
            TraverseAffectedFaces(currentFace);
        }

        /// <summary>
        /// Recursively traverse all the relevant faces.
        /// </summary>
        void TraverseAffectedFaces(const std::shared_ptr<SimplexWrap> &currentFace)
        {

            mBuffer->TraverseStack = std::stack<std::shared_ptr<SimplexWrap>>();
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

                    if (adjFace == nullptr)
                        throw std::invalid_argument("(2) Adjacent Face should never be nullptr");

                    if (adjFace->GetTag() == 0 && GetVertexDistance(mBuffer->CurrentVertex, adjFace) >= PLANE_DISTANCE_TOLERANCE)
                    {
                        mBuffer->AffectedFaceBuffer.emplace_back(top->AdjacentFaces[i]);
                        top->AdjacentFaces[i]->SetTag(1);
                        mBuffer->TraverseStack.push(top->AdjacentFaces[i]);
                    }
                }
            }
        }

        bool CreateCone()
        {
            auto currentVertexIndex = mBuffer->CurrentVertex;
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

                    auto oldVertexIndex = vertices[forbidden];

                    auto orderedPivotIndex = 0;

                    // correct the ordering
                    if (currentVertexIndex < oldVertexIndex)
                    {
                        orderedPivotIndex = 0;
                        for (auto j = forbidden - 1; j >= 0; j--)
                        {
                            if (vertices[j] > currentVertexIndex)
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
                            if (vertices[j] < currentVertexIndex)
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

        std::shared_ptr<DeferredSimplex> MakeDeferredFace(
            const std::shared_ptr<SimplexWrap> &face,
            int faceIndex,
            const std::shared_ptr<SimplexWrap> &pivot,
            int pivotIndex,
            const std::shared_ptr<SimplexWrap> &oldFace)
        {
            auto ret = mBuffer->ObjManager->GetDeferredSimplex();

            ret->Face = face;
            ret->FaceIndex = faceIndex;
            ret->Pivot = pivot;
            ret->PivotIndex = pivotIndex;
            ret->OldFace = oldFace;

            return ret;
        }

        void CommitCone()
        {

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

                //   This could slightly help...
                if (mBuffer->ConeFaceBuffer[i]->Pivot->VerticesBeyond.size() < mBuffer->ConeFaceBuffer[i]->OldFace->VerticesBeyond.size())
                {
                    FindBeyondVertices(mBuffer->ConeFaceBuffer[i]->Face, mBuffer->ConeFaceBuffer[i]->Pivot->VerticesBeyond, mBuffer->ConeFaceBuffer[i]->OldFace->VerticesBeyond);
                }
                else
                {
                    FindBeyondVertices(mBuffer->ConeFaceBuffer[i]->Face, mBuffer->ConeFaceBuffer[i]->OldFace->VerticesBeyond, mBuffer->ConeFaceBuffer[i]->Pivot->VerticesBeyond);
                }

                // This face will definitely lie on the hull
                if (mBuffer->ConeFaceBuffer[i]->Face->VerticesBeyond.size() == 0)
                {
                    mBuffer->ConvexSimplexs.emplace_back(mBuffer->ConeFaceBuffer[i]->Face);
                    mBuffer->UnprocessedFaces->Remove(mBuffer->ConeFaceBuffer[i]->Face);
                    mBuffer->ObjManager->DepositVertexBuffer(mBuffer->ConeFaceBuffer[i]->Face->VerticesBeyond);

                    mBuffer->ConeFaceBuffer[i]->Face->VerticesBeyond = mBuffer->EmptyBuffer;
                }
                else // Add the face to the list
                {

                    mBuffer->UnprocessedFaces->Add(mBuffer->ConeFaceBuffer[i]->Face);
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
        void ConnectFace(const std::shared_ptr<SimplexConnector> &connector)
        {
            auto index = connector->GetHashCode() % mBuffer->CONNECTOR_TABLE_SIZE;

            for (auto current = mBuffer->ConnectorTable[index]->First; current != nullptr; current = current->Next)
            {
                if (SimplexConnector().AreConnectable(connector, current, mDimension))
                {
                    mBuffer->ConnectorTable[index]->Remove(current);
                    SimplexConnector().Connect(current, connector);

                    mBuffer->ObjManager->DepositConnector(current);
                    mBuffer->ObjManager->DepositConnector(connector);
                    return;
                }
            }

            mBuffer->ConnectorTable[index]->Add(connector);
        }

        void FindBeyondVertices(
            const std::shared_ptr<SimplexWrap> &face,
            const std::vector<int> &beyond,
            const std::vector<int> &beyond1)
        {
            auto beyondVertices = mBuffer->BeyondBuffer;

            mBuffer->MaxDistance = std::numeric_limits<double>::lowest();
            mBuffer->FurthestVertex = 0;

            int v;

            auto count = beyond1.size();

            for (auto i = 0; i < count; i++)
                mVertexVisited[beyond1[i]] = true;

            mVertexVisited[mBuffer->CurrentVertex] = false;

            count = beyond.size();
            for (auto i = 0; i < count; i++)
            {
                v = beyond[i];

                if (v == mBuffer->CurrentVertex)
                    continue;
                mVertexVisited[v] = false;
                IsBeyond(face, beyondVertices, v);
            }

            count = beyond1.size();
            for (auto i = 0; i < count; i++)
            {
                v = beyond1[i];

                if (mVertexVisited[v])
                    IsBeyond(face, beyondVertices, v);
            }

            face->FurthestVertex = mBuffer->FurthestVertex;

            // Pull the old switch a roo
            auto temp = face->VerticesBeyond;
            face->VerticesBeyond = beyondVertices;
            if (temp.size() > 0)
                temp.clear();
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
                for (auto i = 0; i < mBuffer->AffectedFaceBuffer[fIndex]->VerticesBeyond.size(); i++)
                {
                    mBuffer->SingularVertices.insert(mBuffer->AffectedFaceBuffer[fIndex]->VerticesBeyond[i]);
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

        const double PLANE_DISTANCE_TOLERANCE = 1e-10;
        int mDimension;
        int mNumberOfVertices;
        bool mForPowerDiagram = false;

        int mConvexHullSize = 0;
        int mIndexOfDimensionWithLeastExtremes;

        std::vector<VERTEXPTR> mVertices;
        std::vector<std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>>> mSimplexs;

        std::shared_ptr<ObjectBuffer<VERTEXPTR>> mBuffer;

        std::vector<VERTEXPTR> mInput;
        std::vector<bool> mVertexVisited;
        std::vector<double> mPositions;
        std::vector<double> mAxisMinma;
        std::vector<double> mAxisMaxma;
        std::vector<double> mShiftAmount;
        std::vector<double> mCentroid;
        std::vector<std::vector<int>> mBoundingBoxPoints;
    };

    class ConvexHull2 : public ConvexHull<HDV::Primitives::Vertex2Ptr>
    {
    public:
        explicit ConvexHull2() : ConvexHull(2) {}

        ~ConvexHull2() {}
    };

    class ConvexHull3 : public ConvexHull<HDV::Primitives::Vertex3Ptr>
    {
    public:
        explicit ConvexHull3() : ConvexHull(3) {}

        ~ConvexHull3() {}
    };
} // namespace HDV::Hull

#endif /* _HDV_CONVEX_HULL_H_ */