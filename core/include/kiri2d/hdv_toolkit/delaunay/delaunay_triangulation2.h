/***
 * @Author: Xu.WANG
 * @Date: 2021-12-08 15:00:22
 * @LastEditTime: 2021-12-08 15:00:56
 * @LastEditors: Xu.WANG
 * @Description:
 */
#ifndef _HDV_DELAUNAY_TRIANGULATION2_H_
#define _HDV_DELAUNAY_TRIANGULATION2_H_

#pragma once

#include <kiri2d/hdv_toolkit/delaunay/delaunay_triangulation.h>
#include <kiri2d/hdv_toolkit/hull/convex_hull.h>

namespace HDV::Delaunay
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class DelaunayTriangulation2D : public DelaunayTriangulation<VERTEXPTR, VERTEX>
    {

    public:
        explicit DelaunayTriangulation2D()
            : DelaunayTriangulation<VERTEXPTR, VERTEX>(2)
        {
            mMatrixBuffer.assign(3, std::vector<double>());
            for (auto i = 0; i < mMatrixBuffer.size(); i++)
            {
                mMatrixBuffer[i].assign(3, 0.0);
            }
        }

        virtual ~DelaunayTriangulation2D()
        {
        }

        void generate(const std::vector<VERTEXPTR> &input, bool assignIds = true, bool checkInput = false) override
        {

            clear();
            auto dim = mDimension;

            if (input.size() <= dim + 1)
                return;

            auto count = input.size();

            for (auto i = 0; i < count; i++)
            {
                auto v = input[i]->positions();
                v.resize(dim + 1);
                v[dim] = input[i]->lengthSquared() - input[i]->weight();
                // KIRI_LOG_DEBUG("weight={0}", input[i]->weight());
                input[i]->positions() = v;
            }

            mHull = std::make_shared<HDV::Hull::ConvexHull<VERTEXPTR>>(dim + 1);
            mHull->enablePowerDiagram(true);
            mHull->generate(input, assignIds, checkInput);

            for (auto i = 0; i < count; i++)
            {
                auto v = input[i]->positions();
                v.resize(dim);
                input[i]->positions() = v;
            }

            mVertices = mHull->vertices();

            mCentroid->positions()[0] = mHull->centroid()[0];
            mCentroid->positions()[1] = mHull->centroid()[1];

            count = mHull->simplexs().size();

            for (auto i = 0; i < count; i++)
            {

                auto simplex = mHull->simplexs()[i];

                if (simplex->normals()[dim] >= 0.0f)
                {
                    for (auto j = 0; j < simplex->adjacents().size(); j++)
                    {
                        if (simplex->adjacents()[j] != nullptr)
                        {
                            simplex->adjacents()[j]->remove(simplex);
                        }
                    }
                }
                else
                {
                    auto cell = createCell(simplex);
                    mCells.emplace_back(cell);
                }
            }
        }

    private:
        std::vector<std::vector<double>> mMatrixBuffer;

        constexpr double determinant() const
        {
            auto factor00 = mMatrixBuffer[1][1] * mMatrixBuffer[2][2] - mMatrixBuffer[1][2] * mMatrixBuffer[2][1];
            auto factor10 = mMatrixBuffer[1][2] * mMatrixBuffer[2][0] - mMatrixBuffer[1][0] * mMatrixBuffer[2][2];
            auto factor20 = mMatrixBuffer[1][0] * mMatrixBuffer[2][1] - mMatrixBuffer[1][1] * mMatrixBuffer[2][0];

            return mMatrixBuffer[0][0] * factor00 + mMatrixBuffer[0][1] * factor10 + mMatrixBuffer[0][2] * factor20;
        }

        /**
         * @reference MathWorld: http://mathworld.wolfram.com/Circumcircle.html
         * @param  {std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>>} simplex :
         * @return {std::shared_ptr<DelaunayCell<VERTEXPTR,}                      :
         */
        std::shared_ptr<DelaunayCell<VERTEXPTR, VERTEX>> createCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> &simplex)
        {
            auto verts = simplex->vertices();

            // x, y, 1
            for (auto i = 0; i < 3; i++)
            {
                mMatrixBuffer[i][0] = verts[i]->positions()[0];
                mMatrixBuffer[i][1] = verts[i]->positions()[1];
                mMatrixBuffer[i][2] = 1;
            }

            auto a = determinant();

            // size, y, 1
            for (auto i = 0; i < 3; i++)
            {
                mMatrixBuffer[i][0] = verts[i]->lengthSquared() - verts[i]->weight();
            }

            auto dx = -determinant();

            // size, x, 1
            for (auto i = 0; i < 3; i++)
            {
                mMatrixBuffer[i][1] = verts[i]->positions()[0];
            }

            auto dy = determinant();

            // size, x, y
            for (auto i = 0; i < 3; i++)
            {
                mMatrixBuffer[i][2] = verts[i]->positions()[1];
            }
            auto c = -determinant();

            auto s = -1.0 / (2.0 * a);

            std::vector<double> circum_center;
            circum_center.assign(2, 0.0);
            circum_center[0] = s * dx;
            circum_center[1] = s * dy;

            auto radius = std::abs(s) * std::sqrtf(dx * dx + dy * dy - 4.0 * a * c);

            return std::make_shared<DelaunayCell<VERTEXPTR, VERTEX>>(simplex, circum_center, radius);
        }
    };

    typedef DelaunayTriangulation2D<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2> DelaunayTriangulation2;

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_TRIANGULATION2_H_ */