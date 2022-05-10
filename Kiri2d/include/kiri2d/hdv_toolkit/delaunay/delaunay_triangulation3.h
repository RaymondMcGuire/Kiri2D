/***
 * @Author: Xu.WANG
 * @Date: 2021-12-08 15:00:22
 * @LastEditTime: 2021-12-08 15:00:56
 * @LastEditors: Xu.WANG
 * @Description:
 */
#ifndef _HDV_DELAUNAY_TRIANGULATION3_H_
#define _HDV_DELAUNAY_TRIANGULATION3_H_

#pragma once

#include <kiri2d/hdv_toolkit/delaunay/delaunay_triangulation.h>
#include <kiri2d/hdv_toolkit/hull/convex_hull.h>
namespace HDV::Delaunay
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class DelaunayTriangulation3D : public DelaunayTriangulation<VERTEXPTR, VERTEX>
    {
    public:
        explicit DelaunayTriangulation3D()
            : DelaunayTriangulation<VERTEXPTR, VERTEX>(3)
        {
            mMatrixBuffer.assign(4, std::vector<double>());
            for (size_t i = 0; i < mMatrixBuffer.size(); i++)
            {
                mMatrixBuffer[i].assign(4, 0.0);
            }
        }

        virtual ~DelaunayTriangulation3D()
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
            KIRI_LOG_DEBUG("Hull->vertices size={0}; input size={1}", mVertices.size(), count);

            mCentroid->positions()[0] = mHull->centroid()[0];
            mCentroid->positions()[1] = mHull->centroid()[1];
            mCentroid->positions()[2] = mHull->centroid()[2];

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
                    // cell.CircumCenter.Id = i;
                    mCells.emplace_back(cell);
                }
            }
        }

    private:
        std::vector<std::vector<double>> mMatrixBuffer;

        constexpr double minor(int r0, int r1, int r2, int c0, int c1, int c2) const
        {
            return mMatrixBuffer[r0][c0] * (mMatrixBuffer[r1][c1] * mMatrixBuffer[r2][c2] - mMatrixBuffer[r2][c1] * mMatrixBuffer[r1][c2]) -
                   mMatrixBuffer[r0][c1] * (mMatrixBuffer[r1][c0] * mMatrixBuffer[r2][c2] - mMatrixBuffer[r2][c0] * mMatrixBuffer[r1][c2]) +
                   mMatrixBuffer[r0][c2] * (mMatrixBuffer[r1][c0] * mMatrixBuffer[r2][c1] - mMatrixBuffer[r2][c0] * mMatrixBuffer[r1][c1]);
        }

        constexpr double determinant() const
        {
            return (mMatrixBuffer[0][0] * minor(1, 2, 3, 1, 2, 3) -
                    mMatrixBuffer[0][1] * minor(1, 2, 3, 0, 2, 3) +
                    mMatrixBuffer[0][2] * minor(1, 2, 3, 0, 1, 3) -
                    mMatrixBuffer[0][3] * minor(1, 2, 3, 0, 1, 2));
        }
        /**
         * @reference MathWorld: http://mathworld.wolfram.com/Circumsphere.html
         * @param  {std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>>} simplex :
         * @return {std::shared_ptr<DelaunayCell<VERTEXPTR,}                      :
         */
        std::shared_ptr<DelaunayCell<VERTEXPTR, VERTEX>> createCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> &simplex)
        {
            auto verts = simplex->vertices();

            // x, y, z, 1
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][0] = verts[i]->positions()[0];
                mMatrixBuffer[i][1] = verts[i]->positions()[1];
                mMatrixBuffer[i][2] = verts[i]->positions()[2];
                mMatrixBuffer[i][3] = 1;
            }
            auto a = determinant();

            // size, y, z, 1
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][0] = verts[i]->lengthSquared() - verts[i]->weight();
            }
            auto dx = determinant();

            // size, x, z, 1
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][1] = verts[i]->positions()[0];
            }
            auto dy = -determinant();

            // size, x, y, 1
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][2] = verts[i]->positions()[1];
            }
            auto dz = determinant();

            // size, x, y, z
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][3] = verts[i]->positions()[2];
            }
            auto c = determinant();

            auto s = 1.0 / (2.0 * a);
            auto radius = std::abs(s) * std::sqrtf(dx * dx + dy * dy + dz * dz - 4 * a * c);

            std::vector<double> circum_center;
            circum_center.assign(3, 0.0);
            circum_center[0] = s * dx;
            circum_center[1] = s * dy;
            circum_center[2] = s * dz;

            return std::make_shared<DelaunayCell<VERTEXPTR, VERTEX>>(simplex, circum_center, radius);
        }
    };

    typedef DelaunayTriangulation3D<HDV::Primitives::Vertex3Ptr, HDV::Primitives::Vertex3> DelaunayTriangulation3;

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_TRIANGULATION3_H_ */