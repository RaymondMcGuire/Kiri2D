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
        explicit DelaunayTriangulation3D() : DelaunayTriangulation<VERTEXPTR, VERTEX>(3)
        {
            mMatrixBuffer.assign(4, std::vector<double>());
            for (size_t i = 0; i < mMatrixBuffer.size(); i++)
            {
                mMatrixBuffer[i].assign(4, 0.0);
            }
        }
        virtual ~DelaunayTriangulation3D() {}

        void Generate(const std::vector<VERTEXPTR> &input, bool assignIds = true, bool checkInput = false) override
        {
            this->Clear();

            auto dim = this->Dimension;
            if (input.size() <= dim + 1)
                return;

            auto count = input.size();
            for (auto i = 0; i < count; i++)
            {
                auto v = input[i]->GetPosition();
                v.resize(dim + 1);
                v[dim] = input[i]->SqrMagnitude() - input[i]->GetWeight();

                input[i]->mPosition = v;
            }

            Hull = std::make_shared<HDV::Hull::ConvexHull<VERTEXPTR>>(dim + 1);
            Hull->SetForPowerDiagram(true);
            Hull->Generate(input, assignIds, checkInput);

            for (auto i = 0; i < count; i++)
            {
                auto v = input[i]->GetPosition();
                v.resize(dim);
                input[i]->mPosition = v;
            }

            this->Vertices = Hull->GetVertices();
            KIRI_LOG_DEBUG("Hull->GetVertices size={0}; input size={1}", this->Vertices.size(), count);

            this->Centroid->mPosition[0] = Hull->GetCentroid()[0];
            this->Centroid->mPosition[1] = Hull->GetCentroid()[1];
            this->Centroid->mPosition[2] = Hull->GetCentroid()[2];

            count = Hull->GetSimplexs().size();

            for (auto i = 0; i < count; i++)
            {

                auto simplex = Hull->GetSimplexs()[i];

                if (simplex->Normals[dim] >= 0.0f)
                {
                    for (auto j = 0; j < simplex->Adjacent.size(); j++)
                    {
                        if (simplex->Adjacent[j] != nullptr)
                        {
                            simplex->Adjacent[j]->Remove(simplex);
                        }
                    }
                }
                else
                {
                    auto cell = CreateCell(simplex);
                    // cell.CircumCenter.Id = i;
                    this->Cells.emplace_back(cell);
                }
            }
        }

    private:
        std::vector<std::vector<double>> mMatrixBuffer;

        double MINOR(int r0, int r1, int r2, int c0, int c1, int c2)
        {
            return mMatrixBuffer[r0][c0] * (mMatrixBuffer[r1][c1] * mMatrixBuffer[r2][c2] - mMatrixBuffer[r2][c1] * mMatrixBuffer[r1][c2]) -
                   mMatrixBuffer[r0][c1] * (mMatrixBuffer[r1][c0] * mMatrixBuffer[r2][c2] - mMatrixBuffer[r2][c0] * mMatrixBuffer[r1][c2]) +
                   mMatrixBuffer[r0][c2] * (mMatrixBuffer[r1][c0] * mMatrixBuffer[r2][c1] - mMatrixBuffer[r2][c0] * mMatrixBuffer[r1][c1]);
        }

        double Determinant()
        {
            return (mMatrixBuffer[0][0] * MINOR(1, 2, 3, 1, 2, 3) -
                    mMatrixBuffer[0][1] * MINOR(1, 2, 3, 0, 2, 3) +
                    mMatrixBuffer[0][2] * MINOR(1, 2, 3, 0, 1, 3) -
                    mMatrixBuffer[0][3] * MINOR(1, 2, 3, 0, 1, 2));
        }

        std::shared_ptr<DelaunayCell<VERTEXPTR, VERTEX>> CreateCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> &simplex)
        {
            // From MathWorld: http://mathworld.wolfram.com/Circumsphere.html

            auto verts = simplex->Vertices;

            // x, y, z, 1
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][0] = verts[i]->mPosition[0];
                mMatrixBuffer[i][1] = verts[i]->mPosition[1];
                mMatrixBuffer[i][2] = verts[i]->mPosition[2];
                mMatrixBuffer[i][3] = 1;
            }
            auto a = Determinant();

            // size, y, z, 1
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][0] = verts[i]->SqrMagnitude() - verts[i]->GetWeight();
            }
            auto dx = Determinant();

            // size, x, z, 1
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][1] = verts[i]->mPosition[0];
            }
            auto dy = -Determinant();

            // size, x, y, 1
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][2] = verts[i]->mPosition[1];
            }
            auto dz = Determinant();

            // size, x, y, z
            for (auto i = 0; i < 4; i++)
            {
                mMatrixBuffer[i][3] = verts[i]->mPosition[2];
            }
            auto c = Determinant();

            auto s = 1.0 / (2.0 * a);
            auto radius = std::abs(s) * std::sqrtf(dx * dx + dy * dy + dz * dz - 4 * a * c);

            std::vector<double> circumCenter;
            circumCenter.assign(3, 0.0);
            circumCenter[0] = s * dx;
            circumCenter[1] = s * dy;
            circumCenter[2] = s * dz;

            return std::make_shared<DelaunayCell<VERTEXPTR, VERTEX>>(simplex, circumCenter, radius);
        }
    };

    typedef DelaunayTriangulation3D<HDV::Primitives::Vertex3Ptr, HDV::Primitives::Vertex3> DelaunayTriangulation3;

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_TRIANGULATION3_H_ */