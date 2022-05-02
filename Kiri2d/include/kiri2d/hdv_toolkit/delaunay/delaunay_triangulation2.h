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
        explicit DelaunayTriangulation2D() : DelaunayTriangulation<VERTEXPTR, VERTEX>(2)
        {
            mMatrixBuffer.assign(3, std::vector<double>());
            for (auto i = 0; i < mMatrixBuffer.size(); i++)
            {
                mMatrixBuffer[i].assign(3, 0.0);
            }
        }
        virtual ~DelaunayTriangulation2D() {}

        void Generate(const std::vector<VERTEXPTR> &input, bool assignIds = true, bool checkInput = false) override
        {

            this->Clear();
            auto dim = this->Dimension;

            if (input.size() <= dim + 1)
                return;

            auto count = input.size();

            //#pragma omp parallel for
            for (auto i = 0; i < count; i++)
            {
                auto v = input[i]->GetPosition();
                v.resize(dim + 1);
                v[dim] = input[i]->SqrMagnitude() - input[i]->GetWeight();
                // KIRI_LOG_DEBUG("weight={0}", input[i]->GetWeight());
                input[i]->mPosition = v;
            }

            Hull = std::make_shared<HDV::Hull::ConvexHull<VERTEXPTR>>(dim + 1);
            Hull->SetForPowerDiagram(true);
            Hull->Generate(input, assignIds, checkInput);

            //#pragma omp parallel for
            for (auto i = 0; i < count; i++)
            {
                auto v = input[i]->GetPosition();
                v.resize(dim);
                input[i]->mPosition = v;
            }

            this->Vertices = Hull->GetVertices();

            this->Centroid->mPosition[0] = Hull->GetCentroid()[0];
            this->Centroid->mPosition[1] = Hull->GetCentroid()[1];

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

        double Determinant()
        {
            auto fCofactor00 = mMatrixBuffer[1][1] * mMatrixBuffer[2][2] - mMatrixBuffer[1][2] * mMatrixBuffer[2][1];
            auto fCofactor10 = mMatrixBuffer[1][2] * mMatrixBuffer[2][0] - mMatrixBuffer[1][0] * mMatrixBuffer[2][2];
            auto fCofactor20 = mMatrixBuffer[1][0] * mMatrixBuffer[2][1] - mMatrixBuffer[1][1] * mMatrixBuffer[2][0];

            auto fDet = mMatrixBuffer[0][0] * fCofactor00 + mMatrixBuffer[0][1] * fCofactor10 + mMatrixBuffer[0][2] * fCofactor20;

            return fDet;
        }

        std::shared_ptr<DelaunayCell<VERTEXPTR, VERTEX>> CreateCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEXPTR>> &simplex)
        {
            // From MathWorld: http://mathworld.wolfram.com/Circumcircle.html

            auto verts = simplex->Vertices;

            // x, y, 1
            for (auto i = 0; i < 3; i++)
            {
                mMatrixBuffer[i][0] = verts[i]->mPosition[0];
                mMatrixBuffer[i][1] = verts[i]->mPosition[1];
                mMatrixBuffer[i][2] = 1;
            }

            auto a = Determinant();

            // size, y, 1
            for (auto i = 0; i < 3; i++)
            {
                mMatrixBuffer[i][0] = verts[i]->SqrMagnitude() - verts[i]->GetWeight();
            }

            auto dx = -Determinant();

            // size, x, 1
            for (auto i = 0; i < 3; i++)
            {
                mMatrixBuffer[i][1] = verts[i]->mPosition[0];
            }

            auto dy = Determinant();

            // size, x, y
            for (auto i = 0; i < 3; i++)
            {
                mMatrixBuffer[i][2] = verts[i]->mPosition[1];
            }
            auto c = -Determinant();

            auto s = -1.0 / (2.0 * a);

            std::vector<double> circumCenter;
            circumCenter.assign(2, 0.0);
            circumCenter[0] = s * dx;
            circumCenter[1] = s * dy;

            auto radius = std::abs(s) * std::sqrtf(dx * dx + dy * dy - 4.0 * a * c);

            return std::make_shared<DelaunayCell<VERTEXPTR, VERTEX>>(simplex, circumCenter, radius);
        }
    };

    typedef DelaunayTriangulation2D<HDV::Primitives::Vertex2Ptr, HDV::Primitives::Vertex2> DelaunayTriangulation2;

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_TRIANGULATION2_H_ */