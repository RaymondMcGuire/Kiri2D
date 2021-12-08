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
    template <typename VERTEX = HDV::Primitives::VertexPtr>
    class DelaunayTriangulation2D : public DelaunayTriangulation<VERTEX>
    {
    public:
        explicit DelaunayTriangulation2D() : DelaunayTriangulation(2)
        {
            mMatrixBuffer.assign(3, std::vector<float>());
            for (size_t i = 0; i < mMatrixBuffer.size(); i++)
            {
                mMatrixBuffer[i].assign(3, 0.f);
            }
        }
        virtual ~DelaunayTriangulation2D() noexcept {}

        void Generate(const std::vector<VERTEX> &input, bool assignIds = true, bool checkInput = false) override
        {
            Clear();

            if (input.size() <= Dimension + 1)
                return;

            auto count = input.size();
            for (auto i = 0; i < count; i++)
            {
                auto v = input[i]->GetPosition();
                v.resize(Dimension + 1);
                v[Dimension] = input[i]->SqrMagnitude();

                input[i]->mPosition = v;
            }

            auto hull = std::make_shared<HDV::Hull::ConvexHull<VERTEX>>(Dimension + 1);
            hull->Generate(input, assignIds, checkInput);

            for (auto i = 0; i < count; i++)
            {
                auto v = input[i]->GetPosition();
                v.resize(Dimension);
                input[i]->mPosition = v;
            }

            Vertices = hull->GetVertices();
            Centroid[0] = hull->GetCentroid()[0];
            Centroid[1] = hull->GetCentroid()[1];

            count = hull->GetSimplexs().size();

            for (auto i = 0; i < count; i++)
            {

                auto simplex = hull->GetSimplexs()[i];

                if (simplex->Normals[Dimension] >= 0.0f)
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
                    Cells.emplace_back(cell);
                }
            }
        }

    private:
        std::vector<std::vector<float>> mMatrixBuffer;

        float Determinant()
        {
            auto fCofactor00 = mMatrixBuffer[1][1] * mMatrixBuffer[2][2] - mMatrixBuffer[1][2] * mMatrixBuffer[2][1];
            auto fCofactor10 = mMatrixBuffer[1][2] * mMatrixBuffer[2][0] - mMatrixBuffer[1][0] * mMatrixBuffer[2][2];
            auto fCofactor20 = mMatrixBuffer[1][0] * mMatrixBuffer[2][1] - mMatrixBuffer[1][1] * mMatrixBuffer[2][0];

            auto fDet = mMatrixBuffer[0][0] * fCofactor00 + mMatrixBuffer[0][1] * fCofactor10 + mMatrixBuffer[0][2] * fCofactor20;

            return fDet;
        }

        std::shared_ptr<DelaunayCell<VERTEX>> CreateCell(const std::shared_ptr<HDV::Primitives::Simplex<VERTEX>> &simplex)
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
                mMatrixBuffer[i][0] = verts[i]->SqrMagnitude();
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

            auto s = -1.f / (2.f * a);

            std::vector<float> circumCenter;
            circumCenter.assign(2, 0.f);
            circumCenter[0] = s * dx;
            circumCenter[1] = s * dy;

            auto radius = std::abs(s) * std::sqrtf(dx * dx + dy * dy - 4.f * a * c);

            return std::make_shared<DelaunayCell<VERTEX>>(simplex, circumCenter, radius);
        }
    };

    typedef DelaunayTriangulation2D<HDV::Primitives::Vertex2Ptr> DelaunayTriangulation2;

} // namespace HDV::Delaunay

#endif /* _HDV_DELAUNAY_TRIANGULATION2_H_ */