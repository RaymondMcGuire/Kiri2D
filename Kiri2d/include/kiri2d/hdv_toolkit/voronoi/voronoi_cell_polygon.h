/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2021-12-29 15:01:19
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_VORONOI_CELL_POLYGON_H_
#define _HDV_VORONOI_CELL_POLYGON_H_

#pragma once

#include <kiri2d/straight_skeleton/sskel_slav.h>
#include <kiri2d/hdv_toolkit/delaunay/delaunay_cell.h>
#include <Mathematics/DistPointTriangle.h>
namespace HDV::Voronoi
{
    template <typename VERTEXPTR = HDV::Primitives::VertexPtr, typename VERTEX = HDV::Primitives::Vertex>
    class VoronoiCellPolygon
    {
    public:
        explicit VoronoiCellPolygon() {}
        virtual ~VoronoiCellPolygon() {}

        void AddVert2(Vector2D vert)
        {
            Positions.emplace_back(vert);
            BBox.merge(vert);
        }

        void UpdateBBox()
        {
            BBox.reset();
            for (auto i = 0; i < Positions.size(); i++)
                BBox.merge(Positions[i]);
        }

        bool CheckBBox()
        {
            if (BBox.isEmpty())
            {
                if (!Positions.empty())
                {
                    UpdateBBox();
                    return true;
                }
                else
                {
                    KIRI_LOG_ERROR("Contains:: No polygon data!!");
                    return false;
                }
            }
            return true;
        }

        Vector2D GetRndInnerPoint()
        {
            if (!CheckBBox())
            {
                KIRI_LOG_ERROR("GetRndInnerPoint:: Get inner point failed!!");
                return Vector2D(0.0);
            }

            Vector2D inner;
            std::random_device seedGen;
            std::default_random_engine rndEngine(seedGen());
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            do
            {
                inner = BBox.LowestPoint + Vector2D(dist(rndEngine) * BBox.width(), dist(rndEngine) * BBox.height());
            } while (!Contains(inner));

            return inner;
        }

        bool Contains(const Vector2D &v)
        {
            if (!CheckBBox())
                return false;

            if (!BBox.contains(v))
                return false;

            bool contains = false;
            for (size_t i = 0, j = Positions.size() - 1; i < Positions.size(); j = i++)
            {
                auto verti = Positions[i];
                auto vertj = Positions[j];
                if ((((verti.y <= v.y) && (v.y < vertj.y) || ((vertj.y <= v.y) && (v.y < verti.y))) && (v.x < (vertj.x - verti.x) * (v.y - verti.y) / (vertj.y - verti.y) + verti.x)))
                    contains = !contains;
            }
            return contains;
        }

        double GetArea()
        {
            auto area = 0.0;
            if (!Positions.empty())
                for (size_t i = 0; i < Positions.size(); i++)
                    area += Positions[i].cross(Positions[(i + 1) % Positions.size()]);

            return std::abs(area * 0.5);
        }

        Vector2D GetCentroid()
        {
            std::vector<Vector2D> tmpPolygonVertices(Positions);

            auto first = tmpPolygonVertices[0], last = tmpPolygonVertices[tmpPolygonVertices.size() - 1];
            if (first.x != last.x || first.y != last.y)
                tmpPolygonVertices.emplace_back(first);

            auto twicearea = 0.0,
                 x = 0.0, y = 0.0, f = 0.0;
            auto nPts = tmpPolygonVertices.size();
            Vector2D p1, p2;

            for (size_t i = 0, j = nPts - 1; i < nPts; j = i++)
            {
                p1 = tmpPolygonVertices[i];
                p2 = tmpPolygonVertices[j];
                f = p1.x * p2.y - p2.x * p1.y;
                twicearea += f;
                x += (p1.x + p2.x) * f;
                y += (p1.y + p2.y) * f;
            }
            f = twicearea * 3.0;

            return Vector2D(x / f, y / f);
        }

        bool IsClockwise(const std::vector<Vector4F> &poly)
        {
            auto a = 0.f;
            auto polySize = poly.size();
            for (size_t i = 0; i < polySize; i++)
            {
                auto s = Vector2F(poly[i].x, poly[i].y);
                auto e = Vector2F(poly[i].z, poly[i].w);
                a += (e.x - s.x) * (e.y + s.y);
            }

            return a < 0.f;
        }

        void ComputeSSkel1998Convex()
        {
            mSkeletons.clear();

            std::vector<Vector4F> poly;
            std::vector<Vector2F> rPolyVert;

            for (size_t i = 0; i < Positions.size(); i++)
            {
                auto v1 = Positions[i];
                auto v2 = Positions[(i + 1) % Positions.size()];
                poly.emplace_back(Vector4F(v1.x, v1.y, v2.x, v2.y));
                rPolyVert.emplace_back(Vector2F(v1.x, v1.y));
            }

            if (IsClockwise(poly))
                std::reverse(rPolyVert.begin(), rPolyVert.end());

            auto sskel_convex = std::make_shared<KIRI2D::SSKEL::SSkelSLAV>(rPolyVert);

            auto skeletons = sskel_convex->GetSkeletons();

            for (size_t i = 0; i < skeletons.size(); i++)
            {
                auto [intersect, sinks] = skeletons[i];
                for (size_t j = 0; j < sinks.size(); j++)
                    mSkeletons.emplace_back(Vector4F(intersect.x, intersect.y, sinks[j].x, sinks[j].y));
            }
        }

        double MinDis2LineSegment2(Vector2D v, Vector2D w, Vector2D p)
        {
            const double l2 = v.distanceSquaredTo(w);
            if (l2 == 0.f)
                return p.distanceTo(v);

            const double t = std::clamp(((p - v).dot(w - v)) / l2, 0.0, 1.0);
            const Vector2D projection = v + t * (w - v);
            return p.distanceTo(projection);
        }

        double ComputeMinDisInPoly(const Vector2D &p)
        {
            auto minDis = std::numeric_limits<double>::max();
            for (size_t i = 0; i < Positions.size(); i++)
                minDis = std::min(minDis, MinDis2LineSegment2(Positions[i], Positions[(i + 1) % Positions.size()], p));
            return minDis;
        }

        Vector3D ComputeMICByStraightSkeleton()
        {
            auto maxCirVec = Vector2D(0.0);
            auto maxCirRad = std::numeric_limits<double>::min();
            if (!mSkeletons.empty())
            {
                for (size_t i = 0; i < mSkeletons.size(); i++)
                {
                    auto v1 = Vector2D(mSkeletons[i].x, mSkeletons[i].y);
                    auto v2 = Vector2D(mSkeletons[i].z, mSkeletons[i].w);

                    if (Contains(v1))
                    {
                        auto minDis = ComputeMinDisInPoly(v1);
                        if (minDis > maxCirRad)
                        {
                            maxCirRad = minDis;
                            maxCirVec = v1;
                        }
                    }

                    if (Contains(v2))
                    {
                        auto minDis = ComputeMinDisInPoly(v2);
                        if (minDis > maxCirRad)
                        {
                            maxCirRad = minDis;
                            maxCirVec = v2;
                        }
                    }
                }
            }
            return Vector3D(maxCirVec.x, maxCirVec.y, maxCirRad);
        }

        BoundingBox2D BBox;
        std::vector<Vector2D> Positions;
        std::vector<Vector4F> mSkeletons;
    };

    class VoronoiPolygon3
    {
    public:
        explicit VoronoiPolygon3() {}
        virtual ~VoronoiPolygon3() {}

        void AddVert3(Vector3D vert)
        {
            Positions.emplace_back(vert);
            BBox.merge(vert);
        }

        void UpdateBBox()
        {
            BBox.reset();
            for (auto i = 0; i < Positions.size(); i++)
                BBox.merge(Positions[i]);
        }

        double ComputeVolume(Vector3D a, Vector3D b, Vector3D c)
        {
            return b.cross(c).dot(a) / 6.0;
        }

        double ComputeMinDisInPoly(Vector3D p)
        {
            // std::vector<tmd::Vec3d> vertices;
            // std::vector<std::array<int, 3>> triangles;

            // for (auto i = 0; i < Positions.size(); i++)
            // {
            //     vertices.emplace_back(tmd::Vec3d(Positions[i].x, Positions[i].y, Positions[i].z));
            // }

            // for (auto i = 0; i < Indices.size() / 3; i++)
            // {
            //     std::array<int, 3> ind = {Indices[i * 3], Indices[i * 3 + 1], Indices[i * 3 + 2]};
            //     triangles.emplace_back(ind);
            // }

            // // Initialize TriangleMeshDistance
            // tmd::TriangleMeshDistance mesh_distance(vertices, triangles);

            // // Query TriangleMeshDistance
            // tmd::Result result = mesh_distance.signed_distance({p.x, p.y, p.z});
            // auto minDis = std::abs(result.distance);
            // KIRI_LOG_DEBUG("minDis={0}", minDis);

            // KIRI_LOG_DEBUG("---------------------------------------------");
            // IndexOffset = IsLoadedFromObj ? 1 : 0;
            auto minDis = std::numeric_limits<double>::max();

            for (size_t j = 0; j < Indices.size() / 3; j++)
            {
                auto a = Positions[Indices[j * 3] - IndexOffset];
                auto b = Positions[Indices[j * 3 + 1] - IndexOffset];
                auto c = Positions[Indices[j * 3 + 2] - IndexOffset];
                // // if indices loaded from a .obj file, index need -1
                // auto l = equation_plane(a, b, c);
                // auto dis = shortest_distance(p, l);

                // auto da = a.distanceTo(p), db = b.distanceTo(p), dc = c.distanceTo(p);
                // if (minDis > dis)
                // {
                //     KIRI_LOG_DEBUG("a={0},{1},{2};b={3},{4},{5};c={6},{7},{8}", a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
                //     KIRI_LOG_DEBUG("current={0};p={1},{2},{3};da={4},db={5},dc={6}", dis, p.x, p.y, p.z, da, db, dc);
                // }

                gte::Vector<3, double> vec3 = {{p.x, p.y, p.z}};
                gte::Vector<3, double> a3 = {{a.x, a.y, a.z}}, b3 = gte::Vector<3, double>{{b.x, b.y, b.z}}, c3 = gte::Vector<3, double>{{c.x, c.y, c.z}};
                gte::Triangle<3, double> tri3(a3, b3, c3);

                gte::DCPPoint3Triangle3<double> dpt3;
                auto res = dpt3(vec3, tri3);
                auto dis = res.distance * 2.0;
                minDis = std::min(minDis, dis);
            }

            KIRI_LOG_DEBUG("minDis={0}", minDis);
            return minDis;
        }

        double GetVolume()
        {
            IndexOffset = IsLoadedFromObj ? 1 : 0;

            auto V = 0.0;
            for (size_t j = 0; j < Indices.size() / 3; j++)
            {
                // if indices loaded from a .obj file, index need -1
                auto a = Positions[Indices[j * 3] - IndexOffset];
                auto b = Positions[Indices[j * 3 + 1] - IndexOffset];
                auto c = Positions[Indices[j * 3 + 2] - IndexOffset];

                V += ComputeVolume(a, b, c);
            }

            if (V < 0.0)
                KIRI_LOG_ERROR("Volume is negnative!!!");

            return V;
        }

        Vector3D GetCentroid()
        {
            IndexOffset = IsLoadedFromObj ? 1 : 0;

            auto volume = 0.0;
            Vector3D centroid;
            for (size_t j = 0; j < Indices.size() / 3; j++)
            {

                // if indices loaded from a .obj file, index need -1
                auto a = Positions[Indices[j * 3] - IndexOffset];
                auto b = Positions[Indices[j * 3 + 1] - IndexOffset];
                auto c = Positions[Indices[j * 3 + 2] - IndexOffset];

                auto v = ComputeVolume(a, b, c);
                volume += v;
                centroid += v * (a + b + c) / 4.0;
            }

            if (centroid.x != centroid.x || centroid.y != centroid.y || centroid.z != centroid.z)
                KIRI_LOG_ERROR("error centroid!!!!");

            return centroid / volume;
        }

        bool IsLoadedFromObj = false;
        int IndexOffset = 0;
        BoundingBox3D BBox;
        std::vector<Vector3D> Positions;
        std::vector<Vector3D> Normals;
        std::vector<int> Indices;
    };

} // namespace HDV::Voronoi

#endif /* _HDV_VORONOI_CELL_POLYGON_H_ */