/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-03 10:28:58
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-03 10:36:55
 * @FilePath: \Kiri2D\demos\poisson_disk_multi_sampler2d\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#include <kiri2d.h>

using namespace KIRI2D;
using namespace HDV;

int main(int argc, char *argv[])
{
    // log system
    KiriLog::init();

    auto alpha_shapes_name = std::vector<String>{"bunny", "alligator", "beast", "cheburashka", "cow", "homer", "horse", "lucy", "nefertiti", "spot", "teapot", "woody", "xyzrgb_dragon"};

    for (auto boundary_file_name : alpha_shapes_name)
    {

        // poisson disk sampler config
        auto scale = 450.f;
        auto sampler = std::make_shared<Sampler::PoissonDiskSampler2D>();

        // clip boundary
        auto boundary_polygon = std::make_shared<Voronoi::VoronoiPolygon2>();

        auto LoadPolygonFromXYFile = [](std::vector<Vector2F> &circles, size_t &num, const char *filePath)
        {
            std::ifstream file(filePath);
            file >> num;
            for (int i = 0; i < num; ++i)
            {
                Vector2F xy;
                file >> xy.x >> xy.y;
                circles.emplace_back(xy);
            }

            file.close();
        };

        std::vector<Vector2F> boundary2d_vertices;
        size_t boundary_vertices;
        String boundary_file_path = String(RESOURCES_PATH) + "alpha_shapes/" + boundary_file_name + ".xy";
        LoadPolygonFromXYFile(boundary2d_vertices, boundary_vertices, boundary_file_path.c_str());

        for (size_t i = 0; i < boundary2d_vertices.size(); i++)
            boundary_polygon->add(Vector2D(boundary2d_vertices[i].x * scale, boundary2d_vertices[i].y * scale));

        // init sampler
        auto boundary_bbox = boundary_polygon->bbox();

        auto min_radius = 2.f;
        auto max_radius = 15.f;

        std::vector<float> radius_range;
        radius_range.push_back(min_radius);
        radius_range.push_back(3.f);
        radius_range.push_back(8.f);
        radius_range.push_back(max_radius);

        std::vector<float> radius_prob;
        radius_prob.push_back(0.5f);
        radius_prob.push_back(0.4f);
        radius_prob.push_back(0.1f);

        sampler->initMultiRadiiSampler(min_radius, max_radius, Vector2F(boundary_bbox.width(), boundary_bbox.height()));

        std::vector<Vector3F> multi_points;
        while (!sampler->sampledFinished())
        {
            multi_points = sampler->generateMultiRadii(radius_range, radius_prob, Vector2F(boundary_bbox.width(), boundary_bbox.height()));
        }

        // rescale size
        for (auto i = 0; i < multi_points.size() - 1; i++)
        {
            for (auto j = i + 1; j < multi_points.size(); j++)
            {
                auto dist = (Vector2F(multi_points[i].x, multi_points[i].y) - Vector2F(multi_points[j].x, multi_points[j].y)).length();
                if (dist < (multi_points[i].z + multi_points[j].z))
                {
                    auto pen_dist = (multi_points[i].z + multi_points[j].z) - dist;

                    if (pen_dist > 0)
                    {
                        multi_points[i].z -= dist / 2;
                        multi_points[j].z -= dist / 2;
                    }
                    else
                    {
                        if (multi_points[i].z > multi_points[j].z)
                            multi_points[j].z = 0.f;
                    }

                    if (multi_points[i].z < 0)
                        multi_points[i].z = 0;

                    if (multi_points[j].z < 0)
                        multi_points[j].z = 0;
                }
            }
        }

        // compute porosity
        auto total_area = 0.0;
        for (auto i = 0; i < multi_points.size(); i++)
            total_area += KIRI_PI<float>() * multi_points[i].z * multi_points[i].z;

        auto porosity = (boundary_polygon->area() - total_area) / boundary_polygon->area();

        KIRI_LOG_DEBUG("boundary_file_name={0}; porosity={1}", boundary_file_name, porosity);
    }

    return 0;
}
