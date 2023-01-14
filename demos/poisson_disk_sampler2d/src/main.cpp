/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-05-31 09:45:40
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-05-31 09:48:44
 * @FilePath: \Kiri2D\demos\poisson_disk_sampler2d\src\main.cpp
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

    // scene renderer config
    auto window_height = 500.f;
    auto window_width = 500.f;
    auto scene = std::make_shared<KiriScene2D>((size_t)window_width, (size_t)window_height);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    // poisson disk sampler config
    auto scale = 450.f;
    auto radius = 10.f;
    auto sampler = std::make_shared<Sampler::PoissonDiskSampler2D>();

    // clip boundary
    auto boundary_polygon = std::make_shared<Voronoi::VoronoiPolygon2>();

    auto LoadPolygonFromXYFile = [](std::vector<Vector2F> &points, size_t &num, const char *filePath)
    {
        std::ifstream file(filePath);
        file >> num;
        for (int i = 0; i < num; ++i)
        {
            Vector2F xy;
            file >> xy.x >> xy.y;
            points.emplace_back(xy);
        }

        file.close();
    };

    std::vector<Vector2F> boundary2d_vertices;
    size_t boundary_vertices;
    auto boundary_file_name = "bunny";
    String boundary_file_path = String(RESOURCES_PATH) + "alpha_shapes/" + boundary_file_name + ".xy";
    LoadPolygonFromXYFile(boundary2d_vertices, boundary_vertices, boundary_file_path.c_str());

    for (size_t i = 0; i < boundary2d_vertices.size(); i++)
        boundary_polygon->add(Vector2D(boundary2d_vertices[i].x * scale, boundary2d_vertices[i].y * scale));

    // init sampler
    auto boundary_bbox = boundary_polygon->bbox();

    auto offset = (Vector2F((size_t)window_width, (size_t)window_height) - Vector2F(boundary_bbox.width(), boundary_bbox.height())) / 2.f;
    sampler->initUniRadiiSampler(radius, Vector2F(boundary_bbox.width(), boundary_bbox.height()));

    // visualization
    std::vector<KiriLine2> lines;
    std::vector<KiriPoint2> points;
    std::vector<KiriLine2> precompute_lines;
    std::vector<Vector2F> precompute_points;

    while (1)
    {
        // clear
        lines.clear();
        points.clear();
        precompute_lines.clear();
        precompute_points.clear();

        // sampling uni radii points
        auto uni_points = sampler->generateUniRadii(radius, Vector2F(boundary_bbox.width(), boundary_bbox.height()));

        // record points and boundary
        for (size_t j = 0; j < boundary_polygon->positions().size(); j++)
        {
            auto vertices = boundary_polygon->positions()[j];
            auto vertices1 = boundary_polygon->positions()[(j + 1) % (boundary_polygon->positions().size())];
            auto line = KiriLine2(Vector2F(vertices.x, vertices.y) + offset, Vector2F(vertices1.x, vertices1.y) + offset);
            line.thick = 1.f;
            precompute_lines.emplace_back(line);
        }

        for (auto i = 0; i < uni_points.size(); i++)
            if (boundary_polygon->contains(Vector2D(uni_points[i].x, uni_points[i].y)))
                precompute_points.emplace_back(uni_points[i]);

        // draw
        for (auto i = 0; i < precompute_points.size(); i++)
            points.emplace_back(KiriPoint2(precompute_points[i] + offset, Vector3F(1.f, 0.f, 0.f)));

        for (auto i = 0; i < precompute_lines.size(); ++i)
            lines.emplace_back(precompute_lines[i]);

        scene->AddLines(lines);
        scene->AddParticles(points);

        renderer->DrawCanvas();
        // renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);

        // clean canvas
        renderer->ClearCanvas();
        scene->Clear();
    }

    return 0;
}
