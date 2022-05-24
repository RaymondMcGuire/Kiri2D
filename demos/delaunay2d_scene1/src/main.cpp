/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-05-24 10:00:54
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-05-24 10:09:17
 * @FilePath: \Kiri2D\demos\delaunay2d_scene1\src\main.cpp
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
    auto offset = Vector2F((size_t)window_width, (size_t)window_height) / 2.f;
    auto scene = std::make_shared<KiriScene2D>((size_t)window_width, (size_t)window_height);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    // delaunay triangulation config
    auto sampler_num = 100;
    auto scale_size = 200.0;
    auto delaunay2d = std::make_shared<Delaunay::DelaunayTriangulation2>();

    // boundary: square
    auto boundary = std::make_shared<Voronoi::VoronoiPolygon2>();
    boundary->add(Vector2D(-scale_size, -scale_size));
    boundary->add(Vector2D(-scale_size, scale_size));
    boundary->add(Vector2D(scale_size, scale_size));
    boundary->add(Vector2D(scale_size, -scale_size));

    std::vector<HDV::Primitives::Vertex2Ptr> sites;
    for (auto i = 0; i < sampler_num; i++)
    {
        auto rnd_point = boundary->rndInnerPoint();
        sites.emplace_back(std::make_shared<HDV::Primitives::Vertex2>(rnd_point.x, rnd_point.y));
    }

    // compute delaunay triangle
    delaunay2d->generate(sites);

    // visualization
    std::vector<KiriLine2> lines;
    std::vector<KiriLine2> precompute_lines;

    // delaunay2d cells
    auto cells = delaunay2d->cells();
    for (auto i = 0; i < cells.size(); i++)
    {
        auto vertices = cells[i]->simplex()->vertices();
        auto center = cells[i]->circumCenter();
        for (auto j = 0; j < vertices.size(); j++)
        {
            auto vert = vertices[j];
            auto vert1 = vertices[(j + 1) % (vertices.size())];
            auto line = KiriLine2(Vector2F(vert->x(), vert->y()) + offset, Vector2F(vert1->x(), vert1->y()) + offset);
            line.thick = 1.f;
            precompute_lines.emplace_back(line);
        }
    }

    // draw delaunay2d triangles
    for (auto i = 0; i < precompute_lines.size(); ++i)
        lines.emplace_back(precompute_lines[i]);

    scene->AddLines(lines);

    renderer->DrawCanvas();

    while (1)
    {
        // renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
    }

    return 0;
}
