/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-05-24 10:26:18
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-26 23:34:40
 * @FilePath: \Kiri2D\demos\convexhull2d_scene1\src\main.cpp
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
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
    auto scene = std::make_shared<KiriScene2D<float>>((size_t)window_width, (size_t)window_height);
    auto renderer = std::make_shared<KiriRenderer2D<float>>(scene);

    // convex hull config
    auto sampler_num = 100;
    auto scale_size = 200.0;
    auto convexhull2d = std::make_shared<Hull::ConvexHull2>();

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

    // compute convex hull
    convexhull2d->generate(sites);
    // sort simplexs
    auto simplexs = convexhull2d->computeSortSimplexsList();

    // visualization
    std::vector<KiriLine2<float>> lines;
    std::vector<KiriPoint2<float>> points;
    std::vector<KiriLine2<float>> precompute_lines;
    std::vector<Vector2F> precompute_points;

    for (auto i = 0; i < simplexs.size(); i++)
    {
        auto line = KiriLine2(Vector2F(simplexs[i].x, simplexs[i].y) + offset, Vector2F(simplexs[i].z, simplexs[i].w) + offset);
        line.thick = 1.f;
        precompute_lines.emplace_back(line);
        precompute_lines.emplace_back(line);
    }

    for (auto i = 0; i < sites.size(); i++)
        precompute_points.emplace_back(Vector2F(sites[i]->x(), sites[i]->y()));

    // draw convex hull
    for (auto i = 0; i < precompute_points.size(); i++)
        points.emplace_back(KiriPoint2(precompute_points[i] + offset, Vector3F(1.f, 0.f, 0.f)));

    for (auto i = 0; i < precompute_lines.size(); ++i)
        lines.emplace_back(precompute_lines[i]);

    scene->AddLines(lines);
    scene->AddParticles(points);

    renderer->DrawCanvas();

    while (1)
    {
        // renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
    }

    return 0;
}
