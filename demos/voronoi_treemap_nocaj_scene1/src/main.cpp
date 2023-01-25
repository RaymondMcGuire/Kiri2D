/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-07 18:37:55
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-07 18:40:22
 * @FilePath: \Kiri2D\demos\voronoi_treemap_nocaj_scene1\src\main.cpp
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
    auto scene = std::make_shared<KiriScene2D<float>>((size_t)window_width, (size_t)window_height);
    auto renderer = std::make_shared<KiriRenderer2D<float>>(scene);

    // voronoi treemap config
    auto sampler_num = 100;
    auto scale_size = 200.0;
    auto voronoi_treemap = std::make_shared<VoronoiTreeMap::VoronoiNocajTreeMap>();

    // clip boundary: square
    auto boundary = std::make_shared<Voronoi::VoronoiPolygon2>();
    boundary->add(Vector2D(-scale_size, -scale_size));
    boundary->add(Vector2D(-scale_size, scale_size));
    boundary->add(Vector2D(scale_size, scale_size));
    boundary->add(Vector2D(scale_size, -scale_size));
    voronoi_treemap->setRootBoundary(boundary);

    // generate examples
    voronoi_treemap->GenExample();

    // visualization
    std::vector<KiriLine2<float>> lines;
    std::vector<KiriPoint2> points;
    std::vector<KiriLine2<float>> precompute_lines;
    std::vector<Vector2F> precompute_points;

    while (1)
    {
        // clear
        lines.clear();
        points.clear();
        precompute_lines.clear();
        precompute_points.clear();

        voronoi_treemap->computeIterate();

        // leaf nodes sites
        auto sites = voronoi_treemap->leafNodeSites();
        for (auto i = 0; i < sites.size(); i++)
        {
            auto site = sites[i];
            if (site->isBoundaryVertex())
                continue;

            auto cell_polygon = site->polygon();
            if (!cell_polygon)
                continue;

            for (auto j = 0; j < cell_polygon->positions().size(); j++)
            {
                auto vert = cell_polygon->positions()[j];
                auto vert1 = cell_polygon->positions()[(j + 1) % (cell_polygon->positions().size())];
                auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset, Vector2F(vert1.x, vert1.y) + offset);
                line.thick = 1.f;
                precompute_lines.emplace_back(line);
            }
            precompute_points.emplace_back(Vector2F(site->x(), site->y()));
        }

        // draw voronoi sites and cells
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
