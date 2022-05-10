/***
 * @Author: Xu.WANG
 * @Date: 2022-05-10 18:25:28
 * @LastEditTime: 2022-05-10 18:31:03
 * @LastEditors: Xu.WANG
 * @Description:
 */

#include <kiri2d.h>

using namespace KIRI2D;
using namespace HDV;

int main(int argc, char *argv[])
{
    // log system
    KIRI::KiriLog::Init();

    // scene renderer config
    auto window_height = 500.f;
    auto window_width = 500.f;
    auto offset = Vector2F(window_width, window_height) / 2.f;
    auto scene = std::make_shared<KiriScene2D>((size_t)window_width, (size_t)window_height);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    // voronoi diagram config
    auto sampler_num = 100;
    auto voronoi2d = std::make_shared<Voronoi::PowerDiagram2D>();

    // clip boundary: regular n-sided polygon
    auto side_num = 8;
    auto side_radius = 200.0;

    auto boundary = std::make_shared<Voronoi::VoronoiPolygon2>();
    for (auto i = 0; i < side_num; i++)
    {
        auto angle = 2.0 * KIRI_PI<double>() * (i * 1.0 / side_num);
        auto rotate = KIRI_PI<double>() / side_num;
        auto y = std::sin(angle + rotate) * side_radius;
        auto x = std::cos(angle + rotate) * side_radius;
        boundary->add(Vector2D(x, y));
    }
    voronoi2d->setBoundary(boundary);

    // generate random sites
    voronoi2d->generateRndSites(sampler_num);
    voronoi2d->compute();

    // visualization
    std::vector<KiriLine2> lines;
    std::vector<KiriPoint2> points;
    std::vector<KiriLine2> precompute_lines;
    std::vector<Vector2F> precompute_points;

    // voronoi sites
    auto sites = voronoi2d->sites();
    for (size_t i = 0; i < sites.size(); i++)
    {
        auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
        if (site->isBoundaryVertex())
            continue;

        auto cellpolygon = site->polygon();
        for (size_t j = 0; j < cellpolygon->positions().size(); j++)
        {
            auto vert = cellpolygon->positions()[j];
            auto vert1 = cellpolygon->positions()[(j + 1) % (cellpolygon->positions().size())];
            auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset, Vector2F(vert1.x, vert1.y) + offset);
            line.thick = 1.f;
            precompute_lines.emplace_back(line);
        }
        precompute_points.emplace_back(Vector2F(site->x(), site->y()));
    }

    // draw voronoi sites and cells
    for (size_t i = 0; i < precompute_points.size(); i++)
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
