/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-08 18:11:34
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-09 16:38:12
 * @FilePath: \Kiri2D\demos\proto_sphere_parallel\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d.h>

using namespace KIRI2D;
using namespace PSPACK;
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

  // boundary: regular n-sided polygon
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

  // visualization
  std::vector<KiriLine2> lines;
  std::vector<KiriPoint2> points;
  std::vector<KiriCircle2> circles;
  std::vector<KiriCircle2> inserted_sphere;
  std::vector<KiriLine2> precompute_lines;
  std::vector<Vector2F> precompute_points;

  for (auto j = 0; j < boundary->positions().size(); j++)
  {
    auto vert = boundary->positions()[j];
    auto vert1 = boundary->positions()[(j + 1) % (boundary->positions().size())];
    auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset, Vector2F(vert1.x, vert1.y) + offset);
    line.thick = 1.f;
    precompute_lines.emplace_back(line);
  }

  // proto sphere algo
  auto total_iter = 0;
  auto proto_sphere_packing = std::make_shared<ProtoSpherePackingSDF2D>(boundary);

  while (1)
  {
    // clear
    lines.clear();
    points.clear();
    circles.clear();
    precompute_points.clear();

    proto_sphere_packing->convergePrototype();

    auto current_sphere = proto_sphere_packing->currentSphere();
    auto current_sphere_radius = proto_sphere_packing->currentSphereRadius();
    auto inserted_sphere_data = proto_sphere_packing->insertedSpheres();
    auto inserted_sphere_color_data = proto_sphere_packing->insertedSpheresColor();

    for (auto i = 0; i < inserted_sphere_data.size(); i++)
      inserted_sphere.emplace_back(KiriCircle2(Vector2F(inserted_sphere_data[i].x, inserted_sphere_data[i].y) + offset, inserted_sphere_color_data[i], inserted_sphere_data[i].z, true));

    circles.emplace_back(KiriCircle2(Vector2F(current_sphere.x, current_sphere.y) + offset, Vector3F(0.f, 1.f, 1.f), current_sphere_radius, false));
    precompute_points.emplace_back(Vector2F(current_sphere.x, current_sphere.y));

    for (auto i = 0; i < precompute_points.size(); i++)
      points.emplace_back(KiriPoint2(precompute_points[i] + offset, Vector3F(1.f, 0.f, 0.f)));

    for (auto i = 0; i < precompute_lines.size(); ++i)
      lines.emplace_back(precompute_lines[i]);

    for (auto i = 0; i < inserted_sphere.size(); i++)
      circles.emplace_back(inserted_sphere[i]);

    scene->addLines(lines);
    scene->addParticles(points);
    scene->addCircles(circles);

    renderer->drawCanvas();

    if (total_iter % 10 == 0)
      renderer->saveImages2File();

    cv::imshow("KIRI2D", renderer->canvas());
    cv::waitKey(5);

    // clean canvas
    renderer->clearCanvas();
    scene->clear();

    total_iter++;
  }

  return 0;
}
