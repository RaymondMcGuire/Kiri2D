/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-08 18:11:34
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-08 23:22:54
 * @FilePath: \Kiri2D\demos\proto_sphere_parallel\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d.h>
#include <kiri2d/hdv_toolkit/sdf/sdf2d.h>

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
  auto scene = std::make_shared<KiriScene2D>((size_t)window_width,
                                             (size_t)window_height);
  auto renderer = std::make_shared<KiriRenderer2D>(scene);

  // config
  auto scale = 450.f;

  // boundary
  auto boundary_polygon = std::make_shared<Voronoi::VoronoiPolygon2>();

  auto LoadPolygonFromXYFile = [](std::vector<Vector2F> &points, size_t &num,
                                  const char *filePath)
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
  String boundary_file_path =
      String(RESOURCES_PATH) + "alpha_shapes/" + boundary_file_name + ".xy";
  LoadPolygonFromXYFile(boundary2d_vertices, boundary_vertices,
                        boundary_file_path.c_str());

  for (size_t i = 0; i < boundary2d_vertices.size(); i++)
    boundary_polygon->add(Vector2D(boundary2d_vertices[i].x * scale,
                                   boundary2d_vertices[i].y * scale));

  auto boundary_bbox = boundary_polygon->bbox();
  auto offset = (Vector2F((size_t)window_width, (size_t)window_height) -
                 Vector2F(boundary_bbox.width(), boundary_bbox.height())) /
                2.f;

  // compute sdf
  auto poly_sdf = std::make_shared<SDF::PolygonSDF2D>(boundary_polygon, 1.0);
  poly_sdf->computeSDF();
  auto rnd_point = boundary_polygon->rndInnerPoint();
  auto dist = poly_sdf->getSDF(rnd_point);
  KIRI_LOG_DEBUG("dist={0}; min dist={1}", dist, boundary_polygon->computeMinDisInPoly(rnd_point));
  auto rnd_point_cir = KiriCircle2(Vector2F(rnd_point.x, rnd_point.y) + offset, Vector3F(0.f, 1.f, 1.f), abs(dist), false);
  auto rnd_point_p = KiriPoint2(Vector2F(rnd_point.x, rnd_point.y) + offset, Vector3F(1.f, 0.f, 0.f));
  // visualization
  std::vector<KiriLine2> lines;
  std::vector<KiriLine2> precompute_lines;

  for (size_t j = 0; j < boundary_polygon->positions().size(); j++)
  {
    auto vertices = boundary_polygon->positions()[j];
    auto vertices1 =
        boundary_polygon
            ->positions()[(j + 1) % (boundary_polygon->positions().size())];
    auto line = KiriLine2(Vector2F(vertices.x, vertices.y) + offset,
                          Vector2F(vertices1.x, vertices1.y) + offset);
    line.thick = 1.f;
    precompute_lines.emplace_back(line);
  }

  for (auto i = 0; i < precompute_lines.size(); ++i)
    lines.emplace_back(precompute_lines[i]);

  scene->addLines(lines);
  scene->addParticle(rnd_point_p);
  scene->addCircle(rnd_point_cir);

  renderer->drawCanvas();

  while (1)
  {
    // renderer->saveImages2File();
    cv::imshow("KIRI2D", renderer->canvas());
    cv::waitKey(5);
  }

  return 0;
}
