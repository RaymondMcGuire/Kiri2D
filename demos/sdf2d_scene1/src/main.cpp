/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-11-30 15:29:43
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-30 15:44:27
 * @FilePath: \Kiri2D\demos\sdf2d_scene1\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#include <kiri2d.h>

using namespace KIRI2D;
using namespace HDV;

int main(int argc, char *argv[]) {
  // log system
  KiriLog::init();

  // scene renderer config
  auto window_height = 500.f;
  auto window_width = 500.f;
  auto scene = std::make_shared<KiriScene2D<float>>((size_t)window_width,
                                             (size_t)window_height);
  auto renderer = std::make_shared<KiriRenderer2D<float>>(scene);

  // config
  auto scale = 450.f;

  // boundary
  auto boundary_polygon = std::make_shared<Voronoi::VoronoiPolygon2>();

  auto LoadPolygonFromXYFile = [](std::vector<Vector2F> &points, size_t &num,
                                  const char *filePath) {
    std::ifstream file(filePath);
    file >> num;
    for (int i = 0; i < num; ++i) {
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

  // visualization
  std::vector<KiriLine2<float>> lines;
  std::vector<KiriLine2<float>> precompute_lines;

  for (size_t j = 0; j < boundary_polygon->positions().size(); j++) {
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

  scene->AddLines(lines);

   std::vector<KiriCircle2<float>> circles;
  auto sdf = std::make_shared<SDF::PolygonSDF2D>(boundary_polygon, 5.0);
  sdf->computeSDF();

  auto size = 10;
  for (auto i = 0; i < 50; i++) {
    auto p = boundary_polygon->rndInnerPointWithDistConstrain(size);

    std::vector<Vector3D> spheres;
    spheres.emplace_back(Vector3D(p.x, p.y, size));
    sdf->updateSDFWithSpheres(spheres);

    circles.emplace_back(KiriCircle2<float>(Vector2F(p.x, p.y) + offset,
                                     Vector3F(1.f, 0.f, 0.f), size, false));
  }

  auto points = sdf->placeGridPoints();
  for (int i = 0; i < points.size(); i++) {
    circles.emplace_back(
        KiriCircle2<float>(Vector2F(points[i].x, points[i].y) + offset,
                    Vector3F(0.f, 1.f, 1.f), 1, true));
  }
  scene->AddCircles(circles);

  renderer->DrawCanvas();
  renderer->SaveImages2File();
  while (1) {
    // renderer->SaveImages2File();
    cv::imshow("KIRI2D", renderer->GetCanvas());
    cv::waitKey(5);
  }

  return 0;
}
