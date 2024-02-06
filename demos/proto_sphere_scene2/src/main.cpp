/*** 
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2023-04-13 10:54:32
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-04-13 10:57:48
 * @FilePath: \Kiri2D\demos\proto_sphere_scene2\src\main.cpp
 * @Description: 
 * @Copyright (c) 2023 by Xu.WANG, All Rights Reserved. 
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
    auto scene = std::make_shared<KiriScene2D<float>>((size_t)window_width, (size_t)window_height);
    auto renderer = std::make_shared<KiriRenderer2D<float>>(scene);

  // config
  auto scale = 450.f;

  // boundary
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

  auto boundary_bbox = boundary_polygon->bbox();
  auto offset = (Vector2F((size_t)window_width, (size_t)window_height) - Vector2F(boundary_bbox.width(), boundary_bbox.height())) / 2.f;

  // visualization
  std::vector<KiriLine2<float>> lines;
  std::vector<KiriPoint2<float>> points;
   std::vector<KiriCircle2<float>> circles;
   std::vector<KiriCircle2<float>> inserted_sphere;
  std::vector<KiriLine2<float>> precompute_lines;
  std::vector<Vector2F> precompute_points;

  for (size_t j = 0; j < boundary_polygon->positions().size(); j++)
  {
    auto vertices = boundary_polygon->positions()[j];
    auto vertices1 = boundary_polygon->positions()[(j + 1) % (boundary_polygon->positions().size())];
    auto line = KiriLine2(Vector2F(vertices.x, vertices.y) + offset, Vector2F(vertices1.x, vertices1.y) + offset);
    line.thick = 1.f;
    precompute_lines.emplace_back(line);
  }
    // proto sphere algo
    auto total_iter = 0;
    auto proto_sphere_packing = std::make_shared<ProtoSpherePacking2D>(boundary_polygon);

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
            inserted_sphere.emplace_back(KiriCircle2<float>(Vector2F(inserted_sphere_data[i].x, inserted_sphere_data[i].y) + offset, inserted_sphere_color_data[i], inserted_sphere_data[i].z, true));

        circles.emplace_back(KiriCircle2<float>(Vector2F(current_sphere.x, current_sphere.y) + offset, Vector3F(0.f, 1.f, 1.f), current_sphere_radius, false));
        precompute_points.emplace_back(Vector2F(current_sphere.x, current_sphere.y));

        for (auto i = 0; i < precompute_points.size(); i++)
            points.emplace_back(KiriPoint2(precompute_points[i] + offset, Vector3F(1.f, 0.f, 0.f)));

        for (auto i = 0; i < precompute_lines.size(); ++i)
            lines.emplace_back(precompute_lines[i]);

        for (auto i = 0; i < inserted_sphere.size(); i++)
            circles.emplace_back(inserted_sphere[i]);

        scene->AddLines(lines);
        scene->AddParticles(points);
        scene->AddCircles(circles);

        renderer->DrawCanvas();

        if (total_iter % 10 == 0)
            renderer->SaveImages2File();

        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);

        // clean canvas
        renderer->ClearCanvas();
        scene->Clear();

        total_iter++;
    }

    return 0;
}