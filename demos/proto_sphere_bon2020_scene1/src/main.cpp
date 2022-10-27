/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-10-27 12:54:43
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-27 13:20:29
 * @FilePath: \Kiri2D\demos\proto_sphere_bon2020_scene1\src\main.cpp
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
  auto offset = Vector2F((size_t)window_width, (size_t)window_height) / 2.f;
  auto scene = std::make_shared<KiriScene2D>((size_t)window_width,
                                             (size_t)window_height);
  auto renderer = std::make_shared<KiriRenderer2D>(scene);

  // boundary: regular n-sided polygon
  auto side_num = 8;
  auto side_radius = 200.0;

  auto boundary = std::make_shared<Voronoi::VoronoiPolygon2>();
  for (auto i = 0; i < side_num; i++) {
    auto angle = 2.0 * KIRI_PI<double>() * (i * 1.0 / side_num);
    auto rotate = KIRI_PI<double>() / side_num;
    auto y = std::sin(angle + rotate) * side_radius;
    auto x = std::cos(angle + rotate) * side_radius;
    boundary->add(Vector2D(x, y));
  }

  auto r_min = 5;
  auto r_max = 50;
  auto target_radius = Random::get(r_min, r_max);
  auto rnd_point = boundary->rndInnerPointWithDistConstrain(r_min);

  // visualization
  std::vector<KiriLine2> lines;
  std::vector<KiriPoint2> points;
  std::vector<KiriCircle2> circles;
  std::vector<KiriLine2> precompute_lines;
  std::vector<Vector2F> precompute_points;

  for (auto j = 0; j < boundary->positions().size(); j++) {
    auto vert = boundary->positions()[j];
    auto vert1 =
        boundary->positions()[(j + 1) % (boundary->positions().size())];
    auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset,
                          Vector2F(vert1.x, vert1.y) + offset);
    line.thick = 1.f;
    precompute_lines.emplace_back(line);
  }

  auto total_iter = 0;
  auto iter_num = 0;
  auto cooling_func = [](const int iter, const double init, const double k) {
    return init * exp(-k * iter);
  };

  bool can_insert = true;
  std::vector<Vector3D> all_circles;
  std::vector<KiriCircle2> all_circles_draw;

  while (1) {
    // clear
    lines.clear();
    points.clear();
    circles.clear();
    precompute_points.clear();

    auto q_c = boundary->computeMinDistPointInPoly(rnd_point);
    // check inserted sphere dist
    for (auto i = 0; i < all_circles.size(); i++) {
      auto point = all_circles[i];
      auto dist = rnd_point.distanceTo(Vector2D(point.x, point.y)) - point.z;
      if (dist < q_c.distanceTo(rnd_point)) {
        q_c = Vector2D(point.x, point.y) +
              point.z * (rnd_point - Vector2D(point.x, point.y)).normalized();
      }
    }

    auto epsilon = cooling_func(iter_num, 0.08, 0.8);
    auto current_move = Vector2D(0.0);
    if (target_radius > rnd_point.distanceTo(q_c)) {
      current_move = epsilon * (rnd_point - q_c).normalized();
    } else {
      current_move = epsilon * (q_c - rnd_point).normalized();
    }

    if (current_move.length() <= q_c.distanceTo(rnd_point))
      rnd_point += current_move;

    auto current_move_len = current_move.length();
    if (current_move_len < 1e-6) {
      iter_num = 0;

      // check satifty the min radius
      if (rnd_point.distanceTo(q_c) >= r_min &&
          rnd_point.distanceTo(q_c) <= r_max) {
        all_circles.emplace_back(
            Vector3D(rnd_point.x, rnd_point.y, rnd_point.distanceTo(q_c)));
        all_circles_draw.emplace_back(
            KiriCircle2(Vector2F(rnd_point.x, rnd_point.y) + offset,
                        Vector3F(Random::get(0.0, 1.0), Random::get(0.0, 1.0),
                                 Random::get(0.0, 1.0)),
                        rnd_point.distanceTo(q_c), true));
        KIRI_LOG_DEBUG("Generate Particle with Radius={0}",
                       rnd_point.distanceTo(q_c));
      }

      auto pos_flag = false;
      while (!pos_flag) {
        pos_flag = true;
        rnd_point = boundary->rndInnerPointWithDistConstrain(r_min);
        target_radius = Random::get(r_min, r_max);
        for (auto i = 0; i < all_circles.size(); i++) {
          auto point = all_circles[i];
          auto dist =
              rnd_point.distanceTo(Vector2D(point.x, point.y)) - point.z;
          if (dist <= r_min) {
            pos_flag = false;
            break;
          }
        }
      }
    } else {
      circles.emplace_back(KiriCircle2(
          Vector2F(rnd_point.x, rnd_point.y) + offset, Vector3F(0.f, 1.f, 1.f),
          rnd_point.distanceTo(q_c), false));
    }

    iter_num++;

    precompute_points.emplace_back(Vector2F(q_c.x, q_c.y));
    precompute_points.emplace_back(Vector2F(rnd_point.x, rnd_point.y));

    for (auto i = 0; i < precompute_points.size(); i++)
      points.emplace_back(
          KiriPoint2(precompute_points[i] + offset, Vector3F(1.f, 0.f, 0.f)));

    for (auto i = 0; i < precompute_lines.size(); ++i)
      lines.emplace_back(precompute_lines[i]);

    for (auto i = 0; i < all_circles_draw.size(); i++)
      circles.emplace_back(all_circles_draw[i]);

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
