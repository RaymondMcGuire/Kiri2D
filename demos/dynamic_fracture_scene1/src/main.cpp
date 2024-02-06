/*
 * File: main.cpp
 * Module: src
 * Created Date: 2024-02-06
 * Author: Xu.WANG
 * -----
 * Last Modified:
 * Modified By:
 * -----
 * Copyright (c) 2024 Xu.WANG
 */

#include <kiri2d.h>

using namespace KIRI2D;
using namespace HDV;
using namespace PHY;

void swap(int *l, int *r) {
  int *tmp = l;
  *l = *r;
  *r = *tmp;
}

int main(int argc, char *argv[]) {
  // log system
  KiriLog::init();

  // scene renderer config
  auto window_height = 500.f;
  auto window_width = 500.f;
  auto object_scale = 10.f;
  auto offset =
      VectorX<2, double>((size_t)window_width, (size_t)window_height) / 2.0;
  auto scene = std::make_shared<KiriScene2D<double>>((size_t)window_width,
                                                     (size_t)window_height);
  auto renderer = std::make_shared<KiriRenderer2D<double>>(scene);

  // voronoi diagram config
  auto sampler_num = 10;
  auto voronoi2d = std::make_shared<Voronoi::PowerDiagram2D>();

  // clip boundary: square
  auto boundary = std::make_shared<Voronoi::VoronoiPolygon2>();
  boundary->add(VectorX<2, double>(-object_scale, -object_scale));
  boundary->add(VectorX<2, double>(-object_scale, object_scale));
  boundary->add(VectorX<2, double>(object_scale, object_scale));
  boundary->add(VectorX<2, double>(object_scale, -object_scale));
  voronoi2d->setBoundary(boundary);

  // generate random sites
  voronoi2d->generateRndSites(sampler_num);

  // compute
  voronoi2d->compute();

  auto system = std::make_shared<RIGIDBODY::RigidBodySystem<double>>(
      VectorX<2, double>(-25.f), VectorX<2, double>(40.f));

  // set world boundary
  auto boundary_btm = std::make_shared<RIGIDBODY::Polygon<double>>();
  boundary_btm->SetAsBox(24.f, 5.f);
  boundary_btm->SetOrientation(0.f);
  system->AddObject(boundary_btm, VectorX<2, double>(0.f, -24.f), true);

  // voronoi sites
  auto sites = voronoi2d->sites();
  for (auto i = 0; i < sites.size(); i++) {
    auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
    if (site->isBoundaryVertex())
      continue;

    auto shape = std::make_shared<RIGIDBODY::Polygon<double>>();
    auto cell_polygon = site->polygon();
    std::vector<VectorX<2, double>> verts;
    for (auto j = 0; j < cell_polygon->positions().size(); j++) {
      verts.emplace_back(VectorX<2, double>(cell_polygon->positions()[j].x,
                                            cell_polygon->positions()[j].y));
    }

    shape->Set(verts);
    system->AddObject(shape, VectorX<2, double>(site->x(), site->y()));
    auto body = shape->GetBody();
    body.lock()->SetRestitution(static_cast<double>(0.5));
    body.lock()->SetStaticFriction(static_cast<double>(0.2));
    body.lock()->SetDynamicFriction(static_cast<double>(0.1));
    body.lock()->SetPosition(shape->GetCentroid());
    body.lock()->SetOrientation(0.0);
  }

  bool enable_sim = false;
  while (1) {
    if (enable_sim)
      system->UpdateSystem();

    system->Render(scene, renderer, object_scale);

    renderer->SaveImages2File();
    cv::imshow("KIRI2D", renderer->GetCanvas());
    auto key = cv::waitKey(5);
    if (key == 'q')
      enable_sim = !enable_sim;

    // clean canvas
    renderer->ClearCanvas();
    scene->Clear();
  }

  return 0;
}
