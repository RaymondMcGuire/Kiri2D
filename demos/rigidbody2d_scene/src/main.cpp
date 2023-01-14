/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2023-01-11 14:46:17
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-14 15:08:33
 * @FilePath: \Kiri2D\demos\rigidbody2d_scene\src\main.cpp
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#include <kiri2d.h>

using namespace KIRI2D;
using namespace PHY;

int main(int argc, char *argv[])
{
  // log system
  KiriLog::init();

  // scene renderer config
  auto window_height = 500.f;
  auto window_width = 500.f;
  auto object_scale = 10.f;

  auto scene = std::make_shared<KiriScene2D>((size_t)window_width,
                                             (size_t)window_height);
  auto renderer = std::make_shared<KiriRenderer2D>(scene);
  auto system = std::make_shared<RIGIDBODY::RigidBodySystem<float>>(Vector2F(-25.f), Vector2F(40.f));

  // set world boundary
  auto boundary_btm = std::make_shared<RIGIDBODY::Polygon<float>>();
  boundary_btm->SetAsBox(24.f, 5.f);
  boundary_btm->SetOrientation(0.f);
  system->AddObject(boundary_btm, Vector2F(0.f, -24.f), true);

  auto LoadPolygonFromXYFile = [](std::vector<Vector2F> &points, const char *filePath, const float scale)
  {
    size_t num;
    std::ifstream file(filePath);
    file >> num;
    for (int i = 0; i < num; ++i)
    {
      Vector2F xy;
      file >> xy.x >> xy.y;
      points.emplace_back(xy * scale);
    }

    file.close();
  };

  while (1)
  {
    system->EmitRndPolygon(3.f, Vector2F(-20.f, 30.f), Vector2F(20.f, 35.f), 5);
    system->UpdateSystem();
    system->Render(scene, renderer, object_scale);

    renderer->SaveImages2File();
    cv::imshow("KIRI2D", renderer->GetCanvas());
    cv::waitKey(5);

    // clean canvas
    renderer->ClearCanvas();
    scene->Clear();
  }

  return 0;
}
