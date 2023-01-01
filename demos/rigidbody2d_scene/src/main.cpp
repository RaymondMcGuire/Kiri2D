/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-23 13:08:40
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-23 14:07:23
 * @FilePath: \Kiri2D\demos\rigidbody2d_scene\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#include <kiri2d.h>
#include <kiri2d/physics/rigidbody/rigidbody_system.h>
using namespace KIRI2D;
using namespace PHY;

int main(int argc, char *argv[])
{
  // log system
  KiriLog::init();

  // scene renderer config
  auto window_height = 500.f;
  auto window_width = 500.f;
  auto offset = Vector2F((size_t)window_width, (size_t)window_height) / 2.f;
  auto scene = std::make_shared<KiriScene2D>((size_t)window_width,
                                             (size_t)window_height);
  auto renderer = std::make_shared<KiriRenderer2D>(scene);
  auto system = std::make_shared<RIGIDBODY::RigidBodySystem<float>>();
  system->AddObject(std::make_shared<RIGIDBODY::Circle<float>>(5.f), Vector2F(0.f), true);

  auto box = std::make_shared<RIGIDBODY::Polygon<float>>();
  box->SetAsBox(30.f, 1.f);
  box->SetOrientation(0.f);
  system->AddObject(box, Vector2F(0.f, -15.f), true);

  system->AddObject(std::make_shared<RIGIDBODY::Circle<float>>(3.f), Vector2F(-5.f, 20.f), false);
  system->AddObject(std::make_shared<RIGIDBODY::Circle<float>>(3.f), Vector2F(2.f, 20.f), false);
  system->AddObject(std::make_shared<RIGIDBODY::Circle<float>>(2.f), Vector2F(1.f, 30.f), false);

  while (1)
  {
    system->UpdateSystem();
    system->Render(scene, renderer);

    cv::imshow("KIRI2D", renderer->canvas());
    cv::waitKey(5);

    // clean canvas
    renderer->clearCanvas();
    scene->clear();
  }

  return 0;
}
