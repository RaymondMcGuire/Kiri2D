/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-23 13:08:40
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-23 14:07:23
 * @FilePath: \Kiri2D\demos\rigidbody2d_scene\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#include <eventpp/eventdispatcher.h>
#include <kiri2d.h>
#include <kiri2d/physics/rigidbody/rigidbody.h>
#include <kiri2d/physics/rigidbody/rigidbody_solver.h>
using namespace KIRI2D;
using namespace PHY::RIGIDBODY;

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

  auto circle = std::make_shared<Circle<float>>(1.f);
  auto rigidbody = std::make_shared<RigidBody<float>>(Vector2F(0.f));
  CompositeShapeRigidBody(circle, rigidbody);
  rigidbody->Print();

  auto circle1 = std::make_shared<Circle<float>>(2.f);
  auto rigidbody1 = std::make_shared<RigidBody<float>>(Vector2F(1.f));
  CompositeShapeRigidBody(circle1, rigidbody1);
  rigidbody1->Print();



  return 0;
}
