/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-17 20:08:37
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-18 16:13:26
 * @FilePath: \Kiri2D\demos\rigidbody2d_scene\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#include <kiri2d.h>
#include <kiri2d/physics/rigidbody/rigidbody.h>

using namespace KIRI2D;
using namespace PHY::RIGIDBODY;

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

    auto circle = std::make_shared<Circle<float>>(1.f);
    auto rigidbody = std::make_shared<RigidBody<float>>(Vector2F(0.f));
    CompositeShapeRigidBody(circle, rigidbody);
    rigidbody->Print();
    return 0;
}