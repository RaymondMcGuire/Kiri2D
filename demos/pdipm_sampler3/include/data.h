/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-08-02 10:18:22
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-08-03 10:43:12
 * @FilePath: \Kiri2D\demos\pdipm_sampler2\include\data.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _DATA_H_
#define _DATA_H_

#pragma once

#include <kiri2d.h>
using namespace KIRI2D;

#include <tuple>

namespace OPTIMIZE::IPM {
struct particle {
  Vector3D pos;
  double radius;
  double new_radius;
  double max_radius;
  std::vector<int> neighbors;
};

struct compositeGrid {
  int id;
  std::vector<int> subid;
};
} // namespace OPTIMIZE::IPM

#endif