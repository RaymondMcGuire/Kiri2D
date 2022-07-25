/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-21 11:00:14
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-25 14:18:53
 * @FilePath: \Kiri2D\demos\pdipm_sampler\include\data.h
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
  double max_radius;
  double min_dist;
  bool optimize;
  std::vector<int> neighbors;
  std::vector<int> need_optimize;
  std::vector<int> need_constrain;
};
} // namespace OPTIMIZE::IPM

#endif