/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-14 12:35:07
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-14 13:22:23
 * @FilePath: \Kiri2D\demos\primal_dual_ipm\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#include <kiri2d.h>
using namespace KIRI2D;

#include <primal_dual_ipm.h>

int main(int argc, char *argv[]) {
  // log system
  KiriLog::init();

  auto n = 2;
  std::vector<double> data;

  for (auto j = 0; j < n; j++) {
    data.emplace_back(Random::get(-1.0, 1.0));
  }

  int inequ_num = 3;

  auto ipm = std::make_shared<OPTIMIZE::IPM::PrimalDualIPM>(data, inequ_num);
  auto results = ipm->solution();

  return 0;
}
