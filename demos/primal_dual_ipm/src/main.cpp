/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-18 12:51:07
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-25 13:51:51
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

  for (auto i = 0; i < 1000; i++) {

    auto n = 3;
    std::vector<double> data;
    for (auto j = 0; j < n; j++) {
      data.emplace_back(Random::get(0.0, 1.0));
    }

    for (auto j = 0; j < n; j++) {
      KIRI_LOG_DEBUG("data.emplace_back({0})", data[j]);
    }

    int equ_num = 2;
    int inequ_num = 0;

    auto ipm = std::make_shared<OPTIMIZE::IPM::PrimalDualIPM>(data, equ_num,
                                                              inequ_num);
    auto results = ipm->solution();
  }

  return 0;
}
