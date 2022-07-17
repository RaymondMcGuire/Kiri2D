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

int main(int argc, char *argv[])
{
  // log system
  KiriLog::init();

  for (auto i = 0; i < 1000; i++)
  {

    auto n = 3;
    // std::vector<double> data = {0.1253, 0.1307, 0.0428, 0.2549, 0.3627, 0.0836};
    std::vector<double> data;
    for (auto j = 0; j < n; j++)
    {
      data.emplace_back(Random::get(-1.0, 1.0));
    }
    // data.emplace_back(0.9503230351783563);
    // data.emplace_back(-0.7261491955624899);
    // data.emplace_back(0.41133531072687424);
    // data.emplace_back(0.2725921015579684);
    // data.emplace_back(-0.14693367664210566);
    // data.emplace_back(-0.5320528053654595);

    for (auto j = 0; j < n; j++)
    {
      KIRI_LOG_DEBUG("data.emplace_back({0})", data[j]);
    }

    int equ_num = 1;
    int inequ_num = 3;

    auto ipm = std::make_shared<OPTIMIZE::IPM::PrimalDualIPM>(data, equ_num, inequ_num);
    auto results = ipm->solution();
  }

  return 0;
}
