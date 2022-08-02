/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-08-02 12:53:40
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-08-02 12:55:07
 * @FilePath: \Kiri2D\demos\eigen_mkl_tester\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d.h>

#define EIGEN_USE_MKL_ALL

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using Eigen::EigenBase;
using Eigen::MatrixXd;

using namespace KIRI2D;

int main(int argc, char *argv[]) {
  // log system
  KiriLog::init();

  MatrixXd a = MatrixXd::Random(12000, 12000);
  MatrixXd b = MatrixXd::Random(12000, 12000);

  double start = clock();
  MatrixXd c = a * b;
  double endd = clock();
  double thisTime = (double)(endd - start) / CLOCKS_PER_SEC;

  std::cout << thisTime << std::endl;

  return 0;
}
