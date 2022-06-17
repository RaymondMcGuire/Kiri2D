/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-15 16:46:01
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-17 11:06:07
 * @FilePath: \Kiri2D\demos\interior_point_method\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d.h>
using namespace KIRI2D;

#include <ipm.h>

template <typename Derived>
std::string get_shape(const EigenBase<Derived> &x)
{
    std::ostringstream oss;
    oss << "(" << x.rows() << ", " << x.cols() << ")";
    return oss.str();
}

int main(int argc, char *argv[])
{
    // log system
    KiriLog::init();

    VectorXdual2nd data(2);
    data << -2.1433, 0.2949;

    auto ipm = std::make_shared<OPTIMIZE::IPM::InteriorPointMethod>(data);
    ipm->solve();

    // dual2nd dual_u;
    // VectorXdual dual_g;
    // Eigen::MatrixXd H = hessian(f, wrt(dual_x), at(dual_x), dual_u, dual_g);

    // std::cout << "dual_g = \n"
    //           << dual_g << std::endl; // print the evaluated gradient vector of u
    // std::cout << "H = \n"
    //           << H << std::endl; // print the evaluated Hessian matrix of u

    // auto norm = dual_g.norm();
    // // std::cout << norm << std::endl;

    // auto d = -(H.inverse() * dual_g);
    // std::cout << get_shape(d) << ";" << get_shape(dual_g.transpose()) << std::endl;

    // auto phi0 = f(dual_x);
    // std::cout << phi0.val << std::endl;

    // dual2nd dphi0 = (dual_g.transpose() * d).cast<dual2nd>()(0, 0);

    // auto eta = 1e-4;
    // auto alpha_smax = 1.0;
    // // search for unconstrained
    // auto x_plus = dual_x + alpha_smax * d.cast<dual2nd>();
    // std::cout << "left=" << f(x_plus) << std::endl;
    // auto right = dual_u + dphi0 * eta * alpha_smax;
    // std::cout << "right=" << eval(right) << std::endl;

    // auto new_x = dual_x + alpha_smax * d.cast<dual2nd>();
    // std::cout << "new_x=" << new_x << std::endl;

    return 0;
}
