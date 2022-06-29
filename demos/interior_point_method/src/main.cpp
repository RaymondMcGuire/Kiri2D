/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-25 01:39:47
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-29 09:17:25
 * @FilePath: \Kiri2D\demos\interior_point_method\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d.h>
using namespace KIRI2D;

#include <ipm.h>

int main(int argc, char *argv[])
{
    // log system
    KiriLog::init();

    std::vector<double> data;

    std::vector<Vector2F> data_pos;
    std::vector<double> data_radius;

    data_pos.emplace_back(Vector2F(0.5, 0.5));
    data_pos.emplace_back(Vector2F(0.25, 0.75));
    data_pos.emplace_back(Vector2F(0.6, 0.75));
    data_pos.emplace_back(Vector2F(0.25, 0.25));
    data_pos.emplace_back(Vector2F(0.7, 0.3));

    data_radius.emplace_back(0.3);
    data_radius.emplace_back(0.25);
    data_radius.emplace_back(0.25);
    data_radius.emplace_back(0.25);
    data_radius.emplace_back(0.3);

    for (auto i = 0; i < 1; i++)
    {
        data.clear();
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));

        KIRI_LOG_INFO("Attempt num={0}", i + 1);

        for (auto j = 0; j < data.size(); j++)
        {
            KIRI_LOG_INFO("Init Data[{0}]={1}", j, data[j]);
        }

        int n = data.size();
        int inequ_num = 2 * n + n * (n - 1) / 2;

        auto ipm = std::make_shared<OPTIMIZE::IPM::InteriorPointMethod>(data, inequ_num, data_radius, data_pos);
    }

    // using Eigen::MatrixXd;
    // using Eigen::VectorXd;
    // ArrayXreal x(2); // the input array x with 5 variables
    // x << -0.6165, -0.1307;

    // real u; // the output scalar u = f(x) evaluated together with gradient below

    // VectorXd g = gradient(f, wrt(x), at(x), u); // evaluate the function value u and its gradient vector g = du/dx

    // std::cout << "u = " << u << std::endl; // print the evaluated output u
    // std::cout << "g = \n"
    //           << g << std::endl; // print the evaluated gradient vector g = du/dx

    // VectorXreal F;
    // MatrixXd J = jacobian(f1, wrt(x), at(x), F);
    // std::cout << "J = \n"
    //           << J << std::endl;
    // // ci
    // auto f_general0 = double(f(x));
    // VectorXd f_general1 = f1(x).cast<double>();
    // MatrixXd Ci(f_general1.size() + 1, 1);
    // Ci << f_general0, f_general1;
    // std::cout << "Ci = \n"
    //           << Ci << std::endl;

    // // dci
    // MatrixXd C(g.rows(), g.cols() + J.cols());
    // C << g, J;

    // std::cout << "C = \n"
    //           << C << std::endl;

    // MatrixXd D = -MatrixXd::Identity(x.size() + 1, x.size() + 1);
    // std::cout << "D = \n"
    //           << D << std::endl;

    // MatrixXd E(C.rows() + D.rows(), C.cols());
    // E << C, D;
    // std::cout << "E = \n"
    //           << E << std::endl;

    // dual2nd mPhi0;
    // VectorXdual mGrad;
    // VectorXdual2nd data(2);
    // data << -0.6165, -0.1307;

    // Eigen::MatrixXd mHessian = hessian(original_func, wrt(data), at(data), mPhi0, mGrad);

    // std::cout << "mGrad = \n"
    //           << mGrad.cast<double>() << std::endl;

    // auto gradXD = mGrad.cast<double>();
    // MatrixXd lambda = C.completeOrthogonalDecomposition().pseudoInverse() * mGrad.cast<double>();

    // std::cout << "lambda = \n"
    //           << lambda << std::endl;

    // for (auto i = 0; i < lambda.rows(); i++)
    // {
    //     for (auto j = 0; j < lambda.cols(); j++)
    //     {
    //         if (lambda(i, j) < 0.0)
    //             lambda(i, j) = 1e-4;
    //     }
    // }

    // std::cout << "lambda = \n"
    //           << lambda << std::endl;

    // MatrixXd grad_tmp(data.size() + 2 * 3, 1);
    // // C << g, J;
    // auto grad_1 = mGrad.cast<double>() - C * lambda; //  // - C.dot(lambda);

    // MatrixXd slack(3, 1);
    // slack << 1e-4, 1e-4, 1e-4;

    // auto tmp_slack = slack;
    // for (auto i = 0; i < tmp_slack.rows(); i++)
    // {
    //     for (auto j = 0; j < tmp_slack.cols(); j++)
    //     {
    //         tmp_slack(i, j) = 0.2 / (tmp_slack(i, j) + MEpsilon<float>());
    //     }
    // }

    // std::cout << "tmp_slack = \n"
    //           << tmp_slack << std::endl;

    // auto grad_2 = lambda - tmp_slack;

    // std::cout << "grad_2 = \n"
    //           << grad_2 << std::endl;

    // auto grad_3 = Ci - slack;

    // std::cout << "grad_3 = \n"
    //           << grad_3 << std::endl;

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
