/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-25 01:39:47
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-29 09:16:04
 * @FilePath: \Kiri2D\demos\interior_point_method\include\ipm.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _IPM_H_
#define _IPM_H_

#pragma once

#include <sstream>

#include <Eigen/LU>
#include <Eigen/QR>

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

using namespace autodiff;
using std::ostringstream;

using Eigen::EigenBase;
using Eigen::MatrixXd;
using Eigen::VectorXd;

//-------------------------------------------------------------- example1
// VectorXreal Constrainsts(const VectorXreal &x)
// {
//     VectorXreal consts(3);
//     consts[0] = x[0] + 2 * x[1] - 10;
//     consts[1] = x[0];
//     consts[2] = x[1];
//     return consts;
// }

// dual2nd ConstrainstFunc(const VectorXdual2nd &x, const MatrixXd &lambda)
// {
//     return (x[0] + 2 * x[1] - 10) * lambda(0, 0) + x[0] * lambda(1, 0) + x[1] * lambda(2, 0);
// }

// dual2nd TargetFunc(const VectorXdual2nd &x)
// {
//     return x[0] * x[0] + 2 * x[1] * x[1] + 2 * x[0] + 8 * x[1];
// }
//-------------------------------------------------------------- example1

//-------------------------------------------------------------- example2
// real Constrainst0(const VectorXreal &x)
// {
//     return 3 - x[0] - 4 * x[1];
// }

// real Constrainst1(const VectorXreal &x)
// {
//     return x[0] - x[1];
// }

// dual2nd ConstrainstFunc(const VectorXdual2nd &x, const MatrixXd &lambda)
// {
//     return (3 - x[0] - 4 * x[1]) * lambda(0, 0) + (x[0] - x[1]) * lambda(1, 0);
// }

// dual2nd TargetFunc(const VectorXdual2nd &x)
// {
//     return (x[0] - 2) * (x[0] - 2) + 2 * (x[1] - 1) * (x[1] - 1);
// }
//-------------------------------------------------------------- example2

VectorXreal Constrainsts(const VectorXreal &lambda, const std::vector<double> &radius, const std::vector<Vector2F> &pos)
{
    auto counter = 0;
    auto n = lambda.size();
    VectorXreal consts(2 * n + n * (n - 1) / 2);

    for (auto i = 0; i < n; i++)
        consts[counter++] = lambda[i];

    for (auto i = 0; i < n; i++)
        consts[counter++] = 1 - lambda[i];

    for (auto i = 0; i < n - 1; i++)
        for (auto j = i + 1; j < n; j++)
            consts[counter++] = (pos[i] - pos[j]).length() - (lambda[i] * radius[i] + lambda[j] * radius[j]);

    return consts;
}

dual2nd ConstrainstFunc(const VectorXdual2nd &x, const MatrixXd &lambda, const std::vector<double> &radius, const std::vector<Vector2F> &pos)
{
    auto counter = 0;
    auto n = x.size();
    dual2nd sum = 0.0;

    for (auto i = 0; i < n; i++)
    {
        sum += x[i] * lambda(counter++, 0);
    }

    for (auto i = 0; i < n; i++)
    {
        sum += (1 - x[i]) * lambda(counter++, 0);
    }

    for (auto i = 0; i < n - 1; i++)
        for (auto j = i + 1; j < n; j++)
            sum += ((pos[i] - pos[j]).length() - (x[i] * radius[i] + x[j] * radius[j])) * lambda(counter++, 0);

    return sum;
}

dual2nd TargetFunc(const VectorXdual2nd &x, std::vector<double> radius)
{
    auto n = x.size();
    dual2nd sum = 0.0;
    for (auto j = 0; j < n; j++)
    {
        sum -= KIRI_PI<dual2nd>() * (x[j] * radius[j]) * (x[j] * radius[j]);
    }

    return sum;
}

template <typename Derived>
std::string get_shape(const EigenBase<Derived> &x)
{
    std::ostringstream oss;
    oss << "(" << x.rows() << ", " << x.cols() << ")";
    return oss.str();
}

namespace OPTIMIZE::IPM
{
    class InteriorPointMethod
    {
    public:
        explicit InteriorPointMethod(
            const std::vector<double> &data,
            int inequ,
            const std::vector<double> &radius,
            const std::vector<Vector2F> &pos)
            : mData(data),
              mRadius(radius),
              mPos(pos),
              mVariableNum(data.size()),
              mInEquNum(inequ)
        {
            this->initDataVector();
            this->initSlack();
            this->initLambdaMultipler();

            this->solve();
        }

        virtual ~InteriorPointMethod()
        {
        }

        void solve()
        {
            mFTolConverged = false;
            mFLast = static_cast<double>(TargetFunc(mDual2ndData, mRadius));

            this->computeKKT();

            for (auto i = 0; i < mOuterIterNum; i++)
            {
                KIRI_LOG_DEBUG("outer iter num={0}", i);

                if (mKKT1.norm() <= mKTol && mKKT2.norm() <= mKTol && mKKT3.norm() <= mKTol && mKKT4.norm() <= mKTol)
                {
                    mSignal = 1;
                    mKtolConverged = true;
                    break;
                }

                for (auto j = 0; j < mInnerIterNum; j++)
                {
                    mMuTol = std::max(mKTol, mMu);
                    if (mKKT1.norm() <= mMuTol && mKKT2.norm() <= mMuTol && mKKT3.norm() <= mMuTol && mKKT4.norm() <= mMuTol)
                    {
                        mSignal = 1;
                        mKtolConverged = true;
                        break;
                    }

                    KIRI_LOG_DEBUG("inner iter num={0}", j);

                    // compute gradient and hessian
                    this->computeGrad();
                    this->computeConstrainstsHessian();

                    // search direction
                    MatrixXd dz = -(mHessian.inverse() * mGrad);
                    // KIRI_LOG_DEBUG("search direction=\n{0}", dz);

                    // change sign definition for lambda multipliers search direction
                    for (auto i = mVariableNum + mInEquNum; i < dz.size(); i++)
                    {
                        dz(i, 0) = -dz(i, 0);
                    }
                    // KIRI_LOG_DEBUG("changed sign search direction=\n{0}", dz);

                    this->computeBarriarCostGrad();
                    auto dot_barrier_dir = mBarrierCostGrad.transpose() * dz(Eigen::seq(0, mVariableNum + mInEquNum - 1), Eigen::placeholders::all);
                    // KIRI_LOG_DEBUG("dot_barrier_dir=\n{0}", dot_barrier_dir);
                    // KIRI_LOG_DEBUG("dz=\n{0}", dir(Eigen::seq(0, 4), Eigen::placeholders::all));

                    auto ci = this->computeConstrainsts(mRealData);
                    MatrixXd con = ci - mSlack;
                    auto sum_abs_con = con.array().abs().sum();
                    // KIRI_LOG_DEBUG("sum_abs_con={0}", sum_abs_con);

                    auto nu_thresh = dot_barrier_dir / (1 - mRho) / sum_abs_con;
                    // KIRI_LOG_DEBUG("nu_thresh={0}", nu_thresh);
                    //  if (mNu < nu_thresh)
                    //{
                    //     // mNu = nu_thresh;
                    //      KIRI_LOG_DEBUG("update nu = {0}", mNu);
                    //  }

                    computeBackTrackingLineSearch(dz);

                    mIterCount++;

                    this->computeKKT();

                    if (mSignal == -2)
                        break;
                }

                if (mSignal != -2)
                {
                    mFNew = static_cast<double>(TargetFunc(mDual2ndData, mRadius));
                    // KIRI_LOG_DEBUG("new={0}; past={1}", mFNew, mFLast);
                    if (abs(mFLast - mFNew) < abs(mFTol))
                    {
                        mSignal = 2;
                        mFTolConverged = true;
                    }
                    else
                        mFLast = mFNew;
                }

                if (mSignal == -2 || mFTolConverged)
                    break;

                // update barrier params

                auto xi = mInEquNum * (mSlack.array() * mLambda.array()).minCoeff() / ((mSlack.transpose() * mLambda)(0, 0) + MEpsilon<float>());
                // KIRI_LOG_DEBUG("xi={0}", xi);

                mMu = 0.1 * pow(std::min(0.05 * (1.0 - xi) / (xi + MEpsilon<float>()), 2.0), 3) * ((mSlack.transpose() * mLambda)(0, 0) / mInEquNum);
                if (mMu < 0)
                    mMu = 0;
                // KIRI_LOG_DEBUG("mMu={0}", mMu);
            }

            KIRI_LOG_INFO("optimal solution={0}", mRealData.transpose());

            // print();
        }

        void print()
        {
            // std::cout << "mFuncGrad = \n"
            //           << mFuncGrad << std::endl;
            // std::cout << "mFuncHessian = \n"
            //           << mFuncHessian << std::endl;
        }

    private:
        int mIterCount = 0;
        int mSignal = 0;
        int mInEquNum = 0;
        int mVariableNum = 0;
        int mOuterIterNum = 10;
        int mInnerIterNum = 20;

        double mEta = 1e-4;
        double mKTol = 1e-4;
        double mMuTol = 1e-4;
        double mFTol = 1e-8;
        double mTau = 0.995;
        double mMu = 0.2;
        double mNu = 10.0;
        double mRho = 0.1;
        bool mKtolConverged = false;
        bool mFTolConverged = false;

        double mFLast, mFNew = 0.0;

        dual2nd mPhi0, mLastPhi0;
        VectorXdual mFuncGrad;
        MatrixXd mFuncHessian;

        std::vector<double> mData;
        std::vector<double> mRadius;
        std::vector<Vector2F> mPos;

        VectorXreal mRealData;
        VectorXdual2nd mDual2ndData;

        MatrixXd mConstrainJacobian;
        MatrixXd mConstrainJacobianWithIdentity;
        MatrixXd mConstraintsFuncHessian;

        MatrixXd mGrad;
        MatrixXd mHessian;

        MatrixXd mLambda, mSlack;
        MatrixXd mKKT1, mKKT2, mKKT3, mKKT4;

        MatrixXd mBarrierCostGrad;

        void initDataVector()
        {
            VectorXreal real_data(mVariableNum);
            VectorXdual2nd dual2nd_data(mVariableNum);

            for (auto i = 0; i < mVariableNum; i++)
            {
                real_data[i] = mData[i];
                dual2nd_data[i] = mData[i];
            }

            mRealData = real_data;
            mDual2ndData = dual2nd_data;
        }

        void initSlack()
        {
            MatrixXd slack(mInEquNum, 1);
            MatrixXd ci = this->computeConstrainsts(mRealData);

            for (auto i = 0; i < slack.rows(); i++)
                for (auto j = 0; j < slack.cols(); j++)
                    slack(i, j) = std::max(mKTol, ci(i, j));

            mSlack = slack;

            // KIRI_LOG_DEBUG("init mSlack=\n {0}", mSlack);
        }

        void initLambdaMultipler()
        {
            this->computeHessian();
            this->computeConstrainstsJacobian();
            MatrixXd lambda = mConstrainJacobian.completeOrthogonalDecomposition().pseudoInverse() * mFuncGrad.cast<double>();

            mLambda = lambda;
            for (auto i = 0; i < mLambda.rows(); i++)
                for (auto j = 0; j < mLambda.cols(); j++)
                    if (mLambda(i, j) < 0.0)
                        mLambda(i, j) = 1e-4;

            // KIRI_LOG_DEBUG("init Lambda=\n {0}", mLambda);
        }

        void computeHessian()
        {
            mFuncHessian = hessian(TargetFunc, wrt(mDual2ndData), at(mDual2ndData, mRadius), mPhi0, mFuncGrad);

            // KIRI_LOG_DEBUG("mFuncHessian={0}", mFuncHessian);
        }

        //! TODO
        MatrixXd computeConstrainsts(VectorXreal realData)
        {
            // auto constrain0 = double(Constrainst0(realData));
            // VectorXd constrain1 = Constrainst1(realData).cast<double>();

            VectorXd constrain = Constrainsts(realData, mRadius, mPos).cast<double>();

            MatrixXd constrain_i(mInEquNum, 1);
            constrain_i << constrain;

            return constrain_i;
        }

        //! TODO
        void computeConstrainstsJacobian()
        {
            VectorXreal constrain_val;
            MatrixXd jaco = jacobian(Constrainsts, wrt(mRealData), at(mRealData, mRadius, mPos), constrain_val);
            mConstrainJacobian = jaco.transpose();

            MatrixXd eyeConstrainMatrix = -MatrixXd::Identity(mInEquNum, mInEquNum);
            MatrixXd constrainst_jacobian_with_identity(mConstrainJacobian.rows() + eyeConstrainMatrix.rows(), mConstrainJacobian.cols());
            constrainst_jacobian_with_identity << mConstrainJacobian, eyeConstrainMatrix;
            mConstrainJacobianWithIdentity = constrainst_jacobian_with_identity;
        }

        //! TODO
        // MatrixXd computeConstrainsts(VectorXreal realData)
        // {
        //     // auto constrain0 = double(Constrainst0(realData));
        //     // auto constrain1 = double(Constrainst1(realData));

        //     VectorXd constrain0 = Constrainst0(realData).cast<double>();
        //     VectorXd constrain1 = Constrainst1(realData).cast<double>();

        //     MatrixXd constrain_i(mInEquNum, 1);
        //     constrain_i << constrain0, constrain1;

        //     return constrain_i;
        // }

        // //! TODO
        // void computeConstrainstsJacobian()
        // {

        //     VectorXreal F, F1;
        //     MatrixXd grad0 = jacobian(Constrainst0, wrt(mRealData), at(mRealData), F);
        //     MatrixXd grad1 = jacobian(Constrainst1, wrt(mRealData), at(mRealData), F1);

        //     MatrixXd constrainst_jacobian(grad0.rows(), grad0.cols() + grad1.cols());
        //     constrainst_jacobian << grad0, grad1;
        //     mConstrainJacobian = constrainst_jacobian;

        //     MatrixXd eyeConstrainMatrix = -MatrixXd::Identity(mInEquNum, mInEquNum);
        //     MatrixXd constrainst_jacobian_with_identity(constrainst_jacobian.rows(), constrainst_jacobian.cols() + eyeConstrainMatrix.cols());
        //     constrainst_jacobian_with_identity << constrainst_jacobian, eyeConstrainMatrix;
        //     mConstrainJacobianWithIdentity = constrainst_jacobian_with_identity;

        //     // KIRI_LOG_DEBUG("mConstrainJacobian=\n{0}; \n mConstrainJacobianWithIdentity=\n{1}", mConstrainJacobian, mConstrainJacobianWithIdentity);
        // }

        void computeBarriarCostGrad()
        {
            MatrixXd barriar_cost_grad = MatrixXd::Zero(mVariableNum + mInEquNum, 1);
            MatrixXd func_grad = mFuncGrad.cast<double>();
            barriar_cost_grad << func_grad, -mMu / (mSlack.array() + MEpsilon<float>());
            mBarrierCostGrad = barriar_cost_grad;
            // KIRI_LOG_DEBUG("mBarrierCostGrad=\n{0}", mBarrierCostGrad);
        }

        double computeMaximumStepSize(Eigen::ArrayXd x, Eigen::ArrayXd dx)
        {
            auto gold = (sqrt(5.0) + 1.0) / 2.0;
            auto a = 0.0;
            auto b = 1.0;

            auto xbdx = x + b * dx - (1 - mTau) * x;
            bool flag = true;

            for (auto i = 0; i < xbdx.size(); i++)
            {
                if (xbdx[i] < 0)
                {
                    flag = false;
                    break;
                }
            }

            if (flag)
                return b;

            auto c = b - (b - a) / gold;
            auto d = a + (b - a) / gold;
            while (abs(b - a) > gold * MEpsilon<float>())
            {
                auto xddx = x + d * dx - (1 - mTau) * x;
                flag = false;
                for (auto i = 0; i < xddx.size(); i++)
                {
                    if (xddx[i] < 0)
                    {
                        flag = true;
                        break;
                    }
                }

                if (flag)
                    b = d;

                else
                    a = d;

                if (c > a)
                {
                    auto xcdx = x + c * dx - (1 - mTau) * x;
                    flag = false;
                    for (auto i = 0; i < xcdx.size(); i++)
                    {
                        if (xcdx[i] < 0)
                        {
                            flag = true;
                            break;
                        }
                    }

                    if (flag)
                        b = c;
                    else
                        a = c;
                }

                c = b - (b - a) / gold;
                d = a + (b - a) / gold;
            }

            return a;
        }

        void computeGrad()
        {
            auto grad2 = mSlack;
            for (auto i = 0; i < grad2.rows(); i++)
            {
                for (auto j = 0; j < grad2.cols(); j++)
                {
                    grad2(i, j) = (mLambda(i, j) - mMu / (mSlack(i, j) + MEpsilon<float>()));
                }
            }

            mGrad = MatrixXd(mKKT1.rows() + mKKT2.rows() + mKKT4.rows(), 1);
            mGrad << mKKT1, grad2, mKKT4;
            // KIRI_LOG_DEBUG("mGrad=\n{0}", mGrad);
        }

        void computeConstrainstsHessian()
        {
            this->computeConstrainstsJacobian();

            dual2nd u;
            VectorXdual g;
            mConstraintsFuncHessian = hessian(ConstrainstFunc, wrt(mDual2ndData), at(mDual2ndData, mLambda,mRadius,mPos), u, g);

            // KIRI_LOG_DEBUG("mConstraintsFuncHessian={0}", mConstraintsFuncHessian);

            // Hessian of the Lagrangian
            MatrixXd d2L = mFuncHessian - mConstraintsFuncHessian;

            // KIRI_LOG_DEBUG("d2L={0}", d2L);

            MatrixXd d2LUpper = d2L.triangularView<Eigen::Upper>();
            // KIRI_LOG_DEBUG("d2LUpper={0}", d2LUpper);

            auto sigma = mSlack;
            for (auto i = 0; i < sigma.rows(); i++)
            {
                for (auto j = 0; j < sigma.cols(); j++)
                {
                    sigma(i, j) = mLambda(i, j) / (mSlack(i, j) + MEpsilon<float>());
                }
            }

            auto diagSize = sigma.array().size();
            MatrixXd SigmaDiag(diagSize, diagSize);

            SigmaDiag = sigma.array().matrix().asDiagonal();
            // KIRI_LOG_DEBUG("sigma={0}", SigmaDiag);

            MatrixXd eyeConstrainMatrix = -MatrixXd::Identity(mInEquNum, mInEquNum);

            MatrixXd HessianColConcate = MatrixXd::Zero(mVariableNum, mVariableNum + 2 * mInEquNum);
            MatrixXd VarConstrainZeroMatrix = MatrixXd::Zero(mVariableNum, mInEquNum);
            HessianColConcate << d2LUpper, VarConstrainZeroMatrix, mConstrainJacobian;

            MatrixXd HessianColConcate1 = MatrixXd::Zero(mInEquNum, mVariableNum + 2 * mInEquNum);
            MatrixXd ConstrainVarZeroMatrix = MatrixXd::Zero(mInEquNum, mVariableNum);
            HessianColConcate1 << ConstrainVarZeroMatrix, SigmaDiag, eyeConstrainMatrix;

            MatrixXd HessianRowConcate = MatrixXd::Zero(mVariableNum + mInEquNum, mVariableNum + 2 * mInEquNum);
            HessianRowConcate << HessianColConcate, HessianColConcate1;

            MatrixXd ConstrainVarPlusTwoConstrainMatrix = MatrixXd::Zero(mInEquNum, mVariableNum + 2 * mInEquNum);

            MatrixXd hessianMatrix(mVariableNum + 2 * mInEquNum, mVariableNum + 2 * mInEquNum);
            hessianMatrix << HessianRowConcate, ConstrainVarPlusTwoConstrainMatrix;

            MatrixXd hessianupperMatrix = hessianMatrix.triangularView<Eigen::Upper>();
            hessianMatrix = hessianupperMatrix + hessianupperMatrix.transpose();

            MatrixXd hessianHalfDiagMatrix = (hessianMatrix.diagonal() / 2).array().matrix().asDiagonal();
            hessianMatrix -= hessianHalfDiagMatrix;

            mHessian = hessianMatrix;
            // KIRI_LOG_DEBUG("hessianMatrix=\n{0}", hessianMatrix);
        }

        void computeKKT()
        {
            this->computeHessian();
            this->computeConstrainstsJacobian();

            auto grad2 = mSlack;
            auto kkt2 = mSlack;
            for (auto i = 0; i < kkt2.rows(); i++)
            {
                for (auto j = 0; j < kkt2.cols(); j++)
                {
                    kkt2(i, j) = (mLambda(i, j) - mMu / (mSlack(i, j) + MEpsilon<float>())) * mSlack(i, j);
                    grad2(i, j) = (mLambda(i, j) - mMu / (mSlack(i, j) + MEpsilon<float>()));
                }
            }

            // KIRI_LOG_DEBUG("mLambda=\n{0}", mLambda);

            mKKT1 = mFuncGrad.cast<double>() - mConstrainJacobian * mLambda;

            mKKT2 = kkt2;

            mKKT3 = MatrixXd::Zero(1, 1);

            auto ci = this->computeConstrainsts(mRealData);
            mKKT4 = ci - mSlack;

            // KIRI_LOG_DEBUG("KKT1=\n{0}", mKKT1);
            // KIRI_LOG_DEBUG("KKT2=\n{0}", mKKT2);
            // KIRI_LOG_DEBUG("KKT3=\n{0}", mKKT3);
            // KIRI_LOG_DEBUG("KKT4=\n{0}", mKKT4);
        }

        void computeBackTrackingLineSearch(MatrixXd dz, double alphaSMax = 1.0, double alphaLMax = 1.0)
        {
            auto correction = false;
            auto alpha_corr = 1.0;
            MatrixXd dzp = MatrixXd::Zero(mVariableNum + mInEquNum, 1);

            // mPhi0 = targetFunc(mData);

            auto alpha_smax = computeMaximumStepSize(mSlack.array(), dz(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all).array());

            // KIRI_LOG_DEBUG("alpha_smax={0}", alpha_smax);

            // KIRI_LOG_DEBUG("mLambda={0}", mLambda);
            auto alpha_lmax = computeMaximumStepSize(mLambda.array(), dz(Eigen::seq(mVariableNum + mInEquNum, dz.size() - 1), Eigen::placeholders::all).array());

            // KIRI_LOG_DEBUG("alpha_lmax={0}", alpha_lmax);

            MatrixXd dx = dz(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all);
            MatrixXd ds = dz(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all);
            MatrixXd dl = dz(Eigen::seq(mVariableNum + mInEquNum, dz.size() - 1), Eigen::placeholders::all);
            // KIRI_LOG_DEBUG("ds={0}", ds);
            // KIRI_LOG_DEBUG("dl={0}", dl);

            auto ci = this->computeConstrainsts(mRealData);

            // compute merit function
            auto phi_nu = eval(TargetFunc(mDual2ndData, mRadius));
            phi_nu -= mMu * log(mSlack.array()).sum();
            phi_nu += mNu * abs(ci.array() - mSlack.array()).sum();
            // KIRI_LOG_DEBUG("phi_nu={0}", phi_nu);

            MatrixXd func_grad = mFuncGrad.cast<double>();
            auto dphi_nu = (func_grad.transpose() * dz(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all))(0, 0);
            dphi_nu -= mNu * abs(ci.array() - mSlack.array()).sum();

            MatrixXd tmp = mSlack;
            for (auto i = 0; i < tmp.rows(); i++)
            {
                for (auto j = 0; j < tmp.cols(); j++)
                {
                    tmp(i, j) = mMu / (mSlack(i, j) + MEpsilon<float>());
                }
            }

            dphi_nu -= (tmp.transpose() * dz(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all))(0, 0);

            // KIRI_LOG_DEBUG("dphi_nu={0}", dphi_nu);

            //

            auto ci1 = this->computeConstrainsts(mRealData + alpha_smax * dx);

            double phi_nu1 = static_cast<double>(TargetFunc(mDual2ndData + alpha_smax * dx, mRadius));
            phi_nu1 -= mMu * log((mSlack + alpha_smax * ds).array()).sum();
            phi_nu1 += mNu * abs(ci1.array() - (mSlack + alpha_smax * ds).array()).sum();
            // KIRI_LOG_DEBUG("phi_nu1={0}", phi_nu1);

            double phi1_next = static_cast<double>(phi_nu) + alpha_smax * mEta * dphi_nu;
            // KIRI_LOG_DEBUG("phi1_next={0}", phi1_next);

            if (phi_nu1 > phi1_next)
            {
                // KIRI_LOG_DEBUG("phi_nu1 > phi1_next");

                MatrixXd con_old = ci - mSlack;
                MatrixXd con_new = ci1 - (mSlack + alpha_smax * ds);

                // KIRI_LOG_DEBUG("con_old={0}", con_old);
                // KIRI_LOG_DEBUG("con_new={0}", con_new);

                auto con_old_sum = con_old.array().abs().sum();
                auto con_new_sum = con_new.array().abs().sum();

                // KIRI_LOG_DEBUG("con_old_sum={0}, con_new_sum={1}", con_old_sum, con_new_sum);
                if (con_new_sum > con_old_sum)
                {
                    KIRI_LOG_INFO("con_new_sum > con_old_sum");
                    auto A = mConstrainJacobianWithIdentity.transpose();

                    // KIRI_LOG_DEBUG("A={0}", A);
                    //  dzp = -(A.inverse() * con_new);
                    VectorXd b = con_new.array();
                    dzp = -A.colPivHouseholderQr().solve(b);

                    // std::cout << get_shape(dzp) << std::endl;
                    // std::cout << get_shape(b) << std::endl;

                    auto ci2 = this->computeConstrainsts(mRealData + alpha_smax * dx + dzp(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all));
                    double phi_nu2 = static_cast<double>(TargetFunc(mDual2ndData + alpha_smax * dx + dzp(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all), mRadius));
                    phi_nu2 -= mMu * log((mSlack + alpha_smax * ds + dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all)).array()).sum();
                    phi_nu2 += mNu * abs(ci2.array() - (mSlack + alpha_smax * ds + dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all)).array()).sum();

                    double phi2_next = static_cast<double>(phi_nu) + alpha_smax * mEta * dphi_nu;

                    if (phi_nu2 <= phi2_next)
                    {
                        alpha_corr = computeMaximumStepSize(mSlack.array(), (alpha_smax * ds + dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all)).array());

                        ci2 = this->computeConstrainsts(mRealData + alpha_corr * (alpha_smax * dx + dzp(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all)));
                        phi_nu2 = static_cast<double>(TargetFunc(mDual2ndData + alpha_corr * (alpha_smax * dx + dzp(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all)), mRadius));
                        phi_nu2 -= mMu * log((mSlack + alpha_corr * (alpha_smax * ds + dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all))).array()).sum();
                        phi_nu2 += mNu * abs(ci2.array() - (mSlack + alpha_corr * (alpha_smax * ds + dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all))).array()).sum();

                        if (phi_nu2 <= phi2_next)
                        {
                            KIRI_LOG_INFO("Second-order feasibility correction accepted");
                            correction = true;
                        }
                    }
                }

                if (!correction)
                {
                    alpha_smax *= mTau;
                    alpha_lmax *= mTau;

                    ci1 = this->computeConstrainsts(mRealData + alpha_smax * dx);

                    phi_nu1 = static_cast<double>(TargetFunc(mDual2ndData + alpha_smax * dx, mRadius));
                    phi_nu1 -= mMu * log((mSlack + alpha_smax * ds).array()).sum();
                    phi_nu1 += mNu * abs(ci1.array() - (mSlack + alpha_smax * ds).array()).sum();

                    phi1_next = static_cast<double>(phi_nu) + alpha_smax * mEta * dphi_nu;

                    while (phi_nu1 > phi1_next)
                    {
                        if (sqrt((alpha_smax * dx).norm() * (alpha_smax * dx).norm() + (alpha_lmax * ds).norm() * (alpha_lmax * ds).norm()) < MEpsilon<float>())
                        {
                            mSignal = -2;
                            KIRI_LOG_INFO(" search direction is unreliable to machine precision, stop solver");
                            return;
                        }

                        alpha_smax *= mTau;
                        alpha_lmax *= mTau;

                        // update
                        ci1 = this->computeConstrainsts(mRealData + alpha_smax * dx);

                        phi_nu1 = static_cast<double>(TargetFunc(mDual2ndData + alpha_smax * dx, mRadius));
                        phi_nu1 -= mMu * log((mSlack + alpha_smax * ds).array()).sum();
                        phi_nu1 += mNu * abs(ci1.array() - (mSlack + alpha_smax * ds).array()).sum();

                        phi1_next = static_cast<double>(phi_nu) + alpha_smax * mEta * dphi_nu;
                    }

                    // KIRI_LOG_DEBUG("alpha_smax={0} ; alpha_lmax={1}", alpha_smax, alpha_lmax);
                }
            }

            // update slack values
            if (correction)
            {
                mSlack += alpha_corr * (alpha_smax * ds + dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all));
            }
            else
            {
                mSlack += alpha_smax * ds;
            }

            // update weights
            if (correction)
            {
                mRealData += alpha_corr * (alpha_smax * dx + dzp(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all));
                mDual2ndData += alpha_corr * (alpha_smax * dx + dzp(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all));
            }
            else
            {
                mRealData += alpha_smax * dx;
                mDual2ndData += alpha_smax * dx;
            }

            // update lambda multiplier
            mLambda += alpha_lmax * dl;

            // KIRI_LOG_DEBUG("mLambda={0}", mLambda);
            KIRI_LOG_DEBUG("searched new x={0}", mRealData.transpose());
        }
    };

    typedef std::shared_ptr<InteriorPointMethod> InteriorPointMethodPtr;

} // namespace OPTIMIZE::IPM

#endif /* _IPM_H_ */