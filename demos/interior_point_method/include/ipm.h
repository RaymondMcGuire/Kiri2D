/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-25 01:39:47
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-26 16:37:54
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

// dual2nd targetFunc(const VectorXdual2nd &x)
// {
//     return x[0] * x[0] - 4 * x[0] + x[1] * x[1] - x[1] - x[0] * x[1];
// }

// minimize f(x, y) = 100*(y - x**2)**2 + (1 - x)**2
// dual2nd targetFunc(const VectorXdual2nd &x)
// {
//     return 100 * (x[1] - x[0] * x[0]) * (x[1] - x[0] * x[0]) + (1 - x[0]) * (1 - x[0]);
// }

real Constrainst0(const VectorXreal &x)
{
    return x[0] + 2 * x[1] - 10;
}

VectorXreal Constrainst1(const VectorXreal &x)
{
    return x;
}

dual2nd ConstrainstFunc(const VectorXdual2nd &x, const MatrixXd &lambda)
{
    return (x[0] + 2 * x[1] - 10) * lambda(0, 0) + x[0] * lambda(1, 0) + x[1] * lambda(2, 0);
}

dual2nd TargetFunc(const VectorXdual2nd &x)
{
    return x[0] * x[0] + 2 * x[1] * x[1] + 2 * x[0] + 8 * x[1];
}

namespace OPTIMIZE::IPM
{
    class InteriorPointMethod
    {
    public:
        explicit InteriorPointMethod(const std::vector<double> &data) : mData(data)
        {
            this->initDataVector();
            this->initHessian();
            this->computeConstrainstsJacobian();
            this->initSlack();
            this->initLambdaMultipler();
            this->computeKKT();
            this->computeConstrainstsHessian();
            this->computeBarriarCostGrad();
            this->computeBackTrackingLineSearch();
        }

        virtual ~InteriorPointMethod()
        {
        }

        void solve()
        {

            this->computeKKT();

            mLastPhi0 = mPhi0;

            for (auto i = 0; i < mOuterIterNum; i++)
            {
                if (mKKT1.norm() <= mKtol && mKKT2.norm() <= mKtol && mKKT3.norm() <= mKtol && mKKT4.norm() <= mKtol)
                {
                    mSignal = 1;
                    mKtolConverged = true;
                    break;
                }

                for (auto j = 0; j < mInnerIterNum; j++)
                {
                    // judge kkt
                    if (mKKT1.norm() <= mKtol && mKKT2.norm() <= mKtol && mKKT3.norm() <= mKtol && mKKT4.norm() <= mKtol)
                    {
                        mSignal = 1;
                        mKtolConverged = true;
                        break;
                    }

                    // compute gradient and hessian

                    computeBackTrackingLineSearch();

                    mIterCount++;

                    this->computeKKT();

                    std::cout << "mLastPhi0:" << mLastPhi0 << "; mPhi0:" << mPhi0 << std::endl;
                    if (abs(mLastPhi0 - mPhi0) <= abs(mFtol))
                    {
                        mSignal = 2;
                        mFtolConverged = true;
                        break;
                    }
                    else
                        mLastPhi0 = mPhi0;

                    if (mSignal == -2)
                        break;
                }

                if (mSignal == -2 || mFtolConverged)
                    break;
            }

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
        int mOuterIterNum = 1;
        int mInnerIterNum = 20;

        double mEta = 1e-4;
        double mKtol = 1e-4;
        double mFtol = 1e-8;
        double mTau = 0.995;
        double mMu = 0.2;
        double mNu = 10.0;
        double mRho = 0.1;
        bool mKtolConverged = false;
        bool mFtolConverged = false;

        dual2nd mPhi0, mLastPhi0;
        VectorXdual mFuncGrad;
        MatrixXd mFuncHessian;

        std::vector<double> mData;
        VectorXreal mRealData;
        VectorXdual2nd mDual2ndData;

        MatrixXd mConstrainJacobian;
        MatrixXd mConstraintsFuncHessian;

        MatrixXd mGrad;
        MatrixXd mHessian;

        MatrixXd mLambda, mSlack;
        MatrixXd mKKT1, mKKT2, mKKT3, mKKT4;

        MatrixXd mBarrierCostGrad;

        void initDataVector()
        {
            VectorXreal real_data(mData.size());
            VectorXdual2nd dual2nd_data(mData.size());

            for (auto i = 0; i < mData.size(); i++)
            {
                real_data[i] = mData[i];
                dual2nd_data[i] = mData[i];
            }

            mRealData = real_data;
            mDual2ndData = dual2nd_data;
        }

        void initSlack()
        {
            MatrixXd slack(3, 1);
            slack << 1e-4, 1e-4, 1e-4;

            mSlack = slack;

            // std::cout << "mSlack = \n"
            //           << mSlack << std::endl;
        }

        void initLambdaMultipler()
        {
            MatrixXd lambda = mConstrainJacobian.completeOrthogonalDecomposition().pseudoInverse() * mFuncGrad.cast<double>();

            mLambda = lambda;
            for (auto i = 0; i < mLambda.rows(); i++)
                for (auto j = 0; j < mLambda.cols(); j++)
                    if (mLambda(i, j) < 0.0)
                        mLambda(i, j) = 1e-4;

            // std::cout << "init mLambda = \n"
            //           << mLambda << std::endl;
        }

        void initHessian()
        {
            mFuncHessian = hessian(TargetFunc, wrt(mDual2ndData), at(mDual2ndData), mPhi0, mFuncGrad);

            KIRI_LOG_DEBUG("mFuncHessian={0}", mFuncHessian);
        }

        MatrixXd computeConstrainsts(VectorXreal realData)
        {
            auto constrain0 = double(Constrainst0(realData));
            VectorXd constrain1 = Constrainst1(realData).cast<double>();

            MatrixXd constrain_i(constrain1.size() + 1, 1);
            constrain_i << constrain0, constrain1;

            return constrain_i;
        }

        void computeConstrainstsJacobian()
        {
            real u;
            VectorXd grad0 = gradient(Constrainst0, wrt(mRealData), at(mRealData), u);

            VectorXreal F;
            MatrixXd grad1 = jacobian(Constrainst1, wrt(mRealData), at(mRealData), F);

            MatrixXd constrainst_jacobian(grad0.rows(), grad0.cols() + grad1.cols());
            constrainst_jacobian << grad0, grad1;
            mConstrainJacobian = constrainst_jacobian;
        }

        void computeBarriarCostGrad()
        {
            MatrixXd barriar_cost_grad = MatrixXd::Zero(2 + 3, 1);
            MatrixXd func_grad = mFuncGrad.cast<double>();
            barriar_cost_grad << func_grad, -mMu / (mSlack.array() + MEpsilon<float>());
            mBarrierCostGrad = barriar_cost_grad;
            KIRI_LOG_DEBUG("mBarrierCostGrad=\n{0}", mBarrierCostGrad);
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

        void UpdateMeritFuncParams()
        {
        }

        void computeConstrainstsHessian()
        {

            dual2nd u;
            VectorXdual g;
            mConstraintsFuncHessian = hessian(ConstrainstFunc, wrt(mDual2ndData), at(mDual2ndData, mLambda), u, g);

            KIRI_LOG_DEBUG("mConstraintsFuncHessian={0}", mConstraintsFuncHessian);

            // Hessian of the Lagrangian
            MatrixXd d2L = mFuncHessian - mConstraintsFuncHessian;

            KIRI_LOG_DEBUG("d2L={0}", d2L);

            MatrixXd d2LUpper = d2L.triangularView<Eigen::Upper>();
            KIRI_LOG_DEBUG("d2LUpper={0}", d2LUpper);

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
            KIRI_LOG_DEBUG("sigma={0}", SigmaDiag);

            MatrixXd eyeConstrainMatrix = -MatrixXd::Identity(3, 3);

            MatrixXd HessianColConcate = MatrixXd::Zero(mData.size(), mData.size() + 2 * 3);
            MatrixXd VarConstrainZeroMatrix = MatrixXd::Zero(mData.size(), 3);
            HessianColConcate << d2LUpper, VarConstrainZeroMatrix, mConstrainJacobian;

            MatrixXd HessianColConcate1 = MatrixXd::Zero(3, mData.size() + 2 * 3);
            MatrixXd ConstrainVarZeroMatrix = MatrixXd::Zero(3, mData.size());
            HessianColConcate1 << ConstrainVarZeroMatrix, SigmaDiag, eyeConstrainMatrix;

            MatrixXd HessianRowConcate = MatrixXd::Zero(mData.size() + 3, mData.size() + 2 * 3);
            HessianRowConcate << HessianColConcate, HessianColConcate1;

            MatrixXd ConstrainVarPlusTwoConstrainMatrix = MatrixXd::Zero(3, mData.size() + 2 * 3);

            MatrixXd hessianMatrix(mData.size() + 2 * 3, mData.size() + 2 * 3);
            hessianMatrix << HessianRowConcate, ConstrainVarPlusTwoConstrainMatrix;

            MatrixXd hessianupperMatrix = hessianMatrix.triangularView<Eigen::Upper>();
            hessianMatrix = hessianupperMatrix + hessianupperMatrix.transpose();

            MatrixXd hessianHalfDiagMatrix = (hessianMatrix.diagonal() / 2).array().matrix().asDiagonal();
            hessianMatrix -= hessianHalfDiagMatrix;

            mHessian = hessianMatrix;
            KIRI_LOG_DEBUG("hessianMatrix=\n{0}", hessianMatrix);
        }

        void computeKKT()
        {

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

            mKKT1 = mFuncGrad.cast<double>() - mConstrainJacobian * mLambda;

            mKKT2 = kkt2;

            mKKT3 = MatrixXd::Zero(1, 1);

            auto ci = this->computeConstrainsts(mRealData);
            mKKT4 = ci - mSlack;

            KIRI_LOG_DEBUG("KKT1=\n{0}", mKKT1);
            KIRI_LOG_DEBUG("KKT2=\n{0}", mKKT2);
            KIRI_LOG_DEBUG("KKT3=\n{0}", mKKT3);
            KIRI_LOG_DEBUG("KKT4=\n{0}", mKKT4);

            mGrad = MatrixXd(mKKT1.rows() + mKKT2.rows() + mKKT4.rows(), 1);
            mGrad << mKKT1, grad2, mKKT4;
            KIRI_LOG_DEBUG("mGrad=\n{0}", mGrad);
        }

        void computeBackTrackingLineSearch(double alphaSMax = 1.0, double alphaLMax = 1.0)
        {
            auto correction = false;
            auto alpha_corr = 1.0;

            // mPhi0 = targetFunc(mData);

            MatrixXd dir = -(mHessian.inverse() * mGrad);

            // KIRI_LOG_DEBUG("search direction=\n{0}", dir);

            // change sign definition for the multipliers' search direction
            for (auto i = 5; i < dir.size(); i++)
            {
                dir(i, 0) = -dir(i, 0);
            }
            // KIRI_LOG_DEBUG("changed sign search direction=\n{0}", dir);

            auto dot_barrier_dir = mBarrierCostGrad.transpose() * dir(Eigen::seq(0, 4), Eigen::placeholders::all);
            // KIRI_LOG_DEBUG("dot_barrier_dir=\n{0}", dot_barrier_dir);
            // KIRI_LOG_DEBUG("dz=\n{0}", dir(Eigen::seq(0, 4), Eigen::placeholders::all));

            auto ci = this->computeConstrainsts(mRealData);
            MatrixXd con = ci - mSlack;
            auto sum_abs_con = con.array().abs().sum();
            // KIRI_LOG_DEBUG("sum_abs_con={0}", sum_abs_con);

            auto nu_thresh = dot_barrier_dir / (1 - mRho) / sum_abs_con;
            // KIRI_LOG_DEBUG("nu_thresh={0}", nu_thresh);

            auto alpha_smax = computeMaximumStepSize(mSlack.array(), dir(Eigen::seq(2, 4), Eigen::placeholders::all).array());

            // KIRI_LOG_DEBUG("alpha_smax={0}", alpha_smax);

            // KIRI_LOG_DEBUG("mLambda={0}", mLambda);
            // KIRI_LOG_DEBUG("dir(Eigen::seq(2, 4), Eigen::placeholders::all).array()={0}", dir(Eigen::seq(5, dir.size() - 1), Eigen::placeholders::all).array());
            auto alpha_lmax = computeMaximumStepSize(mLambda.array(), dir(Eigen::seq(5, dir.size() - 1), Eigen::placeholders::all).array());

            // KIRI_LOG_DEBUG("alpha_lmax={0}", alpha_lmax);

            MatrixXd dx = dir(Eigen::seq(0, 1), Eigen::placeholders::all);
            MatrixXd ds = dir(Eigen::seq(2, 4), Eigen::placeholders::all);
            MatrixXd dl = dir(Eigen::seq(5, dir.size() - 1), Eigen::placeholders::all);
            KIRI_LOG_DEBUG("ds={0}", dl);

            // compute merit function
            auto phi_nu = eval(TargetFunc(mDual2ndData));
            phi_nu -= mMu * log(mSlack.array()).sum();
            phi_nu += mNu * abs(ci.array() - mSlack.array()).sum();
            KIRI_LOG_DEBUG("phi_nu={0}", phi_nu);

            MatrixXd func_grad = mFuncGrad.cast<double>();
            auto dphi_nu = (func_grad.transpose() * dir(Eigen::seq(0, 1), Eigen::placeholders::all))(0, 0);
            dphi_nu -= mNu * abs(ci.array() - mSlack.array()).sum();

            MatrixXd tmp = mSlack;
            for (auto i = 0; i < tmp.rows(); i++)
            {
                for (auto j = 0; j < tmp.cols(); j++)
                {
                    tmp(i, j) = mMu / (mSlack(i, j) + MEpsilon<float>());
                }
            }

            dphi_nu -= (tmp.transpose() * dir(Eigen::seq(2, 4), Eigen::placeholders::all))(0, 0);

            KIRI_LOG_DEBUG("dphi_nu={0}", dphi_nu);

            //

            auto ci1 = this->computeConstrainsts(mRealData + alpha_smax * dx);

            auto phi_nu1 = eval(TargetFunc(mDual2ndData + alpha_smax * dx));
            phi_nu1 -= mMu * log((mSlack + alpha_smax * ds).array()).sum();
            phi_nu1 += mNu * abs(ci1.array() - (mSlack + alpha_smax * ds).array()).sum();
            KIRI_LOG_DEBUG("phi_nu1={0}", phi_nu1);

            auto phi1_next = phi_nu + alpha_smax * mEta * dphi_nu;
            KIRI_LOG_DEBUG("phi1_next={0}", eval(phi1_next));

            if (phi_nu1 > phi1_next)
            {
                KIRI_LOG_DEBUG("phi_nu1 > phi1_next");
            }

            // update slack values
            if (correction)
            {
                // TODO
            }
            else
            {
                mSlack += alpha_smax * ds;
            }

            // update weights
            if (correction)
            {
                // TODO
            }
            else
            {
                mRealData += alpha_smax * dx;
                mDual2ndData += alpha_smax * dx;
            }

            // update lambda multiplier
            mLambda += alpha_lmax * dl;

            KIRI_LOG_DEBUG("mLambda={0}", mLambda);

            // auto dphi0 = (mFuncGrad.transpose() * dir).cast<dual2nd>()(0, 0);

            // // equality constraints or unconstrained problems
            // auto phi1 = targetFunc(mData + alphaSMax * dir.cast<dual2nd>());
            // auto phi1_prime = mPhi0 + alphaSMax * mEta * dphi0;
            // // std::cout << "phi1=" << phi1 << "; phi1 prime=" << phi1_prime << std::endl;
            // if (phi1 > phi1_prime)
            // {
            //     if (!correction)
            //     {
            //         alphaSMax *= mTau;
            //         alphaLMax *= mTau;

            //         while (targetFunc(mData + alphaSMax * dir.cast<dual2nd>()) > mPhi0 + alphaSMax * mEta * dphi0)
            //         {
            //             // backtracking line search
            //             if ((alphaSMax * dir.cast<dual2nd>()).norm() < std::numeric_limits<dual2nd>::epsilon())
            //             {
            //                 // search direction is unreliable to machine precision, stop solver
            //                 mSignal = -2;
            //                 return;
            //             }
            //             alphaSMax *= mTau;
            //             alphaLMax *= mTau;
            //         }
            //     }
            // }

            // if (correction)
            //     mData += alphaSMax * dir.cast<dual2nd>();
            // else
            //     mData += alphaSMax * dir.cast<dual2nd>();

            // std::cout << "searched new=" << mData << std::endl;
        }
    };

    typedef std::shared_ptr<InteriorPointMethod> InteriorPointMethodPtr;

} // namespace OPTIMIZE::IPM

#endif /* _IPM_H_ */