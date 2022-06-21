/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-15 23:16:19
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-21 09:06:38
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

real Constrainst0(const ArrayXreal &x)
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
            this->computeConstrainsts();
            this->computeConstrainstsJacobian();
            this->initSlack();
            this->initLambdaMultipler();
            this->computeKKT();
            this->computeConstrainstsHessian();
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
        bool mKtolConverged = false;
        bool mFtolConverged = false;

        dual2nd mPhi0, mLastPhi0;
        VectorXdual mFuncGrad;
        MatrixXd mFuncHessian;
        MatrixXd mGrad;

        std::vector<double> mData;
        ArrayXreal mRealData;
        VectorXdual2nd mDual2ndData;

        MatrixXd mCi;
        MatrixXd mConstrainJacobian;

        MatrixXd mConstraintsFuncHessian;
        MatrixXd mHessian;

        MatrixXd mLambda, mSlack;
        MatrixXd mKKT1, mKKT2, mKKT3, mKKT4;

        void initDataVector()
        {
            ArrayXreal real_data(mData.size());
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

        void computeConstrainsts()
        {
            auto constrain0 = double(Constrainst0(mRealData));
            VectorXd constrain1 = Constrainst1(mRealData).cast<double>();

            MatrixXd constrain_i(constrain1.size() + 1, 1);
            constrain_i << constrain0, constrain1;

            mCi = constrain_i;

            // std::cout << "mCi = \n"
            //           << mCi << std::endl;
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
            // KIRI_LOG_DEBUG("hessianMatrix=\n{0}", hessianMatrix);
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

            mKKT4 = mCi - mSlack;

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
            // auto correction = false;
            // auto alphaCorr = 1.0;

            // mPhi0 = targetFunc(mData);

            // auto dir = -(mFuncHessian.inverse() * mFuncGrad);
            // std::cout << "search direction=" << dir.cast<dual2nd>() << std::endl;

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