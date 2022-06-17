/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-15 23:16:19
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-17 10:50:03
 * @FilePath: \Kiri2D\demos\interior_point_method\include\ipm.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _IPM_H_
#define _IPM_H_

#pragma once

#include <sstream>
#include <Eigen/LU>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

using Eigen::EigenBase;
using std::ostringstream;
using namespace autodiff;

// dual2nd targetFunc(const VectorXdual2nd &x)
// {
//     return x[0] * x[0] - 4 * x[0] + x[1] * x[1] - x[1] - x[0] * x[1];
// }

// minimize f(x, y) = 100*(y - x**2)**2 + (1 - x)**2
dual2nd targetFunc(const VectorXdual2nd &x)
{
    return 100 * (x[1] - x[0] * x[0]) * (x[1] - x[0] * x[0]) + (1 - x[0]) * (1 - x[0]);
}

namespace OPTIMIZE::IPM
{
    class InteriorPointMethod
    {
    public:
        explicit InteriorPointMethod(const VectorXdual2nd &data) : mData(data)
        {
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
                if (mGrad.norm() <= mKtol)
                {
                    mSignal = 1;
                    mKtolConverged = true;
                    break;
                }

                for (auto j = 0; j < mInnerIterNum; j++)
                {
                    // judge kkt
                    if (mGrad.norm() <= mKtol)
                    {
                        mSignal = 1;
                        mKtolConverged = true;
                        break;
                    }

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
            std::cout << "mGrad = \n"
                      << mGrad << std::endl;
            std::cout << "mHessian = \n"
                      << mHessian << std::endl;
        }

    private:
        int mIterCount = 0;
        int mSignal = 0;
        int mOuterIterNum = 10;
        int mInnerIterNum = 20;

        double mEta = 1e-4;
        double mKtol = 1e-4;
        double mFtol = 1e-8;
        double mTau = 0.995;
        bool mKtolConverged = false;
        bool mFtolConverged = false;

        dual2nd mPhi0, mLastPhi0;
        VectorXdual mGrad;
        Eigen::MatrixXd mHessian;
        VectorXdual2nd mData;

        void computeKKT()
        {
            mHessian = hessian(targetFunc, wrt(mData), at(mData), mPhi0, mGrad);
        }

        void computeBackTrackingLineSearch(double alphaSMax = 1.0, double alphaLMax = 1.0)
        {
            auto correction = false;
            auto alphaCorr = 1.0;

            mPhi0 = targetFunc(mData);

            auto dir = -(mHessian.inverse() * mGrad);
            std::cout << "search direction=" << dir.cast<dual2nd>() << std::endl;

            auto dphi0 = (mGrad.transpose() * dir).cast<dual2nd>()(0, 0);

            // equality constraints or unconstrained problems
            auto phi1 = targetFunc(mData + alphaSMax * dir.cast<dual2nd>());
            auto phi1_prime = mPhi0 + alphaSMax * mEta * dphi0;
            // std::cout << "phi1=" << phi1 << "; phi1 prime=" << phi1_prime << std::endl;
            if (phi1 > phi1_prime)
            {
                if (!correction)
                {
                    alphaSMax *= mTau;
                    alphaLMax *= mTau;

                    while (targetFunc(mData + alphaSMax * dir.cast<dual2nd>()) > mPhi0 + alphaSMax * mEta * dphi0)
                    {
                        // backtracking line search
                        if ((alphaSMax * dir.cast<dual2nd>()).norm() < std::numeric_limits<dual2nd>::epsilon())
                        {
                            // search direction is unreliable to machine precision, stop solver
                            mSignal = -2;
                            return;
                        }
                        alphaSMax *= mTau;
                        alphaLMax *= mTau;
                    }
                }
            }

            if (correction)
                mData += alphaSMax * dir.cast<dual2nd>();
            else
                mData += alphaSMax * dir.cast<dual2nd>();

            std::cout << "searched new=" << mData << std::endl;
        }
    };

    typedef std::shared_ptr<InteriorPointMethod> InteriorPointMethodPtr;

} // namespace OPTIMIZE::IPM

#endif /* _IPM_H_ */