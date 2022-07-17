/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-14 12:35:07
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-14 13:23:46
 * @FilePath: \Kiri2D\demos\primal_dual_ipm\include\primal_dual_ipm.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _PRIMAL_DUAL_IPM_H_
#define _PRIMAL_DUAL_IPM_H_

#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
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
using Eigen::VectorXcd;
using Eigen::VectorXd;

VectorXreal EquConstrainsts(const VectorXreal &x)
{
  VectorXreal consts(1);
  consts[0] = x.sum() - 1.0;

  return consts;
}

VectorXreal InEquConstrainsts(const VectorXreal &x)
{
  VectorXreal consts(3);
  consts[0] = x[0];
  consts[1] = x[1];
  consts[2] = x[2];
  return consts;
}

dual2nd EquConstrainstFunc(const VectorXdual2nd &x, const MatrixXd &lambda)
{
  return x.sum() * lambda(0, 0);
}

dual2nd InEquConstrainstFunc(const VectorXdual2nd &x, const MatrixXd &lambda, const int equNum)
{
  return x[0] * lambda(equNum + 0, 0) + x[1] * lambda(equNum + 1, 0) + x[2] * lambda(equNum + 2, 0);
}

dual2nd TargetFunc(const VectorXdual2nd &x)
{
  return -x[0] * x[1] * x[2];
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
  class PrimalDualIPM
  {
  public:
    explicit PrimalDualIPM(const std::vector<double> &data, int equNum, int inEquNum)
        : mData(data), mVariableNum(data.size()), mEquNum(equNum), mInEquNum(inEquNum)
    {
      this->initDataVector();
      this->initSlack();
      this->initLambdaMultipler();

      this->solve();
    }

    virtual ~PrimalDualIPM() {}

    void solve()
    {
      mFTolConverged = false;
      mFLast = static_cast<double>(TargetFunc(mDual2ndData));

      this->computeKKT();

      for (auto i = 0; i < mOuterIterNum; i++)
      {
        // KIRI_LOG_DEBUG("outer iter num={0}", i);

        if (mKKT1.norm() <= mKTol && mKKT2.norm() <= mKTol &&
            mKKT3.norm() <= mKTol && mKKT4.norm() <= mKTol)
        {
          mSignal = 1;
          mKtolConverged = true;
          // KIRI_LOG_INFO("mKtolConverged=true");
          break;
        }

        for (auto j = 0; j < mInnerIterNum; j++)
        {
          mMuTol = std::max(mKTol, mMu);
          if (mKKT1.norm() <= mMuTol && mKKT2.norm() <= mMuTol &&
              mKKT3.norm() <= mMuTol && mKKT4.norm() <= mMuTol)
          {
            mSignal = 1;
            mKtolConverged = true;
            // KIRI_LOG_INFO("mKtolConverged=true");
            break;
          }

          // KIRI_LOG_DEBUG("inner iter num={0}", j);

          // KIRI_LOG_DEBUG("computeConstrainstsHessian start");

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

          // KIRI_LOG_DEBUG("computeBarriarCostGrad start");

          this->computeBarriarCostGrad();
          auto dot_barrier_dir = (mBarrierCostGrad.transpose() *
                                  dz(Eigen::seq(0, mVariableNum + mInEquNum - 1),
                                     Eigen::placeholders::all))(0, 0);
          // KIRI_LOG_DEBUG("dot_barrier_dir=\n{0}", dot_barrier_dir);

          // KIRI_LOG_DEBUG("dz=\n{0}", dir(Eigen::seq(0, 4),
          // Eigen::placeholders::all));

          // KIRI_LOG_DEBUG("computeInEquConstrainsts start");

          MatrixXd con = this->computeConstainMatrix(mRealData, mSlack);
          double sum_abs_con = con.array().abs().sum();
          // KIRI_LOG_DEBUG("sum_abs_con={0}", sum_abs_con);

          //! FIXME sum_abs_con=0???
          if (sum_abs_con != 0.0)
          {
            double nu_thresh = dot_barrier_dir / (1 - mRho) / sum_abs_con;
            KIRI_LOG_DEBUG("nu_thresh={0}", nu_thresh);

            if (mNu < nu_thresh)
            {
              mNu = nu_thresh;
              KIRI_LOG_DEBUG("update nu = {0}", mNu);
            }
          }

          // KIRI_LOG_DEBUG("computeBackTrackingLineSearch start");
          computeBackTrackingLineSearch(dz);

          mIterCount++;

          // KIRI_LOG_DEBUG("computeKKT start");
          this->computeKKT();

          if (mSignal == -2)
            break;
        }

        if (mSignal != -2)
        {
          mFNew = static_cast<double>(TargetFunc(mDual2ndData));
          // KIRI_LOG_DEBUG("new={0}; past={1}", mFNew, mFLast);
          if (abs(mFLast - mFNew) < abs(mFTol))
          {
            mSignal = 2;
            mFTolConverged = true;
            KIRI_LOG_INFO("mKtolConverged=true");
          }
          else
            mFLast = mFNew;
        }

        if (mSignal == -2 || mFTolConverged)
          break;

        // update barrier params
        MatrixXd lambda_ineq = mLambda(Eigen::seq(mEquNum, mLambda.rows() - 1), Eigen::placeholders::all);
        auto xi = mInEquNum * (mSlack.array() * lambda_ineq.array()).minCoeff() /
                  ((mSlack.transpose() * lambda_ineq)(0, 0) + MEpsilon<float>());
        // KIRI_LOG_DEBUG("xi={0}", xi);

        mMu =
            0.1 *
            pow(std::min(0.05 * (1.0 - xi) / (xi + MEpsilon<float>()), 2.0), 3) *
            ((mSlack.transpose() * lambda_ineq)(0, 0) / mInEquNum);
        if (mMu < 0)
          mMu = 0;
        // KIRI_LOG_DEBUG("mMu={0}", mMu);
      }

      KIRI_LOG_INFO("optimal solution={0}; func={1}", mRealData.transpose(),
                    -static_cast<double>(TargetFunc(mDual2ndData)));

      // print();
    }

    void print()
    {
      // std::cout << "mFuncGrad = \n"
      //           << mFuncGrad << std::endl;
      // std::cout << "mFuncHessian = \n"
      //           << mFuncHessian << std::endl;
    }

    VectorXreal solution() { return mRealData; }

  private:
    int mIterCount = 0;
    int mSignal = 0;
    int mEquNum = 0;
    int mInEquNum = 0;
    int mVariableNum = 0;
    int mOuterIterNum = 10;
    int mInnerIterNum = 20;
    // int mOuterIterNum = 1;
    // int mInnerIterNum = 1;

    bool mHasEqualConstrainsts = false;
    bool mHasInEqualConstrainsts = false;

    double mEta = 1e-4;
    double mKTol = 1e-4;
    double mMuTol = 1e-4;
    double mFTol = 1e-8;
    double mTau = 0.995;
    double mMu = 0.2;
    double mNu = 10.0;
    double mRho = 0.1;
    double mDelta = 0.0;
    double mBeta = 0.4;
    bool mKtolConverged = false;
    bool mFTolConverged = false;

    double mFLast, mFNew = 0.0;

    dual2nd mPhi0, mLastPhi0;
    VectorXdual mFuncGrad;
    MatrixXd mFuncHessian;

    std::vector<double> mData;

    VectorXreal mRealData;
    VectorXdual2nd mDual2ndData;

    MatrixXd mConstrainJacobian;
    MatrixXd mJaco, mDce, mDci;
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
      MatrixXd ci = this->computeInEquConstrainsts(mRealData);

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

      MatrixXd lambda =
          mJaco(Eigen::seq(0, mVariableNum - 1),
                Eigen::placeholders::all)
              .completeOrthogonalDecomposition()
              .pseudoInverse() *
          mFuncGrad.cast<double>();

      // KIRI_LOG_DEBUG("init Lambda=\n {0}", lambda);
      // KIRI_LOG_DEBUG("Lambda Shape=\n {0}", get_shape(lambda));

      mLambda = lambda;

      if (mEquNum > 0 && mInEquNum > 0)
      {
        for (auto i = mEquNum; i < mLambda.rows(); i++)
          for (auto j = 0; j < mLambda.cols(); j++)
            if (mLambda(i, j) < 0.0)
              mLambda(i, j) = 1e-4;
      }
      else if (mInEquNum > 0)
      {
        for (auto i = 0; i < mLambda.rows(); i++)
          for (auto j = 0; j < mLambda.cols(); j++)
            if (mLambda(i, j) < 0.0)
              mLambda(i, j) = 1e-4;
      }

      // KIRI_LOG_DEBUG("edited Lambda=\n {0}", mLambda);
    }

    void computeHessian()
    {
      mFuncHessian = hessian(TargetFunc, wrt(mDual2ndData), at(mDual2ndData),
                             mPhi0, mFuncGrad);

      // KIRI_LOG_DEBUG("mFuncHessian={0}", mFuncHessian);
    }

    //! TODO
    MatrixXd computeInEquConstrainsts(VectorXreal realData)
    {
      // auto constrain0 = double(Constrainst0(realData));
      // VectorXd constrain1 = Constrainst1(realData).cast<double>();

      VectorXd constrain = InEquConstrainsts(realData).cast<double>();

      MatrixXd constrain_i(mInEquNum, 1);
      constrain_i << constrain;

      return constrain_i;
    }

    MatrixXd computeConstainMatrix(VectorXreal data, MatrixXd slack)
    {
      auto ci = this->computeInEquConstrainsts(mRealData);
      auto ce = this->computeEquConstrainsts(mRealData);
      MatrixXd con = MatrixXd(mEquNum + mInEquNum, 1);

      con.block(0, 0, mEquNum, con.cols()) = ce;
      con.block(mEquNum, 0, con.rows() - mEquNum, con.cols()) = ci - slack;

      return con;
    }

    MatrixXd computeEquConstrainsts(VectorXreal realData)
    {
      VectorXd constrain = EquConstrainsts(realData).cast<double>();

      MatrixXd constrain_e(mEquNum, 1);
      constrain_e << constrain;

      return constrain_e;
    }

    void computeConstrainstsJacobian()
    {
      VectorXreal val_ce, val_ci;
      MatrixXd dce =
          jacobian(EquConstrainsts, wrt(mRealData), at(mRealData), val_ce);
      MatrixXd dci =
          jacobian(InEquConstrainsts, wrt(mRealData), at(mRealData), val_ci);

      mDce = dce.transpose();
      mDci = dci.transpose();

      MatrixXd constrainst_jacobian(mDce.rows(), mDce.cols() + mDci.cols());
      constrainst_jacobian << mDce, mDci;
      mConstrainJacobian = constrainst_jacobian;

      MatrixXd ce_zero_matrix =
          MatrixXd::Zero(mInEquNum, mEquNum);
      MatrixXd eye_constrain_matrix = -MatrixXd::Identity(mInEquNum, mInEquNum);

      MatrixXd combine_ce_eye(mInEquNum, mEquNum + mInEquNum);
      combine_ce_eye << ce_zero_matrix, eye_constrain_matrix;

      MatrixXd jaco(mConstrainJacobian.rows() +
                        combine_ce_eye.rows(),
                    mConstrainJacobian.cols());
      jaco << mConstrainJacobian, combine_ce_eye;
      mJaco = jaco;

      // KIRI_LOG_DEBUG("mJaco=\n{0}", mJaco);
    }

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
      mGrad = MatrixXd(mVariableNum + 2 * mInEquNum + mEquNum, 1);

      MatrixXd grad_var_section = mFuncGrad.cast<double>();
      MatrixXd grad_equ_section, grad_inequ_section, grad_inequ_last_section;

      if (mEquNum > 0)
      {
        grad_var_section -= mDce * mLambda(Eigen::seq(0, mEquNum - 1), Eigen::placeholders::all);

        auto ce = this->computeEquConstrainsts(mRealData);
        grad_equ_section = ce;
      }

      if (mInEquNum > 0)
      {
        MatrixXd lambda_ineq = mLambda(Eigen::seq(mEquNum, mLambda.rows() - 1), Eigen::placeholders::all);
        grad_var_section -= mDci * lambda_ineq;

        auto grad2 = mSlack;
        for (auto i = 0; i < grad2.rows(); i++)
        {
          for (auto j = 0; j < grad2.cols(); j++)
          {
            grad2(i, j) =
                (lambda_ineq(i, j) - mMu / (mSlack(i, j) + MEpsilon<float>()));
          }
        }
        grad_inequ_section = grad2;
        auto ci = this->computeInEquConstrainsts(mRealData);
        grad_inequ_last_section = ci - mSlack;
      }

      if (mEquNum > 0 && mInEquNum > 0)
      {
        mGrad << grad_var_section, grad_inequ_section, grad_equ_section, grad_inequ_last_section;
      }
      else if (mEquNum > 0)
      {
        mGrad << grad_var_section, grad_equ_section;
      }
      else if (mInEquNum > 0)
      {
        mGrad << grad_var_section, grad_inequ_section, grad_inequ_last_section;
      }
      else
      {
        mGrad << grad_var_section;
      }

      // auto grad2 = mSlack;
      // for (auto i = 0; i < grad2.rows(); i++)
      // {
      //   for (auto j = 0; j < grad2.cols(); j++)
      //   {
      //     grad2(i, j) =
      //         (mLambda(i, j) - mMu / (mSlack(i, j) + MEpsilon<float>()));
      //   }
      // }

      // mGrad = MatrixXd(mKKT1.rows() + mKKT2.rows() + mKKT4.rows(), 1);
      // mGrad << mKKT1, grad2, mKKT4;
      // KIRI_LOG_DEBUG("mGrad=\n{0}", mGrad.transpose());
    }

    int computeMatrixInertia(VectorXcd mat)
    {
      auto matrix_inertia = 0;
      for (auto i = 0; i < mat.array().size(); i++)
        if (mat.array()[i].real() < -MEpsilon<float>())
          matrix_inertia++;
      return matrix_inertia;
    }

    void computeConstrainstsHessian()
    {
      mDelta = 0.0;
      this->computeConstrainstsJacobian();

      dual2nd u_ce, u_ci;
      VectorXdual g_ce, g_ci;

      MatrixXd d2ce = hessian(EquConstrainstFunc, wrt(mDual2ndData),
                              at(mDual2ndData, mLambda), u_ce, g_ce);

      MatrixXd d2ci = hessian(InEquConstrainstFunc, wrt(mDual2ndData),
                              at(mDual2ndData, mLambda, mEquNum), u_ci, g_ci);

      // Hessian of the Lagrangian
      MatrixXd d2L = mFuncHessian;
      if (mEquNum > 0)
        d2L -= d2ce;

      if (mInEquNum > 0)
        d2L -= d2ci;
      // KIRI_LOG_DEBUG("d2L=\n{0}", d2L);

      MatrixXd d2L_upper = d2L.triangularView<Eigen::Upper>();
      // KIRI_LOG_DEBUG("d2L_upper={0}", d2L_upper);

      auto sigma = mSlack;
      for (auto i = 0; i < sigma.rows(); i++)
      {
        for (auto j = 0; j < sigma.cols(); j++)
        {
          sigma(i, j) = mLambda(i + mEquNum, j) / (mSlack(i, j) + MEpsilon<float>());
        }
      }

      auto diagSize = sigma.array().size();
      MatrixXd sigma_diag(diagSize, diagSize);

      sigma_diag = sigma.array().matrix().asDiagonal();
      // // KIRI_LOG_DEBUG("sigma={0}", sigma_diag);

      MatrixXd eye_constrain_matrix = -MatrixXd::Identity(mInEquNum, mInEquNum);

      MatrixXd hess_first_row =
          MatrixXd::Zero(mVariableNum, mVariableNum + 2 * mInEquNum + mEquNum);
      MatrixXd first_col_zero_matrix = MatrixXd::Zero(mVariableNum, mInEquNum);
      hess_first_row << d2L_upper, first_col_zero_matrix, mConstrainJacobian;

      MatrixXd hess_second_row =
          MatrixXd::Zero(mInEquNum, mVariableNum + 2 * mInEquNum + mEquNum);
      MatrixXd second_col_zero_matrix1 = MatrixXd::Zero(mInEquNum, mVariableNum);
      MatrixXd second_col_zero_matrix2 = MatrixXd::Zero(mInEquNum, mEquNum);
      hess_second_row << second_col_zero_matrix1, sigma_diag, second_col_zero_matrix2, eye_constrain_matrix;

      MatrixXd hess_concate_row_first_second =
          MatrixXd::Zero(mVariableNum + mInEquNum, mVariableNum + 2 * mInEquNum + mEquNum);
      hess_concate_row_first_second << hess_first_row, hess_second_row;

      MatrixXd hess_third_row_zero =
          MatrixXd::Zero(mInEquNum + mEquNum, mVariableNum + 2 * mInEquNum + mEquNum);

      MatrixXd hess(mVariableNum + 2 * mInEquNum + mEquNum,
                    mVariableNum + 2 * mInEquNum + mEquNum);
      hess << hess_concate_row_first_second, hess_third_row_zero;

      MatrixXd hess_upper = hess.triangularView<Eigen::Upper>();
      hess = hess_upper + hess_upper.transpose();

      MatrixXd hess_half_diag =
          (hess.diagonal() / 2).array().matrix().asDiagonal();
      hess -= hess_half_diag;

      // KIRI_LOG_DEBUG("before reg hess=\n{0}", hess);

      // // reg
      auto w = hess.eigenvalues();
      auto rcond = w.array().abs().minCoeff() / w.array().abs().maxCoeff();

      auto matrix_inertia = this->computeMatrixInertia(w);

      if (rcond <= MEpsilon<float>() || (mInEquNum + mEquNum) != matrix_inertia)
      {

        if (rcond <= MEpsilon<float>() && mEquNum > 0)
        {
          auto ind1 = mVariableNum + mInEquNum;
          auto ind2 = ind1 + mEquNum;
          hess.block(ind1, ind1, ind2 - ind1, ind2 - ind1) -=
              sqrt((double)MEpsilon<float>()) * mEta * std::pow(mMu, mBeta) * MatrixXd::Identity(mEquNum, mEquNum);
        }

        if (mDelta == 0.0)
        {
          mDelta = sqrt(MEpsilon<float>());
        }
        else
        {
          // prevent the diagonal shift coefficient from becoming too small

          mDelta = std::max(mDelta / 2, sqrt((double)MEpsilon<float>()));
        }
        hess.block(0, 0, mData.size(), mData.size()) +=
            mDelta * MatrixXd::Identity(mData.size(), mData.size());
        w = hess.eigenvalues();
        matrix_inertia = this->computeMatrixInertia(w);
        auto counter = 0;
        auto max_loop = 100;
        while ((mInEquNum + mEquNum) != matrix_inertia && counter++ < max_loop)
        {
          hess.block(0, 0, mData.size(), mData.size()) -=
              mDelta * MatrixXd::Identity(mData.size(), mData.size());
          mDelta *= 10;
          hess.block(0, 0, mData.size(), mData.size()) +=
              mDelta * MatrixXd::Identity(mData.size(), mData.size());
          w = hess.eigenvalues();
          matrix_inertia = this->computeMatrixInertia(w);
        }
      }

      mHessian = hess;
      // KIRI_LOG_DEBUG("hess=\n{0}", hess);
    }

    void computeKKT()
    {
      this->computeHessian();
      this->computeConstrainstsJacobian();
      this->computeGrad();

      // KIRI_LOG_DEBUG("mLambda=\n{0}", mLambda);
      mKKT1 = mGrad(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all);
      mKKT2 = MatrixXd::Zero(1, 1);
      mKKT3 = MatrixXd::Zero(1, 1);
      mKKT4 = MatrixXd::Zero(1, 1);

      if (mEquNum > 0)
      {
        mKKT3 = mGrad(Eigen::seq(mVariableNum + mInEquNum, mVariableNum + mInEquNum + mEquNum - 1), Eigen::placeholders::all);
      }

      if (mInEquNum > 0)
      {
        auto grad2 = mGrad(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1), Eigen::placeholders::all);
        auto kkt2 = MatrixXd(grad2.rows(), grad2.cols());
        for (auto i = 0; i < grad2.rows(); i++)
        {
          for (auto j = 0; j < grad2.cols(); j++)
          {
            kkt2(i, j) =
                grad2(i, j) *
                mSlack(i, j);
          }
        }
        mKKT2 = kkt2;
        mKKT4 = mGrad(Eigen::seq(mVariableNum + mInEquNum + mEquNum, mGrad.rows() - 1), Eigen::placeholders::all);
      }

      // KIRI_LOG_DEBUG("KKT1=\n{0}", mKKT1.transpose());
      // KIRI_LOG_DEBUG("KKT2=\n{0}", mKKT2.transpose());
      // KIRI_LOG_DEBUG("KKT3=\n{0}", mKKT3.transpose());
      // KIRI_LOG_DEBUG("KKT4=\n{0}", mKKT4.transpose());
    }

    void computeBackTrackingLineSearch(MatrixXd dz, double alphaSMax = 1.0,
                                       double alphaLMax = 1.0)
    {
      auto correction = false;
      auto alpha_corr = 1.0;
      MatrixXd dzp = MatrixXd::Zero(mVariableNum + mInEquNum, 1);

      // mPhi0 = targetFunc(mData);

      auto alpha_smax = computeMaximumStepSize(
          mSlack.array(),
          dz(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1),
             Eigen::placeholders::all)
              .array());

      // KIRI_LOG_DEBUG("alpha_smax={0}", alpha_smax);

      // KIRI_LOG_DEBUG("mLambda={0}", mLambda);
      auto alpha_lmax = computeMaximumStepSize(
          mLambda(Eigen::seq(mEquNum, mLambda.size() - 1),
                  Eigen::placeholders::all)
              .array(),
          dz(Eigen::seq(mVariableNum + mInEquNum + mEquNum, dz.size() - 1),
             Eigen::placeholders::all)
              .array());

      // KIRI_LOG_DEBUG("alpha_lmax={0}", alpha_lmax);

      MatrixXd dx = dz(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all);
      MatrixXd ds = dz(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1),
                       Eigen::placeholders::all);
      MatrixXd dl = dz(Eigen::seq(mVariableNum + mInEquNum, dz.size() - 1),
                       Eigen::placeholders::all);
      // KIRI_LOG_DEBUG("ds={0}", ds);
      // KIRI_LOG_DEBUG("dl={0}", dl);

      auto ci = this->computeInEquConstrainsts(mRealData);
      auto ce = this->computeEquConstrainsts(mRealData);

      // compute merit function
      auto phi_nu = eval(TargetFunc(mDual2ndData));

      if (mEquNum > 0)
      {
        phi_nu += mNu * abs(ce.array()).sum();
      }

      if (mInEquNum > 0)
      {
        phi_nu -= mMu * log(mSlack.array()).sum();
        phi_nu += mNu * abs(ci.array() - mSlack.array()).sum();
      }

      // KIRI_LOG_DEBUG("phi_nu={0}", phi_nu);

      MatrixXd func_grad = mFuncGrad.cast<double>();
      auto dphi_nu = (func_grad.transpose() * dz(Eigen::seq(0, mVariableNum - 1),
                                                 Eigen::placeholders::all))(0, 0);

      if (mEquNum > 0)
      {
        dphi_nu -= mNu * abs(ce.array()).sum();
      }

      if (mInEquNum > 0)
      {
        dphi_nu -= mNu * abs(ci.array() - mSlack.array()).sum();
        MatrixXd tmp = mSlack;
        for (auto i = 0; i < tmp.rows(); i++)
        {
          for (auto j = 0; j < tmp.cols(); j++)
          {
            tmp(i, j) = mMu / (mSlack(i, j) + MEpsilon<float>());
          }
        }

        dphi_nu -= (tmp.transpose() *
                    dz(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1),
                       Eigen::placeholders::all))(0, 0);
      }

      // KIRI_LOG_DEBUG("dphi_nu={0}", dphi_nu);

      auto ci1 = this->computeInEquConstrainsts(mRealData + alpha_smax * dx);
      auto ce1 = this->computeEquConstrainsts(mRealData + alpha_smax * dx);

      double phi_nu1 =
          static_cast<double>(TargetFunc(mDual2ndData + alpha_smax * dx));

      if (mEquNum > 0)
      {
        phi_nu1 += mNu * abs(ce1.array()).sum();
      }

      if (mInEquNum > 0)
      {
        phi_nu1 -= mMu * log((mSlack + alpha_smax * ds).array()).sum();
        phi_nu1 += mNu * abs(ci1.array() - (mSlack + alpha_smax * ds).array()).sum();
      }

      // KIRI_LOG_DEBUG("phi_nu1={0}", phi_nu1);

      double phi1_next =
          static_cast<double>(phi_nu) + alpha_smax * mEta * dphi_nu;
      // KIRI_LOG_DEBUG("phi1_next={0}", phi1_next);

      if (phi_nu1 > phi1_next)
      {
        // KIRI_LOG_DEBUG("phi_nu1 > phi1_next");

        MatrixXd con_old = this->computeConstainMatrix(mRealData, mSlack);
        MatrixXd con_new = this->computeConstainMatrix(mRealData + alpha_smax * dx, mSlack + alpha_smax * ds);
        // KIRI_LOG_DEBUG("con_old={0}", con_old);
        // KIRI_LOG_DEBUG("con_new={0}", con_new);

        auto con_old_sum = con_old.array().abs().sum();
        auto con_new_sum = con_new.array().abs().sum();

        // KIRI_LOG_DEBUG("con_old_sum={0}, con_new_sum={1}", con_old_sum,
        // con_new_sum);
        if (con_new_sum > con_old_sum)
        {
          // KIRI_LOG_INFO("con_new_sum > con_old_sum");
          auto A = mJaco.transpose();

          // KIRI_LOG_DEBUG("A={0}", A);
          //  dzp = -(A.inverse() * con_new);
          VectorXd b = con_new.array();
          dzp = -A.colPivHouseholderQr().solve(b);

          // std::cout << get_shape(dzp) << std::endl;
          // std::cout << get_shape(b) << std::endl;

          auto ci2 = this->computeInEquConstrainsts(
              mRealData + alpha_smax * dx +
              dzp(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all));
          auto ce2 = this->computeEquConstrainsts(
              mRealData + alpha_smax * dx +
              dzp(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all));

          double phi_nu2 = static_cast<double>(TargetFunc(
              mDual2ndData + alpha_smax * dx +
              dzp(Eigen::seq(0, mVariableNum - 1), Eigen::placeholders::all)));

          if (mEquNum > 0)
          {
            phi_nu2 += mNu * abs(ce2.array()).sum();
          }

          if (mInEquNum > 0)
          {
            phi_nu2 -=
                mMu *
                log((mSlack + alpha_smax * ds +
                     dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1),
                         Eigen::placeholders::all))
                        .array())
                    .sum();
            phi_nu2 +=
                mNu *
                abs(ci2.array() -
                    (mSlack + alpha_smax * ds +
                     dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1),
                         Eigen::placeholders::all))
                        .array())
                    .sum();
          }

          double phi2_next =
              static_cast<double>(phi_nu) + alpha_smax * mEta * dphi_nu;

          if (phi_nu2 <= phi2_next)
          {
            alpha_corr = computeMaximumStepSize(
                mSlack.array(),
                (alpha_smax * ds +
                 dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1),
                     Eigen::placeholders::all))
                    .array());

            ci2 = this->computeInEquConstrainsts(
                mRealData + alpha_corr * (alpha_smax * dx +
                                          dzp(Eigen::seq(0, mVariableNum - 1),
                                              Eigen::placeholders::all)));

            ce2 = this->computeEquConstrainsts(
                mRealData + alpha_corr * (alpha_smax * dx +
                                          dzp(Eigen::seq(0, mVariableNum - 1),
                                              Eigen::placeholders::all)));

            phi_nu2 = static_cast<double>(TargetFunc(
                mDual2ndData + alpha_corr * (alpha_smax * dx +
                                             dzp(Eigen::seq(0, mVariableNum - 1),
                                                 Eigen::placeholders::all))));

            if (mEquNum > 0)
            {
              phi_nu2 += mNu * abs(ce2.array()).sum();
            }

            if (mInEquNum > 0)
            {
              phi_nu2 -=
                  mMu *
                  log((mSlack +
                       alpha_corr * (alpha_smax * ds +
                                     dzp(Eigen::seq(mVariableNum,
                                                    mVariableNum + mInEquNum - 1),
                                         Eigen::placeholders::all)))
                          .array())
                      .sum();
              phi_nu2 +=
                  mNu *
                  abs(ci2.array() -
                      (mSlack +
                       alpha_corr * (alpha_smax * ds +
                                     dzp(Eigen::seq(mVariableNum,
                                                    mVariableNum + mInEquNum - 1),
                                         Eigen::placeholders::all)))
                          .array())
                      .sum();
            }

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

          ci1 = this->computeInEquConstrainsts(mRealData + alpha_smax * dx);
          ce1 = this->computeEquConstrainsts(mRealData + alpha_smax * dx);

          phi_nu1 =
              static_cast<double>(TargetFunc(mDual2ndData + alpha_smax * dx));

          if (mEquNum > 0)
          {
            phi_nu1 += mNu * abs(ce1.array()).sum();
          }

          if (mInEquNum > 0)
          {

            phi_nu1 -= mMu * log((mSlack + alpha_smax * ds).array()).sum();
            phi_nu1 +=
                mNu * abs(ci1.array() - (mSlack + alpha_smax * ds).array()).sum();
          }

          phi1_next = static_cast<double>(phi_nu) + alpha_smax * mEta * dphi_nu;

          while (phi_nu1 > phi1_next)
          {
            if (sqrt((alpha_smax * dx).norm() * (alpha_smax * dx).norm() +
                     (alpha_lmax * ds).norm() * (alpha_lmax * ds).norm()) <
                MEpsilon<float>())
            {
              mSignal = -2;
              KIRI_LOG_INFO(" search direction is unreliable to machine "
                            "precision, stop solver");
              return;
            }

            alpha_smax *= mTau;
            alpha_lmax *= mTau;

            // update
            ci1 = this->computeInEquConstrainsts(mRealData + alpha_smax * dx);
            ce1 = this->computeEquConstrainsts(mRealData + alpha_smax * dx);

            phi_nu1 =
                static_cast<double>(TargetFunc(mDual2ndData + alpha_smax * dx));

            if (mEquNum > 0)
            {
              phi_nu1 += mNu * abs(ce1.array()).sum();
            }

            if (mInEquNum > 0)
            {

              phi_nu1 -= mMu * log((mSlack + alpha_smax * ds).array()).sum();
              phi_nu1 +=
                  mNu * abs(ci1.array() - (mSlack + alpha_smax * ds).array()).sum();
            }

            phi1_next = static_cast<double>(phi_nu) + alpha_smax * mEta * dphi_nu;
          }

          // KIRI_LOG_DEBUG("alpha_smax={0} ; alpha_lmax={1}", alpha_smax,
          // alpha_lmax);
        }
      }

      // update slack values
      if (correction)
      {
        mSlack += alpha_corr *
                  (alpha_smax * ds +
                   dzp(Eigen::seq(mVariableNum, mVariableNum + mInEquNum - 1),
                       Eigen::placeholders::all));
      }
      else
      {
        mSlack += alpha_smax * ds;
      }

      // update weights
      if (correction)
      {
        mRealData +=
            alpha_corr * (alpha_smax * dx + dzp(Eigen::seq(0, mVariableNum - 1),
                                                Eigen::placeholders::all));
        mDual2ndData +=
            alpha_corr * (alpha_smax * dx + dzp(Eigen::seq(0, mVariableNum - 1),
                                                Eigen::placeholders::all));
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

  typedef std::shared_ptr<PrimalDualIPM> PrimalDualIPMPtr;

} // namespace OPTIMIZE::IPM

#endif /* _PRIMAL_DUAL_IPM_H_ */