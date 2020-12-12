/*!
 * @file     momentum_observer.h
 * @author   Giuseppe Rizzi
 * @date     04.12.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <robot_control/modeling/robot_wrapper.h>
#include <Eigen/Core>
#include <chrono>

using namespace Eigen;
/**
 * Implementation of a simple momentum observer. Details can be found
 * in the baseline of the following paper
 * @brief https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8793529
 */
namespace rc {

double getTimeSec() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
             .count() /
         1e6;
}

class MomentumObserver {
 public:
  MomentumObserver(const std::string& robot_description, 
                   const VectorXd& gain, 
                   double alpha = 0.9);
  ~MomentumObserver() = default;

 private:
  size_t dof_;
  RobotWrapper model_;

  double alpha_;
  VectorXd gain_;

  VectorXd tauTot_;
  VectorXd tauExt_;
  VectorXd tauExtFiltered_;

  VectorXd momentumIntegral_;
  VectorXd momentumModel_;
  VectorXd momentumInitial_;

  VectorXd extWrench_;

  MatrixXd M_;
  MatrixXd Mprev_;
  MatrixXd Md_;
  MatrixXd J_;
  MatrixXd Jt_inv_;
  double last_time_;

 public:
  inline size_t get_dof() const { return dof_; }

  void reset(const VectorXd& q, const VectorXd& qd);
  void get_wrench(Matrix<double, 6, 1>& wrench);
  void estimate_wrench(const VectorXd& q, const VectorXd& qd, const VectorXd& tau,
                       std::string& frame);
};
}  // namespace rc
