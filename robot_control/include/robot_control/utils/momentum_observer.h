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
  MomentumObserver(const std::string& robot_description, const VectorXd& gain, double alpha = 0.9) {
    alpha_ = alpha;
    model_.initFromXml(robot_description);
    dof_ = model_.getDof();
    gain_ = gain;
    assert(gain_.size() == dof_);
  };
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

  void reset(const VectorXd& q, const VectorXd& qd) {
    model_.updateState(q, qd, true);
    model_.computeAllTerms();
    M_ = model_.getInertia();
    Mprev_ = M_;
    Md_.setZero(dof_, dof_);

    tauExt_.setZero(dof_);
    momentumIntegral_.setZero(dof_);
    momentumInitial_ = M_ * qd;
    tauExtFiltered_.setZero(dof_);
    extWrench_.setZero(6);
    last_time_ = getTimeSec();
    std::cout << "[MomentumObserver::reset] Estimator reset." << std::endl;
  }

  void get_wrench(Matrix<double, 6, 1>& wrench){
    wrench = extWrench_;
  }

  void estimate_wrench(const VectorXd& q, const VectorXd& qd, const VectorXd& tau,
                       std::string& frame) {
    model_.updateState(q, qd, true);
    model_.computeAllTerms();

    M_ = model_.getInertia();
    double dt = getTimeSec() - last_time_;
    if (dt >= 0) Md_ = (M_ - Mprev_) / dt;
    Mprev_ = M_;
    last_time_ = getTimeSec();

    tauTot_ = tau - model_.getNonLinearTerms() + Md_ * qd;

    momentumIntegral_ += (tauTot_ + tauExt_) * dt;
    momentumModel_ = M_ * qd;

    tauExt_ = gain_.asDiagonal() * (momentumModel_ - momentumIntegral_ - momentumInitial_);
    tauExtFiltered_ = alpha_ * tauExtFiltered_ + (1.0 - alpha_) * tauExt_;

    model_.getLocalFrameJacobian(frame, J_);

    // Compute SVD of the jacobian using Eigen functions
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_.transpose(),
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singular_inv(svd.singularValues());
    for (int j = 0; j < singular_inv.size(); ++j)
      singular_inv(j) = (singular_inv(j) < 1e-6) ? 0.0 : 1.0 / singular_inv(j);
    Jt_inv_.noalias() =
        svd.matrixV() * singular_inv.matrix().asDiagonal() * svd.matrixU().adjoint();

    // Compute End-Effector Cartesian forces from joint external torques
    extWrench_ = Jt_inv_ * tauExtFiltered_;
  }
};
}  // namespace rc
