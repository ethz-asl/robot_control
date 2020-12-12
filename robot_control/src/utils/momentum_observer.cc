/*!
 * @file     momentum_observer.h
 * @author   Giuseppe Rizzi
 * @date     04.12.2020
 * @version  1.0
 * @brief    description
 */
#include <robot_control/utils/momentum_observer.h>
#include <Eigen/Core>

using namespace Eigen;

namespace rc {

MomentumObserver::MomentumObserver(const std::string& robot_description, const VectorXd& gain,
                                   double alpha) {
  alpha_ = alpha;
  model_.initFromXml(robot_description);
  dof_ = model_.getDof();
  gain_ = gain;
  assert(gain_.size() == dof_);
}

void MomentumObserver::reset(const VectorXd& q, const VectorXd& qd) {
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

void MomentumObserver::get_wrench(Matrix<double, 6, 1>& wrench) { wrench = extWrench_; }

void MomentumObserver::estimate_wrench(const VectorXd& q, const VectorXd& qd, const VectorXd& tau,
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
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd singular_inv(svd.singularValues());
  for (int j = 0; j < singular_inv.size(); ++j)
    singular_inv(j) = (singular_inv(j) < 1e-6) ? 0.0 : 1.0 / singular_inv(j);
  Jt_inv_.noalias() = svd.matrixV() * singular_inv.matrix().asDiagonal() * svd.matrixU().adjoint();

  // Compute End-Effector Cartesian forces from joint external torques
  extWrench_ = Jt_inv_ * tauExtFiltered_;
}

}  // namespace rc
