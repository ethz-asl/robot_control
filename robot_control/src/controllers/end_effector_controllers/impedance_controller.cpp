/*!
 * @file     impedance_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     04.12.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control/controllers/end_effector_controllers/impedance_controller.h"
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

using namespace Eigen;
namespace pin = pinocchio;

namespace rc {

ImpedanceController::ImpedanceController(std::shared_ptr<RobotWrapper> wrp,
                                         std::string &controlled_frame)
    : controlled_frame(controlled_frame), J(6, wrp->getDof()) {
  wrapper = wrp;
  kp_ = MatrixXd::Identity(6, 6) * 1.0;
  kd_ = MatrixXd::Identity(6, 6) * 1.0;
  target = pin::SE3(Matrix<double, 3, 3>::Identity(), Matrix<double, 3, 1>::Zero());
  std::cout << "Manipulator has " << wrapper->getDof() << " dofs." << std::endl;
  std::cout << "Controlling frame: " << controlled_frame << std::endl;
}

void ImpedanceController::setTaskTarget(pin::SE3 task_target) {
  std::lock_guard<std::mutex> lock(target_mutex);
  target = task_target;
  target_set = true;
}

void ImpedanceController::setKp(const Matrix<double, 6, 1> &kp) { kp_ = kp.asDiagonal(); }

void ImpedanceController::setKd(const Matrix<double, 6, 1> &kd) { kd_ = kd.asDiagonal(); }

VectorXd ImpedanceController::computeCommand() {
  std::lock_guard<std::mutex> lock(target_mutex);
  pin::SE3 &current_pose = wrapper->getFramePlacement(controlled_frame);
  pin::SE3 desired_pose;
  if (target_set) {
    desired_pose = target;
  } else {
    desired_pose = current_pose;
  }

  Matrix<double, 6, 1> error;
  error.head(3) << current_pose.translation() - desired_pose.translation();

  // orientation error
  Quaterniond orientation_d(desired_pose.rotation());
  Quaterniond orientation(current_pose.rotation());
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -current_pose.rotation() * error.tail(3);

  Matrix<double, 6, 1> desired_velocity;
  desired_velocity.setZero();

  pin::Motion frame_velocity = wrapper->getFrameVelocity(controlled_frame);
  Matrix<double, 6, 1> current_velocity;
  current_velocity.head<3>() = frame_velocity.linear();
  current_velocity.tail<3>() = frame_velocity.angular();
  Matrix<double, 6, 1> velocity_error = current_velocity - desired_velocity;

  wrapper->getFrameJacobian(controlled_frame, J);
  wrapper->computeAllTerms();

  size_t dof = wrapper->getDof();
  VectorXd tau_task(dof), tau_nullspace(dof), tau_d(dof);

  // Cartesian PD control with damping ratio = 1
  tau_task << J.transpose() * (-kp_ * error - kd_ * velocity_error);

  // kinematic pseuoinverse for nullspace handling
  Eigen::MatrixXd Jt_pinv;
  pseudoInverse(J.transpose(), Jt_pinv);
  VectorXd q = wrapper->getQ();
  VectorXd qd = wrapper->getV();
  tau_nullspace << (MatrixXd::Identity(dof, dof) - J.transpose() * Jt_pinv) *
                       (nullspace_stiffness_ * (q_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * qd);

  // Desired torque
  //tau_nullspace.setZero(dof);
  tau_d << tau_task + tau_nullspace + wrapper->getNonLinearTerms();
  return tau_d;
}

VectorXd ImpedanceController::advance(VectorXd &q, VectorXd &v) {
  wrapper->updateState(q, v, true);
  return computeCommand();
}

void ImpedanceController::pseudoInverse(const MatrixXd &m, MatrixXd &m_pinv, bool damped) {
  double lambda_ = damped ? 0.002 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = m;  // copying the dimensions of m, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  m_pinv = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

}  // namespace rc
