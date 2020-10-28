/*!
 * @file     ik_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     28.10.2020
 * @version  1.0
 * @brief    description
 */
#include "robot_control/controllers/end_effector_controllers/ik_controller.h"

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>



using namespace Eigen;
namespace pin = pinocchio;

namespace rc {

IKController::IKController(std::shared_ptr<RobotWrapper> wrp,
                           std::string& controlled_frame)
{
  wrapper = wrp;
  wrapper_ik = std::make_shared<RobotWrapper>(*wrp);   // copy wrapper for internal IK
  int dof = wrapper->getDof();
  kp_ = MatrixXd::Identity(dof, dof) * 10.0;
  kd_ = MatrixXd::Identity(dof, dof) * 0.1;
  q_rest = wrapper->getNeutralConfiguration();
  qd_desired.setZero(dof);
}

void IKController::setIKMaxIterations(const int it_max){
  assert(it_max>0);
  ik_params.it_max = it_max;
}

void IKController::setIKDamping(const double damping){
  assert(damping>0);
  ik_params.damping = damping;
}

void IKController::setIKTolerance(const double eps){
  assert(eps>0);
  ik_params.eps = eps;
}

void IKController::setIKStepSize(const double step_size){
  assert(step_size>0);
  ik_params.step_size = step_size;
}

bool IKController::computeJointTarget(const pinocchio::SE3& task_target, Eigen::VectorXd& qdes, const Eigen::VectorXd& q0){
  bool success = false;
  static Eigen::VectorXd err;
  static pinocchio::Data::Matrix6 J, JJt;
  Eigen::VectorXd err_sol;
  J.setZero();
  static Eigen::VectorXd v(wrapper_ik->getDof());
  v.setZero(wrapper_ik->getDof());
  qdes = q0;
  for (int i=0;;i++)
  {
    wrapper_ik->updateState(qdes, v, true);
    const pinocchio::SE3 dMi = task_target.actInv(wrapper_ik->getFramePlacement(controlled_frame));
    err = pinocchio::log6(dMi).toVector();
    if(err.norm() < ik_params.eps)
    {
      success = true;
      std::cout << "Ik succeeded" << std::endl;
      break;
    }
    if (i >= ik_params.it_max)
    {
      success = false;
      std::cout << "Ik failed" << std::endl;
      break;
    }
    std::cout << "Get frame jacobian" << std::endl;
    J = wrapper_ik->getFrameJacobian(controlled_frame);
    std::cout << "JJt" << std::endl;
    JJt.noalias() = J * J.transpose();
    std::cout << "adding damping" << std::endl;
    JJt.diagonal().array() += ik_params.damping;
    std::cout << "Solving for velocities" << std::endl;
    std::cout << "V has size: " << v.size() << std::endl;
    err_sol = JJt.ldlt().solve(err);
    std::cout << "err sol: " << err_sol << std::endl;
    std::cout << "J t: " << J.transpose() << std::endl;

    v.noalias() = - J.transpose() * err_sol;
    std::cout << "V has size: " << v.size() << std::endl;

    wrapper_ik->integrateQ(qdes, v, ik_params.step_size);
  }
};

void IKController::setTaskTarget(const pin::SE3& task_target) {
  std::lock_guard<std::mutex> lock(target_mutex);
  target = task_target;
  target_set = true;
}
void IKController::setKp(const VectorXd& kp){ kp_ = kp.asDiagonal(); }

void IKController::setKd(const VectorXd& kd){ kd_ = kd.asDiagonal(); }

bool IKController::computeCommand(Eigen::VectorXd& cmd) {
  std::lock_guard<std::mutex> lock(target_mutex);
  pin::SE3& current_pose = wrapper->getFramePlacement(controlled_frame);
  pin::SE3 desired_pose;
  if (target_set) {
    desired_pose = target;
    std::cout << "Set new target to: " << desired_pose << std::endl;
  } else {
    desired_pose = current_pose;
  }

  wrapper->computeAllTerms();

  // keep current pose on failure
  std::cout << "Computing IK" << std::endl;
  if (!computeJointTarget(desired_pose, q_desired, wrapper->getQ())){
    cmd = wrapper->getNonLinearTerms();
    return false;
  }

  cmd = wrapper->getNonLinearTerms() + kp_ * (q_desired-wrapper->getQ()) + kd_ * (qd_desired-wrapper->getV());
  return true;
}

bool IKController::advance(const VectorXd& q, const VectorXd& v, VectorXd& cmd) {
  wrapper->updateState(q, v, true);
  wrapper_ik->updateState(q, v, true);
  return computeCommand(cmd);
}

}