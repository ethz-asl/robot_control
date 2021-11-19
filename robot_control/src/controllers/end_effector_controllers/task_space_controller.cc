#include "robot_control/controllers/end_effector_controllers/task_space_controller.h"

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace pin = pinocchio;

namespace rc {

TaskSpaceController::TaskSpaceController(std::shared_ptr<RobotWrapper> wrp, std::string& controlled_frame) : controlled_frame(controlled_frame),
  svd(6, wrp->getDof()),
  J(6, wrp->getDof()),
  dJ(6, wrp->getDof()),
  solver(svd)
{
  wrapper = wrp;
  kp_ = Eigen::MatrixXd::Identity(6, 6) * 10.0;
  kd_ = Eigen::MatrixXd::Identity(6, 6) * 5.0;
  int dof = wrapper->getDof();
  kqd_ns = 0.1 * Eigen::MatrixXd::Identity(dof, dof);
  kqp_res = 0.1 * Eigen::MatrixXd::Identity(dof, dof);
  q_rest = wrapper->getNeutralConfiguration();
  target = pin::SE3(Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 1>::Zero());
}

void TaskSpaceController::setTaskTarget(pin::SE3 task_target) {
  std::lock_guard<std::mutex> lock(target_mutex);
  target = task_target;
  target_set = true;
}

void TaskSpaceController::setKp(const Eigen::Matrix<double, 6, 1>& kp){ kp_ = kp.asDiagonal(); }

void TaskSpaceController::setKd(const Eigen::Matrix<double, 6, 1>& kd){ kd_ = kd.asDiagonal(); }

Eigen::VectorXd TaskSpaceController::computeCommand() {
  std::lock_guard<std::mutex> lock(target_mutex);
  pin::SE3& current_pose = wrapper->getFramePlacement(controlled_frame);
  pin::SE3 desired_pose;
  if (target_set) {
    desired_pose = target;
  } else {
    desired_pose = current_pose;
  }

  Eigen::Quaternion<double> dR(current_pose.rotation().transpose() * desired_pose.rotation());
  dR = dR.normalized();
  Eigen::Matrix<double, 6, 1> position_error;
  position_error.head<3>() = current_pose.rotation().transpose() * (desired_pose.translation() - current_pose.translation());
  position_error[3] = 2.0 * dR.w() * dR.x();
  position_error[4] = 2.0 * dR.w() * dR.y();
  position_error[5] = 2.0 * dR.w() * dR.z();

  pin::Motion velocity = pin::log6(current_pose.actInv(desired_pose));
  Eigen::Matrix<double, 6, 1> desired_velocity;
  desired_velocity.head<3>() = velocity.linear();
  desired_velocity.tail<3>() = velocity.angular();

  pin::Motion frame_velocity = wrapper->getFrameVelocity(controlled_frame);
  Eigen::Matrix<double, 6, 1> current_velocity;
  current_velocity.head<3>() = frame_velocity.linear();
  current_velocity.tail<3>() = frame_velocity.angular();
  Eigen::Matrix<double, 6, 1> velocity_error = desired_velocity - current_velocity;

  wrapper->getAllFrameJacobians(controlled_frame, J, dJ);
  wrapper->computeAllTerms();

  // task space dynamics
  Eigen::VectorXd error = kp_ * position_error + kd_ * velocity_error - dJ * wrapper->v;
  solver.compute(J);
  Eigen::VectorXd y = solver.solve(error);
  return wrapper->getInertia() * y  + wrapper->getNonLinearTerms();
}

Eigen::VectorXd TaskSpaceController::advance(Eigen::VectorXd& q, Eigen::VectorXd& v) {
  wrapper->updateState(q, v, true);
  return computeCommand();
}

}
