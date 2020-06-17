#include "robot_control/controllers/end_effector_controllers/task_space_controller.h"

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>


using namespace Eigen;
namespace pin = pinocchio;

namespace rc {

TaskSpaceController::TaskSpaceController(RobotWrapper* wrp, std::string& controlled_frame) : controlled_frame(controlled_frame),
  solver(6, wrp->getDof()),
  J(6, wrp->getDof()),
  dJ(6, wrp->getDof())
{
  wrapper = wrp;
  kp_ = MatrixXd::Identity(6, 6) * 0.0;
  kd_ = MatrixXd::Identity(6, 6) * 10.0;
  int dof = wrapper->getDof();
  kqd_ns = 0.1 * MatrixXd::Identity(dof, dof);
  kqp_res = 0.1 * MatrixXd::Identity(dof, dof);
  q_rest = wrapper->getNeutralConfiguration();
  target = pin::SE3(Matrix<double, 3, 3>::Identity(), Matrix<double, 3, 1>::Zero());
}

void TaskSpaceController::setTaskTarget(pin::SE3& task_target) {
  target = task_target;
  target_set = true;
}

void TaskSpaceController::setKp(const Matrix<double, 6, 1>& kp){ kp_ = kp.asDiagonal(); }

void TaskSpaceController::setKd(const Matrix<double, 6, 1>& kd){ kd_ = kd.asDiagonal(); }

VectorXd TaskSpaceController::computeCommand() {
  pin::SE3 desired_pose;
  pin::SE3 current_pose = wrapper->getFramePlacement(controlled_frame).normalized();
  if (!target_set) {
    desired_pose = current_pose;
  } else {
    desired_pose = target;
  }

  Quaternion<double> dR(current_pose.rotation().transpose() * desired_pose.rotation());
  dR = dR.normalized();
  Matrix<double, 6, 1> position_error;
  position_error.head<3>() = current_pose.rotation().transpose() * (desired_pose.translation() - current_pose.translation());
  position_error[3] = 2.0 * dR.w() * dR.x();
  position_error[4] = 2.0 * dR.w() * dR.y();
  position_error[5] = 2.0 * dR.w() * dR.z();

  pin::Motion velocity = pin::log6(current_pose.actInv(desired_pose));
  Matrix<double, 6, 1> desired_velocity;
  desired_velocity.head<3>() = velocity.linear();
  desired_velocity.tail<3>() = velocity.angular();

  pin::Motion frame_velocity = wrapper->getFrameVelocity(controlled_frame);
  Matrix<double, 6, 1> current_velocity;
  current_velocity.head<3>() = frame_velocity.linear();
  current_velocity.tail<3>() = frame_velocity.angular();
  Matrix<double, 6, 1> velocity_error = desired_velocity - current_velocity;

  wrapper->getAllFrameJacobians(controlled_frame, J, dJ);
  wrapper->computeAllTerms();

  // task space dynamics
  VectorXd error = kp_ * position_error + kd_ * velocity_error - dJ * wrapper->v;
  MatrixXd Jpinv = linear_algebra::computePInvDLS(solver, J);
  VectorXd y = Jpinv * error;

  // null-space control
  VectorXd epsilon = -(kqd_ns * wrapper->v) - kqp_res * (wrapper->q - q_rest);
  VectorXd tau_null_space = linear_algebra::computeNullSpace(J) * epsilon;
  return wrapper->getInertia() * y  + wrapper->getNonLinearTerms();
}

VectorXd TaskSpaceController::advance(VectorXd& q, VectorXd v) {
  wrapper->updateState(q, v, true);
  return computeCommand();
}

}
