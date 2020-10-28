/*!
 * @file     ik_controller.h
 * @author   Giuseppe Rizzi
 * @date     28.10.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <mutex>
#include <pinocchio/fwd.hpp>
#include "robot_control/math/math.h"
#include "robot_control/modeling/robot_wrapper.h"

using namespace Eigen;
namespace pin = pinocchio;

namespace rc {

struct IKSolverParams{
  int it_max = 1000;
  double damping = 1e-6;
  double eps = 1e-4;
  double step_size = 0.1;
};

class IKController {
 public:
  IKController(std::shared_ptr<RobotWrapper> wrp, std::string& controlled_frame);

  /// Setters
  void setKp(const VectorXd& kp);
  void setKd(const VectorXd& kd);
  void setIKMaxIterations(const int it);
  void setIKDamping(const double it);
  void setIKTolerance(const double eps);
  void setIKStepSize(const double step_size);
  void setTaskTarget(const pin::SE3& task_target);

  bool computeCommand(VectorXd& cmd);
  bool advance(const VectorXd& q, const VectorXd& v, VectorXd& cmd);

 private:
  bool computeJointTarget(const pinocchio::SE3& task_target, Eigen::VectorXd& qdes, const Eigen::VectorXd& q0);

 private:
  std::string controlled_frame;
  std::shared_ptr<RobotWrapper> wrapper;
  std::shared_ptr<RobotWrapper> wrapper_ik;

  Eigen::VectorXd q_desired;
  Eigen::VectorXd qd_desired;

  pin::SE3 target;
  std::mutex target_mutex;
  bool target_set = false;

  MatrixXd kp_, kd_, q_rest;
  IKSolverParams ik_params;

};
}