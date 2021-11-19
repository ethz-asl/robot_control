#pragma once
#include <mutex>
#include <pinocchio/fwd.hpp>
#include "robot_control/math/math.h"
#include "robot_control/modeling/robot_wrapper.h"

namespace pin = pinocchio;

namespace rc {
class TaskSpaceController {
  private:
  std::shared_ptr<RobotWrapper> wrapper;
  bool target_set = false;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  linear_algebra::DLSSolver solver;
  Eigen::MatrixXd J;
  Eigen::MatrixXd dJ;
  std::mutex target_mutex;

  public:
  pin::SE3 target;
  std::string controlled_frame;
  Eigen::MatrixXd kp_, kd_, kqd_ns, kqp_res, q_rest;

  TaskSpaceController(std::shared_ptr<RobotWrapper> wrp, std::string& controlled_frame);

  void setTaskTarget(pin::SE3 task_target);

  void setKp(const Eigen::Matrix<double, 6, 1>& kp);

  void setKd(const Eigen::Matrix<double, 6, 1>& kd);

  Eigen::VectorXd computeCommand();

  Eigen::VectorXd advance(Eigen::VectorXd& q, Eigen::VectorXd& v);

};
}
