#pragma once
#include <pinocchio/fwd.hpp>
#include "robot_control/math/math.h"
#include "robot_control/modeling/robot_wrapper.h"

using namespace Eigen;
namespace pin = pinocchio;

namespace rc {
class TaskSpaceController {
  private:
  RobotWrapper* wrapper;
  pin::SE3 target;
  bool target_set = false;
  JacobiSVD<MatrixXd> solver;
  MatrixXd J;
  MatrixXd dJ;

  public:
  std::string controlled_frame;
  MatrixXd kp_, kd_, kqd_ns, kqp_res, q_rest;

  TaskSpaceController(RobotWrapper* wrp, std::string& controlled_frame);

  void setTaskTarget(pin::SE3& task_target);

  void setKp(const Matrix<double, 6, 1>& kp);

  void setKd(const Matrix<double, 6, 1>& kd);

  VectorXd computeCommand();

  VectorXd advance(VectorXd& q, VectorXd v);

};
}
