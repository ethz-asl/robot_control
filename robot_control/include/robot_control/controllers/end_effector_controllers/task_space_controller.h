#pragma once
#include <mutex>
#include <pinocchio/fwd.hpp>
#include "robot_control/math/math.h"
#include "robot_control/modeling/robot_wrapper.h"

using namespace Eigen;
namespace pin = pinocchio;

namespace rc {
class TaskSpaceController {
  private:
  std::shared_ptr<RobotWrapper> wrapper;
  bool target_set = false;
  JacobiSVD<MatrixXd> svd;
  linear_algebra::DLSSolver solver;
  MatrixXd J;
  MatrixXd dJ;
  std::mutex target_mutex;

  public:
  pin::SE3 target;
  std::string controlled_frame;
  MatrixXd kp_, kd_, kqd_ns, kqp_res, q_rest;

  TaskSpaceController(std::shared_ptr<RobotWrapper> wrp, std::string& controlled_frame);

  void setTaskTarget(pin::SE3 task_target);

  void setKp(const Matrix<double, 6, 1>& kp);

  void setKd(const Matrix<double, 6, 1>& kd);

  VectorXd computeCommand();

  VectorXd advance(VectorXd& q, VectorXd& v);

};
}
