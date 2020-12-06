/*!
 * @file     impedance_controller.h
 * @author   Giuseppe Rizzi
 * @date     04.12.2020
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
class ImpedanceController {
 private:
  std::shared_ptr<RobotWrapper> wrapper;
  bool target_set = false;
  MatrixXd J;
  std::mutex target_mutex;

 public:
  pin::SE3 target;
  std::string controlled_frame;
  Matrix<double, 6, 6> kp_;
  Matrix<double, 6, 6> kd_;
  VectorXd q_nullspace_;
  double filter_params_{0.005};
  double nullspace_stiffness_{2.0};

  ImpedanceController(std::shared_ptr<RobotWrapper> wrp, std::string& controlled_frame);

  void setTaskTarget(pin::SE3 task_target);

  void setKp(const Matrix<double, 6, 1>& kp);

  void setKd(const Matrix<double, 6, 1>& kd);

  VectorXd computeCommand();

  VectorXd advance(VectorXd& q, VectorXd& v);

  inline void pseudoInverse(const MatrixXd& m, MatrixXd& m_pinv, bool damped=true);

};
}