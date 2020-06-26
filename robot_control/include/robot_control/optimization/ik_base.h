/*!
 * @file     ik.h
 * @author   Giuseppe Rizzi
 * @date     26.06.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include "robot_control/typedefs.h"
#include <string>
#include <optional>
#include <Eigen/Core>

namespace rc{

class IKSolverBase{
 public:
  IKSolverBase() = delete;
  ~IKSolverBase() = default;

 protected:
  virtual std::optional<JointsVelocity> jointVelFromTwist(const Twist& twist) = 0;
  virtual std::optional<JointsPosition> jointPosFromPose(const Pose& pose) = 0;
};

}
