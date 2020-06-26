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

struct IKSolverConfig{
  std::string xml_string;
  std::string urdf_file;
  std::string root_link;
  std::string tip_link;

  double accuracy = 1e-9;
};

class IKSolverBase{
 public:
  IKSolverBase() = delete;
  IKSolverBase(const IKSolverConfig config) : config_(config){};
  ~IKSolverBase() = default;

 protected:
  IKSolverConfig config_;
  virtual std::optional<JointsVelocity> jointVelFromTwist(const Twist& twist, const JointsPosition& q_in) = 0;
  virtual std::optional<JointsPosition> jointPosFromPose(const Pose& pose) = 0;
};

}
