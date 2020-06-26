/*!
 * @file     ik_kdl.h
 * @author   Giuseppe Rizzi
 * @date     26.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include "robot_control/optimization/ik_base.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

namespace rc{
class IKSolverKDL : public IKSolverBase{
 public:
  IKSolverKDL() = default;
  ~IKSolverKDL() = default;

  bool initFromXml(const std::string& xml_string);
  bool initFromUrdf(const std::string& urdf_path);

 protected:
  std::optional<JointsVelocity> jointVelFromTwist(const Twist &twist) override;
  std::optional<JointsPosition> jointPosFromPose(const Pose &pose) override;

 private:
 private:
  KDL::Chain kdlChain;
  unsigned int numJoints;
  KDL::ChainIkSolverVel_pinv *ikSolver;
};

}