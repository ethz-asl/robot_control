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
  IKSolverKDL(const IKSolverConfig config);
  ~IKSolverKDL() = default;

 private:
  void initSolver();

 protected:
  std::optional<JointsVelocity> jointVelFromTwist(const Twist& twist, const JointsPosition& q_in) override;
  std::optional<JointsPosition> jointPosFromPose(const Pose& pose) override;

 private:
 private:
  KDL::Chain kdl_chain_;
  unsigned int num_joints_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_;
};

}