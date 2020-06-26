/*!
 * @file     ik_kdl.cpp
 * @author   Giuseppe Rizzi
 * @date     26.06.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control/optimization/ik_kdl.h"
#include <iostream>

std::optional<rc::JointsVelocity> rc::IKSolverKDL::jointVelFromTwist(const rc::Twist &twist) {
  return std::optional<JointsVelocity>();
}
std::optional<rc::JointsPosition> rc::IKSolverKDL::jointPosFromPose(const rc::Pose &pose) {
  std::cout << "Not implemented yet!" << std::endl;
  return {};
}
