/*!
 * @file     typedefs.h
 * @author   Giuseppe Rizzi
 * @date     26.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/force.hpp>
#include <Eigen/Core>

namespace rc{
  typedef pinocchio::SE3 Pose;
  typedef pinocchio::Motion Twist;
  typedef pinocchio::Force Wrench;

  typedef Eigen::VectorXd JointsPosition;
  typedef Eigen::VectorXd JointsVelocity;
  typedef Eigen::VectorXd JointsTorque;

  typedef Eigen::Matrix4d Transform;
}