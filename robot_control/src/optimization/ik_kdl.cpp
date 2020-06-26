/*!
 * @file     ik_kdl.cpp
 * @author   Giuseppe Rizzi
 * @date     26.06.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control/optimization/ik_kdl.h"
#include <iostream>

using namespace rc;

IKSolverKDL::IKSolverKDL(const rc::IKSolverConfig config) : IKSolverBase(config){
  initSolver();
}


void IKSolverKDL::initSolver(){
  KDL::Tree kdl_tree;
  if (!config_.urdf_file.empty()) {
    if (!kdl_parser::treeFromFile(config_.urdf_file, kdl_tree))
      throw std::runtime_error("Failed to initialize KDL solver.");
  }
  else if (!config_.xml_string.empty()) {
    if (!kdl_parser::treeFromString(config_.urdf_file, kdl_tree))
      throw std::runtime_error("Failed to initialize KDL solver.");
  }
  else{
    throw std::runtime_error("Failed to initialize KDL solver. Both urdf and xml are empty.");
  }

  if (!kdl_tree.getChain(config_.root_link, config_.tip_link, kdl_chain_)){
    throw std::invalid_argument("Failed to extract chain");
  }

  num_joints_ = kdl_chain_.getNrOfJoints();
  ik_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
}

std::optional<JointsVelocity> IKSolverKDL::jointVelFromTwist(const Twist &twist, const JointsPosition& q_in) {
  KDL::JntArray q_in_arr(num_joints_);
  KDL::Twist v_desired;
  for (size_t i = 0; i < num_joints_; i++)
  {
    q_in_arr(i) = q_in[i];
  }

  for (int i = 0; i < 3; i++)
  {
    v_desired[i] = twist.linear()(i);
    v_desired[i + 3] = twist.angular()(i);
  }

  KDL::JntArray qdot_out(num_joints_);
  ik_solver_->CartToJnt(q_in_arr, v_desired, qdot_out);

  JointsVelocity res;
  res.resize(num_joints_);
  for (int i = 0; i < num_joints_; i++)
    res[i] = qdot_out(i);

  return res;
}

std::optional<JointsPosition> IKSolverKDL::jointPosFromPose(const Pose &pose) {
  std::cout << "Not implemented yet!" << std::endl;
  return {};
}


