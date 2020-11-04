/*!
 * @file     ik_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     29.10.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control_ros/ik_controller.h"

#include <franka_example_controllers/compliance_paramConfig.h>
#include <cmath>
#include <Eigen/Dense>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <pinocchio/spatial/se3.hpp>
#include <functional>
#include <urdf/model.h>

using namespace rc_ros;

/// Panda Specific controller
bool IKControllerPanda::addStateHandles(hardware_interface::RobotHW* robot_hw) {
  auto state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get franka state interface");
    return false;
  }

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;
  try {
    state_handle = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle("panda_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("Excepion getting franka state handle: " << ex.what());
    return false;
  }

  const franka::RobotState& state = state_handle->getRobotState();
  for (size_t i = 0; i < nr_chain_joints_; i++) {
    hardware_interface::JointStateHandle joint_state_handle = hardware_interface::JointStateHandle(joint_names_[i], &state.q[i],
                                                                                                   &state.dq[i], &state.tau_J[i]);
    state_handles_.push_back(joint_state_handle);
  }
  return true;
}


PLUGINLIB_EXPORT_CLASS(rc_ros::IKControllerPanda, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(rc_ros::IKControllerEffortSim, controller_interface::ControllerBase)
