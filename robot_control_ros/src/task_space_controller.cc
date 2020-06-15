#include "robot_control_ros/task_space_controller.h"

#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <pinocchio/spatial/se3.hpp>

namespace rc_ros {

bool TaskSpaceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle, ros::NodeHandle& ctrl_handle) {
  bool is_real_robot;
  std::string robot_description;
  std::string controlled_frame;
  if (!node_handle.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("Can't read robot description.");
    return false;
  }
  if (!ctrl_handle.getParam("controllled_frame", controlled_frame)) {
    ROS_ERROR_STREAM("Set the controller_frame parameter.");
    return false;
  }
  if (!ctrl_handle.getParam("is_real_robot", is_real_robot_)) {
    ROS_ERROR_STREAM("Set the is_real_robot parameter.");
    return false;
  }

  robot_wrapper = new rc::RobotWrapper(robot_description);
  controller = new rc::TaskSpaceController(robot_wrapper, controlled_frame);

  if (is_real_robot_){
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get franka state interface");
      return false;
    }
    try {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle("panda_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("Excepion getting franka state handle: " << ex.what());
      return false;
    }
  }
  else{
    auto state_interface = robot_hw->get<hardware_interface::JointStateInterface>();
    if (state_interface == nullptr){
      ROS_ERROR_STREAM("Can't get the joint state interface");
      return false;
    }
    for (auto & joint_name : joint_names_){
      state_handles_sim_.push_back(state_interface->getHandle(joint_name));
    }
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting effort joint interface.");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("Exception getting joint handles: " << ex.what());
      return false;
    }
	}
}

void TaskSpaceController::starting(const ros::Time& time) {
  pin::SE3 target;
  if (is_real_robot_){
    Eigen::Affine3d initial_transform;
    franka::RobotState initial_state = state_handle_->getRobotState();
    initial_transform = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    target = pin::SE3(initial_transform.rotation(), initial_transform.translation());

  }
  else{
    Eigen::VectorXd initial_q = getJointPositions();
    Eigen::VectorXd initial_v = getJointVelocities();
    robot_wrapper->updateState(initial_q, initial_v);
    target = robot_wrapper->getFramePlacement(joint_names_[END_EFFECTOR_INDEX]);
  }
  // Set the current position to be the target position.
  controller->setTaskTarget(target);
}

void TaskSpaceController::update(const ros::Time& time, const ros::Duration& period) {
  Eigen::VectorXd joint_positions = getJointPositions();
  Eigen::VectorXd joint_velocities = getJointVelocities();
  Eigen::VectorXd command = controller->advance(joint_positions, joint_velocities);
  for (int i=0; i < 7; i++) {
    joint_handles_[i].setCommand(command[i]);
  }
}

Eigen::VectorXd TaskSpaceController::getJointVelocities() const {
  if (is_real_robot_){
    // gripper adds 2 joints but franka state reports only the arm
    franka::RobotState state = state_handle_->getRobotState();
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(9);
    joint_velocities.head<7>() = Eigen::Matrix<double, 7, 1>::Map(state.dq.data());
    return joint_velocities;
  }
  else{
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(9);
    for(size_t i=0; i < 7; i++){
      joint_velocities(i) = state_handles_sim_[i].getVelocity();
    }
    return joint_velocities;
  }
}

Eigen::VectorXd TaskSpaceController::getJointPositions() const {
  if (is_real_robot_){
    // gripper adds 2 joints but franka state reports only the arm
    franka::RobotState state = state_handle_->getRobotState();
    Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(9);
    joint_positions.head<7>() = Eigen::Matrix<double, 7, 1>::Map(state.q.data());
    return joint_positions;
  }
  else{
    Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(9);
    for(size_t i=0; i< 7; i++){
      joint_positions(i) = state_handles_sim_[i].getVelocity();
    }
    return joint_positions;
  }
}

}

PLUGINLIB_EXPORT_CLASS(rc_ros::TaskSpaceController, controller_interface::ControllerBase)

