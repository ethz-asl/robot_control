#include "robot_control_ros/task_space_controller.h"

#include <franka_example_controllers/compliance_paramConfig.h>
#include <cmath>
#include <Eigen/Dense>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <pinocchio/spatial/se3.hpp>

namespace rc_ros {

bool TaskSpaceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle, ros::NodeHandle& ctrl_handle) {
  std::string robot_description;
  std::string controlled_frame;
  if (!node_handle.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("Can't read robot description.");
    return false;
  }
  if (!ctrl_handle.getParam("controlled_frame", controlled_frame)) {
    ROS_ERROR_STREAM("Set the controlled_frame parameter.");
    return false;
  }

  if (!ctrl_handle.getParam("publish_ros", publish_ros_)) {
    ROS_ERROR_STREAM("Set the publish_ros parameter.");
    return false;
  }

  robot_wrapper = new rc::RobotWrapper();
  robot_wrapper->initFromXml(robot_description);
  controller = new rc::TaskSpaceController(robot_wrapper, controlled_frame);

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

  pose_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>>(node_handle, "/current_pose", 10);
  target_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>>(node_handle, "/target_pose", 10);
  return true;
}

void TaskSpaceController::starting(const ros::Time& time) {
  Eigen::Affine3d initial_transform;
  franka::RobotState initial_state = state_handle_->getRobotState();
  initial_transform = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  pin::SE3 target = pin::SE3(initial_transform.rotation(), initial_transform.translation());
  controller->setTaskTarget(target);
}

void TaskSpaceController::update(const ros::Time& time, const ros::Duration& period) {
  Eigen::VectorXd joint_positions = getJointPositions();
  Eigen::VectorXd joint_velocities = getJointVelocities();
  Eigen::VectorXd command = controller->advance(joint_positions, joint_velocities);
  for (int i=0; i < 7; i++) {
    joint_handles_[i].setCommand(command[i]);
  }
  if (publish_ros_)
    publishRos();
}

Eigen::VectorXd TaskSpaceController::getJointVelocities() const {
  // gripper adds 2 joints but franka state reports only the arm
  franka::RobotState state = state_handle_->getRobotState();
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(9);
  joint_velocities.head<7>() = Eigen::Matrix<double, 7, 1>::Map(state.dq.data());
  return joint_velocities;
}

Eigen::VectorXd TaskSpaceController::getJointPositions() const {
  // gripper adds 2 joints but franka state reports only the arm
  franka::RobotState state = state_handle_->getRobotState();
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(9);
  joint_positions.head<7>() = Eigen::Matrix<double, 7, 1>::Map(state.q.data());
  return joint_positions;
}

void TaskSpaceController::publishRos() {
  if (target_publisher_->trylock()){
    target_publisher_->msg_.header.stamp = ros::Time::now();
    target_publisher_->msg_.header.frame_id = "world";
    target_publisher_->msg_.pose.position.x = target_pose_.translation()(0);
    target_publisher_->msg_.pose.position.y = target_pose_.translation()(1);
    target_publisher_->msg_.pose.position.z = target_pose_.translation()(2);
    Eigen::Quaterniond q(target_pose_.rotation());
    target_publisher_->msg_.pose.orientation.x = q.x();
    target_publisher_->msg_.pose.orientation.y = q.y();
    target_publisher_->msg_.pose.orientation.z = q.z();
    target_publisher_->msg_.pose.orientation.w = q.w();
    target_publisher_->unlockAndPublish();
  }

  if (pose_publisher_->trylock()){
    current_pose_ = robot_wrapper->getFramePlacement(controlled_frame_);
    pose_publisher_->msg_.header.stamp = ros::Time::now();
    pose_publisher_->msg_.header.frame_id = "world";
    pose_publisher_->msg_.pose.position.x = current_pose_.translation()(0);
    pose_publisher_->msg_.pose.position.y = current_pose_.translation()(1);
    pose_publisher_->msg_.pose.position.z = current_pose_.translation()(2);
    Eigen::Quaterniond q(current_pose_.rotation());
    pose_publisher_->msg_.pose.orientation.x = q.x();
    pose_publisher_->msg_.pose.orientation.y = q.y();
    pose_publisher_->msg_.pose.orientation.z = q.z();
    pose_publisher_->msg_.pose.orientation.w = q.w();
    pose_publisher_->unlockAndPublish();
  }
}

}

PLUGINLIB_EXPORT_CLASS(rc_ros::TaskSpaceController, controller_interface::ControllerBase)

