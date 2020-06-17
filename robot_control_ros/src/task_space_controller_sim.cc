#include "robot_control_ros/task_space_controller_sim.h"

#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <pinocchio/spatial/se3.hpp>

namespace rc_ros {

bool TaskSpaceControllerSim::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle, ros::NodeHandle& ctrl_handle) {
  std::string robot_description;
  if (!node_handle.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("Can't read robot description.");
    return false;
  }
  if (!ctrl_handle.getParam("controlled_frame", controlled_frame_)) {
    ROS_ERROR_STREAM("Set the controlled_frame parameter.");
    return false;
  }
  ROS_INFO_STREAM("Controlling frame [" << controlled_frame_ << "]");

  if (!ctrl_handle.getParam("publish_ros", publish_ros_)) {
    ROS_ERROR_STREAM("Set the publish_ros parameter.");
    return false;
  }

  robot_wrapper = new rc::RobotWrapper();
  robot_wrapper->initFromXml(robot_description);
  controller = new rc::TaskSpaceController(robot_wrapper, controlled_frame_);

  auto state_interface = robot_hw->get<hardware_interface::JointStateInterface>();
  if (state_interface == nullptr){
    ROS_ERROR_STREAM("Can't get the joint state interface");
    return false;
  }
  for (auto & joint_name : joint_names_){
    state_handles_sim_.push_back(state_interface->getHandle(joint_name));
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting effort joint interface.");
    return false;
  }
  for (size_t i = 0; i < 7; i++) {
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

void TaskSpaceControllerSim::starting(const ros::Time& time) {
  Eigen::VectorXd initial_q = getJointPositions();
  Eigen::VectorXd initial_v = getJointVelocities();
  robot_wrapper->updateState(initial_q, initial_v);
  target_pose_ = robot_wrapper->getFramePlacement(joint_names_[END_EFFECTOR_INDEX]);
  controller->setTaskTarget(target_pose_);
}

void TaskSpaceControllerSim::update(const ros::Time& time, const ros::Duration& period) {
  Eigen::VectorXd joint_positions = getJointPositions();
  Eigen::VectorXd joint_velocities = getJointVelocities();
  Eigen::VectorXd command = controller->advance(joint_positions, joint_velocities);
  for (int i=0; i < 7; i++) {
    joint_handles_[i].setCommand(command[i]);
  }
  if (publish_ros_){
    publishRos();
  }
}

Eigen::VectorXd TaskSpaceControllerSim::getJointVelocities() const {
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(9);
  for(size_t i=0; i < 7; i++){
    joint_velocities(i) = state_handles_sim_[i].getVelocity();
  }
  return joint_velocities;
}

Eigen::VectorXd TaskSpaceControllerSim::getJointPositions() const {
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(9);
  for(size_t i=0; i< 7; i++){
    joint_positions(i) = state_handles_sim_[i].getPosition();
  }
  return joint_positions;
}

void TaskSpaceControllerSim::publishRos() {
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
    pin::SE3 current_pose = robot_wrapper->getFramePlacement(controlled_frame_);
    pose_publisher_->msg_.header.stamp = ros::Time::now();
    pose_publisher_->msg_.header.frame_id = "world";
    pose_publisher_->msg_.pose.position.x = current_pose.translation()(0);
    pose_publisher_->msg_.pose.position.y = current_pose.translation()(1);
    pose_publisher_->msg_.pose.position.z = current_pose_.translation()(2);
    Eigen::Quaterniond q(current_pose.rotation());
    pose_publisher_->msg_.pose.orientation.x = q.x();
    pose_publisher_->msg_.pose.orientation.y = q.y();
    pose_publisher_->msg_.pose.orientation.z = q.z();
    pose_publisher_->msg_.pose.orientation.w = q.w();
    pose_publisher_->unlockAndPublish();
  }

}

}

PLUGINLIB_EXPORT_CLASS(rc_ros::TaskSpaceControllerSim, controller_interface::ControllerBase)

