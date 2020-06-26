#include "robot_control_ros/task_space_controller.h"

#include <franka_example_controllers/compliance_paramConfig.h>
#include <cmath>
#include <Eigen/Dense>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <pinocchio/spatial/se3.hpp>
#include <functional>

namespace rc_ros {

template<class StateInterface, class StateHandle>
bool TaskSpaceControllerBase<StateInterface, StateHandle>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle, ros::NodeHandle& ctrl_handle) {
  std::string robot_description;
  if (!node_handle.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("Can't read robot description.");
    return false;
  }
  if (!ctrl_handle.getParam("controlled_frame", controlled_frame_)) {
    ROS_ERROR_STREAM("Set the controlled_frame parameter.");
    return false;
  }

  if (!ctrl_handle.getParam("publish_ros", publish_ros_)) {
    ROS_ERROR_STREAM("Set the publish_ros parameter.");
    return false;
  }

  robot_wrapper = std::make_shared<rc::RobotWrapper>();
  robot_wrapper->initFromXml(robot_description);
  controller = std::make_shared<rc::TaskSpaceController>(robot_wrapper, controlled_frame_);

  bool added = addStateHandles(robot_hw);
  if (!added) {
    return added;
  }

  std::string current_pose_topic = ctrl_handle.param<std::string>("current_pose_topic", "/current_pose");
  pose_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>>(node_handle, current_pose_topic, 10);

  std::string target_pose_topic = ctrl_handle.param<std::string>("target_pose_topic", "/target_pose");
  target_subscriber_ = node_handle.subscribe("/target_pose", 10, &TaskSpaceControllerBase::newTargetCallback, this);
  return true;
}

template <class SI, class SH>
void TaskSpaceControllerBase<SI, SH>::newTargetCallback(const geometry_msgs::PoseStamped& msg) {
  Eigen::Vector3d translation(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  Eigen::Quaterniond rotation(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
  target_pose_ = pin::SE3(rotation, translation);
  controller->setTaskTarget(target_pose_);
}

bool TaskSpaceController::addStateHandles(hardware_interface::RobotHW* robot_hw) {
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
  for (size_t i = 0; i < 7; i++) {
    hardware_interface::JointStateHandle joint_state_handle = hardware_interface::JointStateHandle(joint_names_[i], &state.q[i],
        &state.dq[i], &state.tau_J[i]);
    state_handles_.push_back(joint_state_handle);
  }

  auto effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting effort joint interface.");
    return false;
  }
  for (auto& joint_name : joint_names_) {
    joint_handles_.push_back(effort_joint_interface->getHandle(joint_name));
  }
  return true;
}

bool TaskSpaceControllerSim::addStateHandles(hardware_interface::RobotHW* robot_hw) {
  auto state_interface = robot_hw->get<hardware_interface::JointStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get franka state interface");
    return false;
  }

  auto effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting effort joint interface.");
    return false;
  }
  for (auto& joint_name : joint_names_) {
    state_handles_.push_back(state_interface->getHandle(joint_name));
    joint_handles_.push_back(effort_joint_interface->getHandle(joint_name));
  }
  return true;
}

template<class SI, class SH>
void TaskSpaceControllerBase<SI, SH>::starting(const ros::Time& time) {}

template<class SI, class SH>
void TaskSpaceControllerBase<SI, SH>::update(const ros::Time& time, const ros::Duration& period) {
  Eigen::VectorXd joint_positions = getJointPositions();
  Eigen::VectorXd joint_velocities = getJointVelocities();
  Eigen::VectorXd command = controller->advance(joint_positions, joint_velocities);
  for (size_t i=0; i < 7; i++) {
    joint_handles_[i].setCommand(command[i]);
  }
  if (publish_ros_)
    publishRos();
}

template<class SI, class SH>
Eigen::VectorXd TaskSpaceControllerBase<SI, SH>::getJointVelocities() const {
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(9);
  for(size_t i=0; i < 7; i++){
    joint_velocities(i) = state_handles_[i].getVelocity();
  }
  return joint_velocities;
}

template<class SI, class SH>
Eigen::VectorXd TaskSpaceControllerBase<SI, SH>::getJointPositions() const {
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(9);
  for(size_t i=0; i< 7; i++) {
    joint_positions(i) = state_handles_[i].getPosition();
  }
  return joint_positions;
}

template<class SI, class SH>
void TaskSpaceControllerBase<SI, SH>::publishRos() {
  if (pose_publisher_->trylock()){
    current_pose_ = robot_wrapper->getFramePlacement(controlled_frame_);
    pose_publisher_->msg_.header.stamp = ros::Time::now();
    pose_publisher_->msg_.header.frame_id = frame_id_;
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
PLUGINLIB_EXPORT_CLASS(rc_ros::TaskSpaceControllerSim, controller_interface::ControllerBase)

