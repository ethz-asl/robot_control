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

#define DEFAULT_CONFIGURATION 0.0, -0.473, 0.10720, -1.937, -0.1610, 1.48333, 0.75098

namespace rc_ros {

template<class StateInterface, class StateHandle>
bool IKControllerBase<StateInterface, StateHandle>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle, ros::NodeHandle& ctrl_handle) {
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

  bool verbose = false;
  controller = std::make_shared<rc::IKNullSpaceController_KDL>(verbose);

  VectorXd q_nullspace(7), q_nullspace_weights(7);
  q_nullspace << DEFAULT_CONFIGURATION;
  q_nullspace_weights.setConstant(1.0);
  controller->initFromXmlString(robot_description, "panda_link0", "panda_hand", q_nullspace, q_nullspace_weights);
  controller->setJointLimitsFromUrdf(robot_description, joint_names_);

  bool added = addStateHandles(robot_hw);
  if (!added) {
    return added;
  }

  std::string current_pose_topic = ctrl_handle.param<std::string>("current_pose_topic", "/current_pose");
  pose_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>>(node_handle, current_pose_topic, 10);

  std::string target_pose_topic = ctrl_handle.param<std::string>("target_pose_topic", "/target_pose");
  target_subscriber_ = node_handle.subscribe("/target_pose", 10, &IKControllerBase::newTargetCallback, this);

  std::string set_target_service_name = ctrl_handle.param<std::string>("set_target_service", "/set_target");
  set_target_service_ = node_handle.advertiseService(set_target_service_name, &IKControllerBase::setTargetCallback, this);
  return true;
}

template <class SI, class SH>
void IKControllerBase<SI, SH>::newTargetCallback(const geometry_msgs::PoseStamped& msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  Eigen::Vector3d translation(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  Eigen::Quaterniond rotation(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
  target_pose_ = pin::SE3(rotation, translation);
  q_current_ = getJointPositions();
  q_desired_ = controller->computeCommand(target_pose_.rotation(),
                                          target_pose_.translation(),
                                          q_current_);
  ROS_INFO_STREAM_THROTTLE(1.0, "New target received: " << target_pose_);
}

template <class SI, class SH>
bool IKControllerBase<SI, SH>::setTargetCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
  std::lock_guard<std::mutex> lock(target_mutex_);
  q_current_ = getJointPositions();
  q_desired_ = controller->computeCommand(target_pose_.rotation(),
                                          target_pose_.translation(),
                                          q_current_);
  ROS_WARN_STREAM("New q desired computed: " << q_desired_.transpose());
  return true;
}

bool IKController::addStateHandles(hardware_interface::RobotHW* robot_hw) {
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

bool IKControllerSim::addStateHandles(hardware_interface::RobotHW* robot_hw) {
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
void IKControllerBase<SI, SH>::starting(const ros::Time& time) {
  Kp_.setZero(robot_wrapper->getDof());
  Kd_.setZero(robot_wrapper->getDof());
  Kp_.setConstant(100.0);
  Kd_.setConstant(10.0);
  q_current_ = getJointPositions();
  q_desired_ = q_current_;
  qd_current_ = getJointVelocities();
  robot_wrapper->updateState(q_current_, qd_current_, true);
  target_pose_ = robot_wrapper->getFramePlacement(controlled_frame_);

}

template<class SI, class SH>
void IKControllerBase<SI, SH>::update(const ros::Time& time, const ros::Duration& period) {

  std::lock_guard<std::mutex> lock(target_mutex_);
  q_current_ = getJointPositions();
  qd_current_ = getJointVelocities();
  robot_wrapper->updateState(q_current_, qd_current_, true);
  robot_wrapper->computeAllTerms();

  Eigen::VectorXd nl_terms = robot_wrapper->getNonLinearTerms();
  Eigen::VectorXd y = Kp_.cwiseProduct(q_desired_- q_current_.head(7)) - Kd_.cwiseProduct(qd_current_);
  command_ = nl_terms + robot_wrapper->getInertia() * y;
  ROS_INFO_STREAM_THROTTLE(5.0,
       "tau command: " << command_.transpose() << std::endl
    << "q_desired_   " << q_desired_.transpose() << std::endl
    << "nlt:         " << nl_terms.transpose() << std::endl
    << "q_current_   " << q_current_.transpose() << std::endl
    << "qd_current_  " << qd_current_.transpose() << std::endl
    << "rot desired  " << target_pose_.rotation() << std::endl
    << "pos desired  " << target_pose_.translation().transpose());
  for (size_t i=0; i < 7; i++) {
    joint_handles_[i].setCommand(command_[i]);
  }
  if (publish_ros_)
    publishRos();
}

template<class SI, class SH>
Eigen::VectorXd IKControllerBase<SI, SH>::getJointVelocities() const {
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(9);
  for(size_t i=0; i < 7; i++){
    joint_velocities(i) = state_handles_[i].getVelocity();
  }
  return joint_velocities;
}

template<class SI, class SH>
Eigen::VectorXd IKControllerBase<SI, SH>::getJointPositions() const {
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(9);
  for(size_t i=0; i< 7; i++) {
    joint_positions(i) = state_handles_[i].getPosition();
  }
  return joint_positions;
}

template<class SI, class SH>
void IKControllerBase<SI, SH>::publishRos() {
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

PLUGINLIB_EXPORT_CLASS(rc_ros::IKController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(rc_ros::IKControllerSim, controller_interface::ControllerBase)
