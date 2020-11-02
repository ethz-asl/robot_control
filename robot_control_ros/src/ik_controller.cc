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


namespace rc_ros {

template<class SI, class SH, class CI, class CH>
bool IKControllerBase<SI, SH, CI, CH>::init(hardware_interface::RobotHW* robot_hw,
                                            ros::NodeHandle& node_handle,
                                            ros::NodeHandle& ctrl_handle) {
  std::string robot_description;
  if (!node_handle.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("Can't read robot description.");
    return false;
  }

  if (!ctrl_handle.getParam("publish_ros", publish_ros_)) {
    ROS_ERROR_STREAM("Set the publish_ros parameter.");
    return false;
  }

  std::string root_link;
  if (!ctrl_handle.getParam("root_link", root_link)){
    ROS_ERROR_STREAM("Set the root link.");
    return false;
  }

  std::string ee_link;
  if (!ctrl_handle.getParam("end_effector_link", ee_link)){
    ROS_ERROR_STREAM("Set the end effector link.");
    return false;
  }

  urdf::Model urdf;
  urdf.initString(robot_description);
  if (!urdf.getLink(root_link)){
    ROS_ERROR_STREAM("Could not find link " << root_link << " in urdf.");
    return false;
  }

  if (!urdf.getLink(ee_link)){
    ROS_ERROR_STREAM("Could not find link " << ee_link << " in urdf.");
    return false;
  }
  controlled_frame_ = ee_link;

  robot_wrapper = std::make_shared<rc::RobotWrapper>();
  robot_wrapper->initFromXml(robot_description);

  ctrl_handle.param("debug", debug_, false);
  bool verbose = false;
  controller = std::make_shared<rc::IKNullSpaceController_KDL>(verbose);
  controller->initFromXmlString(robot_description, root_link, ee_link);
  nr_chain_joints_ = controller->getNumJoints();
  command_.setZero(nr_chain_joints_);

  if (!ctrl_handle.getParam("joint_names", joint_names_)) {
    ROS_ERROR_STREAM("Set the joint_names parameter.");
    return false;
  }

  std::vector<double> p_gains;
  if (!ctrl_handle.getParam("p_gains", p_gains)){
    ROS_ERROR_STREAM("Set the proportional gains.");
    return false;
  }

  std::vector<double> d_gains;
  if (!ctrl_handle.getParam("d_gains", d_gains)){
    ROS_ERROR_STREAM("Set the derivative gains.");
    return false;
  }

  std::vector<double> default_configuration;
  if (!ctrl_handle.getParam("default_configuration", default_configuration)){
    ROS_ERROR_STREAM("Set the default configuration.");
    return false;
  }

  bool user_weights = true;
  std::vector<double> configuration_weights;
  if (!ctrl_handle.getParam("configuration_weights", configuration_weights)){
    ROS_WARN_STREAM("Configuration weights not set: setting all to 1.");
    user_weights = false;
  }

  if (joint_names_.size() != nr_chain_joints_){
    ROS_ERROR("joint names has the wrong size");
    return false;
  }

  if (p_gains.size() != nr_chain_joints_){
    ROS_ERROR("proportional gains the wrong size");
    return false;
  }

  if (d_gains.size() != nr_chain_joints_){
    ROS_ERROR("derivative gains have the wrong size");
    return false;
  }

  Kp_.setZero(nr_chain_joints_);
  Kd_.setZero(nr_chain_joints_);
  for (size_t i=0; i<nr_chain_joints_; i++){
    Kp_(i) = p_gains[i];
    Kd_(i) = d_gains[i];
  }
  ROS_INFO_STREAM("Kp: " << Kp_.transpose());
  ROS_INFO_STREAM("Kd: " << Kd_.transpose());

  if (default_configuration.size() != nr_chain_joints_){
    ROS_ERROR("default configuration has the wrong size");
    return false;
  }

  if (user_weights && configuration_weights.size() != nr_chain_joints_){
    ROS_ERROR("configuration weights have the wrong size");
    return false;
  }

  VectorXd q_nullspace(nr_chain_joints_);
  VectorXd q_nullspace_weights(nr_chain_joints_);

  for (size_t i=0; i<nr_chain_joints_; i++){
    q_nullspace(i) = default_configuration[i];
    q_nullspace_weights(i) = user_weights ? configuration_weights[i] : 1.0;
  }

  q_nullspace_ = q_nullspace;

  controller->setNullspaceConfiguration(q_nullspace);
  controller->setNullspaceWeights(q_nullspace_weights);
  controller->setJointLimitsFromUrdf(robot_description, joint_names_);

  bool state_handle_ok = addStateHandles(robot_hw);
  if (!state_handle_ok) {
    ROS_ERROR("Failed to add the joint state handles.");
    return false;
  }

  bool command_handle_ok = addCommandHandles(robot_hw);
  if (!command_handle_ok) {
    ROS_ERROR("Failed to add the joint command handles.");
    return false;
  }

  std::string current_pose_topic = ctrl_handle.param<std::string>("current_pose_topic", "/current_pose");
  pose_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>>(node_handle, current_pose_topic, 10);

  std::string target_pose_topic = ctrl_handle.param<std::string>("target_pose_topic", "/target_pose");
  target_subscriber_ = node_handle.subscribe("/target_pose", 10, &IKControllerBase::newTargetCallback, this);

  return true;
}

template <class SI, class SH, class CI, class CH>
void IKControllerBase<SI, SH, CI, CH>::newTargetCallback(const geometry_msgs::PoseStamped& msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  Eigen::Vector3d translation(msg.pose.position.x,
                              msg.pose.position.y,
                              msg.pose.position.z);
  Eigen::Quaterniond rotation(msg.pose.orientation.w,
                              msg.pose.orientation.x,
                              msg.pose.orientation.y,
                              msg.pose.orientation.z);
  target_pose_ = pin::SE3(rotation, translation);
  q_current_ = getJointPositions();
  q_desired_ = controller->computeCommand(target_pose_.rotation(),
                                          target_pose_.translation(),
                                          q_current_);
  if (debug_) ROS_INFO_STREAM_THROTTLE(0.5, ">>> " << q_desired_.transpose());
}

template<class SI, class SH, class CI, class CH>
void IKControllerBase<SI, SH, CI, CH>::starting(const ros::Time& time) {
  q_current_ = getJointPositions();
  q_desired_ = q_current_;
  qd_current_ = getJointVelocities();
  robot_wrapper->updateState(q_current_, qd_current_, true);
  target_pose_ = robot_wrapper->getFramePlacement(controlled_frame_);
}

template<class SI, class SH, class CI, class CH>
void IKControllerBase<SI, SH, CI, CH>::updateCommand(){
  std::lock_guard<std::mutex> lock(target_mutex_);
  q_current_ = getJointPositions();
  qd_current_ = getJointVelocities();
  robot_wrapper->updateState(q_current_, qd_current_, true);
  robot_wrapper->computeAllTerms();

  Eigen::VectorXd nl_terms = robot_wrapper->getNonLinearTerms().head(nr_chain_joints_);
  Eigen::VectorXd y = Kp_.cwiseProduct(q_desired_- q_current_.head(nr_chain_joints_)) -
                      Kd_.cwiseProduct(qd_current_.head(nr_chain_joints_));
  command_ = nl_terms + robot_wrapper->getInertia().topLeftCorner(nr_chain_joints_, nr_chain_joints_) * y;

  if (publish_ros_) publishRos();

  if (debug_){
    ROS_INFO_STREAM_THROTTLE(5.0,
       "tau command: " << command_.transpose() << std::endl
    << "q_desired_   " << q_desired_.transpose() << std::endl
    << "nlt:         " << nl_terms.transpose() << std::endl
    << "y:           " << y.transpose() << std::endl
    << "q_current_   " << q_current_.transpose() << std::endl
    << "inertia matrix: " << robot_wrapper->getInertia() << std::endl
    << "qd_current_  " << qd_current_.transpose() << std::endl
    << "rot desired  " << target_pose_.rotation() << std::endl
    << "pos desired  " << target_pose_.translation().transpose());
  }
}

template<class SI, class SH, class CI, class CH>
void IKControllerBase<SI, SH, CI, CH>::update(const ros::Time& time, const ros::Duration& period) {
  updateCommand();
  for (size_t i=0; i < nr_chain_joints_; i++) {
    joint_handles_[i].setCommand(command_[i]);
  }
}

template<class SI, class SH, class CI, class CH>
Eigen::VectorXd IKControllerBase<SI, SH, CI, CH>::getJointVelocities() const {
  Eigen::VectorXd joint_velocities(robot_wrapper->getDof());
  joint_velocities.setZero();
  for(size_t i=0; i < nr_chain_joints_; i++){
    joint_velocities(i) = state_handles_[i].getVelocity();
  }
  return joint_velocities;
}

template<class SI, class SH, class CI, class CH>
Eigen::VectorXd IKControllerBase<SI, SH, CI, CH>::getJointPositions() const {
  Eigen::VectorXd joint_positions(robot_wrapper->getDof());
  joint_positions.setZero();
  for(size_t i=0; i< nr_chain_joints_; i++) {
    joint_positions(i) = state_handles_[i].getPosition();
  }
  return joint_positions;
}

template<class SI, class SH, class CI, class CH>
void IKControllerBase<SI, SH, CI, CH>::publishRos() {
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

/// Specializations
template<class SI, class SH, class CI, class CH>
bool IKControllerBase<SI, SH, CI, CH>::addCommandHandles(hardware_interface::RobotHW * robot_hw) {
  auto effort_joint_interface = robot_hw->get<CI>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting effort joint interface.");
    return false;
  }
  for (auto& joint_name : joint_names_) {
    joint_handles_.push_back(effort_joint_interface->getHandle(joint_name));
  }
  return true;
}

bool IKControllerEffortSim::addStateHandles(hardware_interface::RobotHW* robot_hw) {
  auto state_interface = robot_hw->get<hardware_interface::JointStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get franka state interface");
    return false;
  }

  for (auto& joint_name : joint_names_) {
    state_handles_.push_back(state_interface->getHandle(joint_name));
  }
  return true;
}

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

}

PLUGINLIB_EXPORT_CLASS(rc_ros::IKControllerPanda, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(rc_ros::IKControllerEffortSim, controller_interface::ControllerBase)
