/*!
 * @file     ik_controller_impl.h
 * @author   Giuseppe Rizzi
 * @date     03.11.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <pinocchio/spatial/se3.hpp>
#include <functional>
#include <urdf/model.h>
#include <angles/angles.h>
#include <tf2/convert.h>

namespace rc_ros {

template<class SI, class SH, class CI, class CH, class... T>
bool IKControllerBase<SI, SH, CI, CH, T...>::init(hardware_interface::RobotHW *robot_hw,
                                                  ros::NodeHandle &node_handle,
                                                  ros::NodeHandle &ctrl_handle) {

  std::string description_name;
  if (!ctrl_handle.param("description_name", description_name, {"/robot_description"})) {
  }
  ROS_INFO_STREAM("Robot description name is " << description_name);

  std::string robot_description;
  if (!node_handle.getParam(description_name, robot_description)) {
    ROS_ERROR_STREAM("Can't read robot description.");
    return false;
  }

  if (!ctrl_handle.getParam("publish_ros", publish_ros_)) {
    ROS_ERROR_STREAM("Set the publish_ros parameter.");
    return false;
  }

  std::string root_link;
  if (!ctrl_handle.getParam("root_link", root_link)) {
    ROS_ERROR_STREAM("Set the root link.");
    return false;
  }

  std::string ee_link;
  if (!ctrl_handle.getParam("end_effector_link", ee_link)) {
    ROS_ERROR_STREAM("Set the end effector link.");
    return false;
  }

  urdf::Model urdf;
  urdf.initString(robot_description);
  if (!urdf.getLink(root_link)) {
    ROS_ERROR_STREAM("Could not find link " << root_link << " in urdf.");
    return false;
  }

  if (!urdf.getLink(ee_link)) {
    ROS_ERROR_STREAM("Could not find link " << ee_link << " in urdf.");
    return false;
  }
  frame_id_ = root_link;
  controlled_frame_ = ee_link;

  robot_wrapper = std::make_shared<rc::RobotWrapper>();
  robot_wrapper->initFromXml(robot_description);

  ctrl_handle.param("debug", debug_, false);
  bool verbose = false;
  controller = std::make_shared<rc::IKNullSpaceController_KDL>(verbose);
  controller->initFromXmlString(robot_description, root_link, ee_link);
  nr_chain_joints_ = controller->getNumJoints();
  eff_command_.setZero(nr_chain_joints_);
  pos_command_.setZero(nr_chain_joints_);

  if (!ctrl_handle.getParam("joint_names", joint_names_)) {
    ROS_ERROR_STREAM("Set the joint_names parameter.");
    return false;
  }

  std::vector<double> p_gains;
  if (!ctrl_handle.getParam("p_gains", p_gains)) {
    ROS_ERROR_STREAM("Set the proportional gains.");
    return false;
  }

  std::vector<double> d_gains;
  if (!ctrl_handle.getParam("d_gains", d_gains)) {
    ROS_ERROR_STREAM("Set the derivative gains.");
    return false;
  }

  std::vector<double> default_configuration;
  if (!ctrl_handle.getParam("default_configuration", default_configuration)) {
    ROS_ERROR_STREAM("Set the default configuration.");
    return false;
  }

  bool user_weights = true;
  std::vector<double> configuration_weights;
  if (!ctrl_handle.getParam("configuration_weights", configuration_weights)) {
    ROS_WARN_STREAM("Configuration weights not set: setting all to 1.");
    user_weights = false;
  }

  if (joint_names_.size() != nr_chain_joints_) {
    ROS_ERROR("joint names has the wrong size");
    return false;
  }

  if (p_gains.size() != nr_chain_joints_) {
    ROS_ERROR("proportional gains the wrong size");
    return false;
  }

  if (d_gains.size() != nr_chain_joints_) {
    ROS_ERROR("derivative gains have the wrong size");
    return false;
  }

  Kp_.setZero(nr_chain_joints_);
  Kd_.setZero(nr_chain_joints_);
  for (size_t i = 0; i < nr_chain_joints_; i++) {
    Kp_(i) = p_gains[i];
    Kd_(i) = d_gains[i];
  }
  ROS_INFO_STREAM("Kp: " << Kp_.transpose());
  ROS_INFO_STREAM("Kd: " << Kd_.transpose());

  if (default_configuration.size() != nr_chain_joints_) {
    ROS_ERROR("default configuration has the wrong size");
    return false;
  }

  if (user_weights && configuration_weights.size() != nr_chain_joints_) {
    ROS_ERROR("configuration weights have the wrong size");
    return false;
  }

  VectorXd q_nullspace(nr_chain_joints_);
  VectorXd q_nullspace_weights(nr_chain_joints_);

  for (size_t i = 0; i < nr_chain_joints_; i++) {
    q_nullspace(i) = default_configuration[i];
    q_nullspace_weights(i) = user_weights ? configuration_weights[i] : 1.0;
  }

  q_nullspace_ = q_nullspace;

  controller->setNullspaceConfiguration(q_nullspace);
  controller->setNullspaceWeights(q_nullspace_weights);
  
  std::vector<double> lower, upper;
  if (!ctrl_handle.getParam("lower_limits", lower)) {
    ROS_WARN_STREAM("Lower limits are not set.");
    return false;
  }

  if (!ctrl_handle.getParam("upper_limits", upper)) {
    ROS_WARN_STREAM("Upper limits are not set.");
    return false;
  }
  
  if (lower.size() != nr_chain_joints_) {
    ROS_ERROR("lower limits have the wrong size");
    return false;
  }

  if (upper.size() != nr_chain_joints_) {
    ROS_ERROR("upper limits have the wrong size");
    return false;
  }

  lower_limit_.resize(nr_chain_joints_);
  upper_limit_.resize(nr_chain_joints_);
  for (size_t i=0; i<nr_chain_joints_; i++){
    lower_limit_(i) = lower[i];
    upper_limit_(i) = upper[i];
  }
  controller->setJointLimits(lower_limit_, upper_limit_);
  ROS_INFO_STREAM("Set lower limits to: " << lower_limit_.transpose());
  ROS_INFO_STREAM("Set upper limits to: " << upper_limit_.transpose());

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

  double max_velocity;
  if (!ctrl_handle.getParam("max_velocity", max_velocity)) {
    ROS_ERROR_STREAM("Max velocity is not set.");
    return false;
  }

  double max_acceleration;
  if (!ctrl_handle.getParam("max_acceleration", max_acceleration)) {
    ROS_ERROR_STREAM("Max acceleration is not set.");
    return false;
  }
  generator_ = std::make_unique<rc::TrajectoryGenerator>(max_velocity, max_acceleration, (unsigned int)nr_chain_joints_);

  ROS_INFO_STREAM("Trajectory profile with v max = " << max_velocity << " and a max = " << max_acceleration);

  std::string current_pose_topic = ctrl_handle.param<std::string>("current_pose_topic", "/current_pose");
  pose_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>>(node_handle,
                                                                                                    current_pose_topic,
                                                                                                    10);

  q_current_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::JointState>>(node_handle,
                                                                                                    "/ik_controller/q_current",
                                                                                                    10);
  q_current_publisher_->msg_.name = joint_names_;
  q_current_publisher_->msg_.position.resize(nr_chain_joints_, 0.0);

  q_desired_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::JointState>>(node_handle,
                                                                                                    "/ik_controller/q_desired",
                                                                                                    10);
  q_desired_publisher_->msg_.name = joint_names_;
  q_desired_publisher_->msg_.position.resize(nr_chain_joints_, 0.0);


  std::string target_pose_topic = ctrl_handle.param<std::string>("target_pose_topic", "/target_pose");
  target_subscriber_ = node_handle.subscribe("/target_pose", 10, &IKControllerBase::newTargetCallback, this);
  q_err_ = Eigen::VectorXd(nr_chain_joints_);
  started_ = false;
  ROS_INFO("IK controller initialized");
  return true;
}

template<class SI, class SH, class CI, class CH, class... T>
void IKControllerBase<SI, SH, CI, CH, T...>::newTargetCallback(const geometry_msgs::PoseStamped &msg) {
  if (!started_ ) return;

  try {
    root_link_tf = tf_buffer.lookupTransform(frame_id_, msg.header.frame_id, ros::Time::now(), ros::Duration(1.0));
  }
  catch (const tf2::TransformException& exc){
    ROS_ERROR_STREAM("Tf lookup failed: " << exc.what());
  }

  geometry_msgs::PoseStamped pose_in_root_frame;
  tf2::doTransform(msg, pose_in_root_frame, root_link_tf);

  Eigen::Vector3d translation(pose_in_root_frame.pose.position.x,
                              pose_in_root_frame.pose.position.y,
                              pose_in_root_frame.pose.position.z);
  Eigen::Quaterniond rotation(pose_in_root_frame.pose.orientation.w,
                              pose_in_root_frame.pose.orientation.x,
                              pose_in_root_frame.pose.orientation.y,
                              pose_in_root_frame.pose.orientation.z);

  target_pose_ = pin::SE3(rotation, translation);
  adaptTarget(target_pose_);
  computeIK();
  timeTrajectory();
}

template<class SI, class SH, class CI, class CH, class... T>
void IKControllerBase<SI, SH, CI, CH, T...>::computeIK(){
  std::string error{""};
  Eigen::VectorXd q_desired = controller->computeCommand(target_pose_.rotation(),
                                                         target_pose_.translation(),
                                                         q_, error);

  // the next desired point is the closest within the limits
  double delta = 0;
  for (size_t i=0; i<nr_chain_joints_; i++){
    angles::shortest_angular_distance_with_large_limits(q_(i), q_desired(i), lower_limit_(i), upper_limit_(i), delta);
    q_desired(i) = q_(i) + delta;
  }

  // copy q desired
  {
    std::lock_guard<std::mutex> lock(ik_solution_mutex_);
    q_desired_ = q_desired;
  }

  if (!error.empty()) ROS_ERROR_STREAM_THROTTLE(1.0, "IK failed: " << error);
  if (debug_) ROS_INFO_STREAM_THROTTLE(0.5, ">>> " << q_desired_.transpose());
}

template<class SI, class SH, class CI, class CH, class... T>
void IKControllerBase<SI, SH, CI, CH, T...>::starting(const ros::Time &time) {
  q_ = getJointPositions();
  qd_ = getJointVelocities();
  robot_wrapper->updateState(q_, qd_, true);
  target_pose_ = robot_wrapper->getFramePlacement(controlled_frame_);

  // init first trajectory
  q_desired_ = q_;
  generator_->compute(q_.head(nr_chain_joints_), q_desired_.head(nr_chain_joints_), time.toSec());
  started_ = true;
}

template<class SI, class SH, class CI, class CH, class... T>
void IKControllerBase<SI, SH, CI, CH, T...>::timeTrajectory() {
  double current_time = ros::Time::now().toSec();
  std::lock_guard<std::mutex> lock(ik_solution_mutex_);
  generator_->compute(generator_->get_next_point(current_time),
                      q_desired_.head(nr_chain_joints_),
                      current_time);
}

template<class SI, class SH, class CI, class CH, class... T>
void IKControllerBase<SI, SH, CI, CH, T...>::updateModel() {
  q_ = getJointPositions();
  qd_ = getJointVelocities();
  robot_wrapper->updateState(q_, qd_, true);
  robot_wrapper->computeAllTerms();
  nl_terms_ = robot_wrapper->getNonLinearTerms().head(nr_chain_joints_);
}

template<class SI, class SH, class CI, class CH, class... T>
void IKControllerBase<SI, SH, CI, CH, T...>::updateCommand() {
  updateModel();

  {
    std::lock_guard<std::mutex> lock(ik_solution_mutex_);
    q_next_ = generator_->get_next_point(ros::Time::now().toSec());  
  }
  bool success = true;
  for (size_t i = 0; i < nr_chain_joints_; i++) {
    success &= angles::shortest_angular_distance_with_large_limits(q_(i),
                                                                   q_next_(i),
                                                                   lower_limit_(i),
                                                                   upper_limit_(i),
                                                                   q_err_(i));
  }

  if (!success){
    ROS_ERROR_THROTTLE(1.0, "Failed to compute joint error: are the limits violated?");
    q_err_.setZero();
  }

  Eigen::VectorXd y = Kp_.cwiseProduct(q_err_) - Kd_.cwiseProduct(qd_.head(nr_chain_joints_));
  eff_command_ = nl_terms_ + robot_wrapper->getInertia().topLeftCorner(nr_chain_joints_, nr_chain_joints_) * y;
  pos_command_ = q_ + q_err_;

  if (publish_ros_) publishRos();

  if (debug_) {
    ROS_INFO_STREAM_THROTTLE(5.0, "q_   " << q_.transpose() << std::endl
                               << "q_desired_   " << q_desired_.transpose() << std::endl
                               << "q_error:     " << q_err_.transpose() << std::endl
                               << "p term:      " << Kp_.cwiseProduct(q_err_).transpose() << std::endl
                               << "d term:      " << Kd_.cwiseProduct(qd_.head(nr_chain_joints_)).transpose() << std::endl
                               << "nl terms:    " << nl_terms_.transpose() << std::endl
                               << "M*y:         " << (robot_wrapper->getInertia().topLeftCorner(nr_chain_joints_, nr_chain_joints_) * y).transpose());
  }
}

template<class SI, class SH, class CI, class CH, class... T>
void IKControllerBase<SI, SH, CI, CH, T...>::update(const ros::Time &time, const ros::Duration &period) {
  updateCommand();
  for (size_t i = 0; i < nr_chain_joints_; i++) {
    joint_handles_[i].setCommand(eff_command_[i]);
  }
}

template<class SI, class SH, class CI, class CH, class... T>
Eigen::VectorXd IKControllerBase<SI, SH, CI, CH, T...>::getJointVelocities() const {
  Eigen::VectorXd joint_velocities(robot_wrapper->getDof());
  joint_velocities.setZero();
  for (size_t i = 0; i < nr_chain_joints_; i++) {
    joint_velocities(i) = state_handles_[i].getVelocity();
  }
  return joint_velocities;
}

template<class SI, class SH, class CI, class CH, class... T>
Eigen::VectorXd IKControllerBase<SI, SH, CI, CH, T...>::getJointPositions() const {
  Eigen::VectorXd joint_positions(robot_wrapper->getDof());
  joint_positions.setZero();
  for (size_t i = 0; i < nr_chain_joints_; i++) {
    joint_positions(i) = state_handles_[i].getPosition();
  }
  return joint_positions;
}

template<class SI, class SH, class CI, class CH, class... T>
void IKControllerBase<SI, SH, CI, CH, T...>::publishRos() {
  if (pose_publisher_->trylock()) {
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
  if(q_desired_publisher_->trylock()){
    q_desired_publisher_->msg_.header.stamp = ros::Time::now();
    for (size_t i=0; i<nr_chain_joints_; i++)
      q_desired_publisher_->msg_.position[i] = q_desired_[i];
    q_desired_publisher_->unlockAndPublish();
  }

  if(q_current_publisher_->trylock()){
    q_current_publisher_->msg_.header.stamp = ros::Time::now();
    for (size_t i=0; i<nr_chain_joints_; i++)
      q_current_publisher_->msg_.position[i] = q_[i];
    q_current_publisher_->unlockAndPublish();
  }
   
}

template<class SI, class SH, class CI, class CH, class... T>
bool IKControllerBase<SI, SH, CI, CH, T...>::addCommandHandles(hardware_interface::RobotHW *robot_hw) {
  auto effort_joint_interface = robot_hw->get<CI>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting effort joint interface.");
    return false;
  }
  for (auto &joint_name : joint_names_) {
    joint_handles_.push_back(effort_joint_interface->getHandle(joint_name));
  }
  return true;
}

template<class SI, class SH, class CI, class CH, class... T>
bool IKControllerBase<SI, SH, CI, CH, T...>::addStateHandles(hardware_interface::RobotHW *robot_hw) {
  auto state_interface = robot_hw->get<SI>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get franka state interface");
    return false;
  }

  for (auto &joint_name : joint_names_) {
    state_handles_.push_back(state_interface->getHandle(joint_name));
  }
  return true;
}

template<class SI, class SH, class CI, class CH, class... T>
void IKControllerBase<SI, SH, CI, CH, T...>::stopping(const ros::Time& /*time*/){}

}