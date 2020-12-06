/*!
 * @file     controller_base.h
 * @author   Giuseppe Rizzi
 * @date     04.12.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#pragma once

#include <robot_control/controllers/end_effector_controllers/kdl_ik_ns_controller.h>
#include <robot_control/modeling/robot_wrapper.h>

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <franka_hw/franka_state_interface.h>

#include <interactive_markers/interactive_marker_server.h>
#include <robot_control/utils/trajectory_generator.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace rc_ros {

template <class StateInterface, class StateHandle, class CommandInterface, class CommandHandle,
          class... T>
class ControllerBaseRos
    : public controller_interface::MultiInterfaceController<CommandInterface, StateInterface,
                                                            T...> {
 public:
  ControllerBaseRos() : tf2_listener(tf_buffer){};

  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle,
            ros::NodeHandle &ctrl_handle) override;
  virtual bool init_impl(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle,
                         ros::NodeHandle &ctrl_handle){};

  virtual void starting(const ros::Time &) = 0;
  virtual void update(const ros::Time &, const ros::Duration &period) = 0;
  virtual void stopping(const ros::Time &time) = 0;

 protected:
  bool addStateHandles(hardware_interface::RobotHW *);
  bool addCommandHandles(hardware_interface::RobotHW *);

  Eigen::VectorXd getJointVelocities() const;
  Eigen::VectorXd getJointPositions() const;

  virtual void targetCallback(const geometry_msgs::PoseStamped &msg);
  void updateModel();
  void publishCurrentPose();

 protected:
  std::atomic_bool started_;
  std::shared_ptr<rc::RobotWrapper> robot_wrapper;

  std::vector<std::string> joint_names_;
  std::vector<CommandHandle> joint_handles_;
  std::vector<StateHandle> state_handles_;

  bool debug_;
  std::mutex target_mutex_;
  pin::SE3 target_pose_;
  pin::SE3 current_pose_;

  std::string reference_frame_;
  std::string controlled_frame_;
  Eigen::VectorXd eff_command_;
  Eigen::VectorXd pos_command_;
  Eigen::VectorXd vel_command_;

  bool publish_ros_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> pose_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> target_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> q_desired_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> q_current_publisher_;

  ros::Subscriber target_subscriber_;

  Eigen::VectorXd lower_limit_;
  Eigen::VectorXd upper_limit_;

  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd nl_terms_;

  // to transform the target
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener;
  geometry_msgs::TransformStamped root_link_tf;

  pinocchio::SE3 pose_start_;
  double target_received_time_;
  double target_reached_dt_;
};

template <class SI, class SH, class CI, class CH, class... T>
bool ControllerBaseRos<SI, SH, CI, CH, T...>::init(hardware_interface::RobotHW *robot_hw,
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
  robot_wrapper = std::make_shared<rc::RobotWrapper>();
  robot_wrapper->initFromXml(robot_description);

  eff_command_.setZero(robot_wrapper->getDof());
  vel_command_.setZero(robot_wrapper->getDof());
  pos_command_.setZero(robot_wrapper->getDof());

  if (!ctrl_handle.getParam("joint_names", joint_names_)) {
    ROS_ERROR_STREAM("Set the joint_names parameter.");
    return false;
  }

  if (!ctrl_handle.getParam("controlled_frame", controlled_frame_)) {
    ROS_ERROR_STREAM("Set the controlled_frame parameter.");
    return false;
  }

  if (!ctrl_handle.getParam("reference_frame", reference_frame_)) {
    ROS_ERROR_STREAM("Set the reference_frame parameter.");
    return false;
  }

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

  std::string pose_topic = ctrl_handle.param<std::string>("current_pose_topic", "/current_pose");
  pose_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>>(
      node_handle, pose_topic, 10);

  target_subscriber_ =
      node_handle.subscribe("/target_pose", 10, &ControllerBaseRos::targetCallback, this);
  if (!init_impl(robot_hw, node_handle, ctrl_handle)) {
    return false;
  }
  started_ = false;
  return true;
}

template <class SI, class SH, class CI, class CH, class... T>
bool ControllerBaseRos<SI, SH, CI, CH, T...>::addCommandHandles(
    hardware_interface::RobotHW *robot_hw) {
  auto command_interface = robot_hw->get<CI>();
  if (command_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting effort joint interface.");
    return false;
  }
  for (auto &joint_name : joint_names_) {
    joint_handles_.push_back(command_interface->getHandle(joint_name));
  }
  return true;
}

template <class SI, class SH, class CI, class CH, class... T>
bool ControllerBaseRos<SI, SH, CI, CH, T...>::addStateHandles(
    hardware_interface::RobotHW *robot_hw) {
  auto state_interface = robot_hw->get<SI>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get state interface");
    return false;
  }

  for (auto &joint_name : joint_names_) {
    state_handles_.push_back(state_interface->getHandle(joint_name));
  }
  return true;
}

template <class SI, class SH, class CI, class CH, class... T>
void ControllerBaseRos<SI, SH, CI, CH, T...>::updateModel() {
  q_ = getJointPositions();
  qd_ = getJointVelocities();
  robot_wrapper->updateState(q_, qd_, true);
  robot_wrapper->computeAllTerms();
  nl_terms_ = robot_wrapper->getNonLinearTerms();
}

template <class SI, class SH, class CI, class CH, class... T>
void ControllerBaseRos<SI, SH, CI, CH, T...>::targetCallback(
    const geometry_msgs::PoseStamped &msg) {
  if (!started_) return;

  try {
    root_link_tf = tf_buffer.lookupTransform(reference_frame_, msg.header.frame_id,
                                             ros::Time::now(), ros::Duration(1.0));
  } catch (const tf2::TransformException &exc) {
    ROS_ERROR_STREAM("Tf lookup failed: " << exc.what());
  }

  geometry_msgs::PoseStamped pose_in_root_frame;
  tf2::doTransform(msg, pose_in_root_frame, root_link_tf);

  Eigen::Vector3d translation(pose_in_root_frame.pose.position.x,
                              pose_in_root_frame.pose.position.y,
                              pose_in_root_frame.pose.position.z);
  Eigen::Quaterniond rotation(
      pose_in_root_frame.pose.orientation.w, pose_in_root_frame.pose.orientation.x,
      pose_in_root_frame.pose.orientation.y, pose_in_root_frame.pose.orientation.z);

  target_pose_ = pin::SE3(rotation, translation);
}

template <class SI, class SH, class CI, class CH, class... T>
Eigen::VectorXd ControllerBaseRos<SI, SH, CI, CH, T...>::getJointVelocities() const {
  Eigen::VectorXd joint_velocities(robot_wrapper->getDof());
  joint_velocities.setZero();
  size_t i = 0;
  for (const auto &handle : state_handles_) {
    joint_velocities(i) = handle.getVelocity();
    i++;
  }
  return joint_velocities;
}

template <class SI, class SH, class CI, class CH, class... T>
Eigen::VectorXd ControllerBaseRos<SI, SH, CI, CH, T...>::getJointPositions() const {
  Eigen::VectorXd joint_positions(robot_wrapper->getDof());
  joint_positions.setZero();
  size_t i = 0;
  for (const auto &handle : state_handles_) {
    joint_positions(i) = handle.getPosition();
    i++;
  }
  return joint_positions;
}

template <class SI, class SH, class CI, class CH, class... T>
void ControllerBaseRos<SI, SH, CI, CH, T...>::publishCurrentPose() {
  if (pose_publisher_->trylock()) {
    current_pose_ = robot_wrapper->getFramePlacement(controlled_frame_);
    pose_publisher_->msg_.header.stamp = ros::Time::now();
    pose_publisher_->msg_.header.frame_id = reference_frame_;
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