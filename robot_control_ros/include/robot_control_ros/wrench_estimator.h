/*!
 * @file     wrench_estimator.h
 * @author   Giuseppe Rizzi
 * @date     12.12.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <robot_control/utils/momentum_observer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

namespace rc_ros {

template <class StateInterface, class StateHandle>
class WrenchEstimatorBase : public controller_interface::Controller<StateInterface> {
 public:
  WrenchEstimatorBase(){};

  bool init(StateInterface *hw, ros::NodeHandle &node_handle,
            ros::NodeHandle &ctrl_handle) override;

  void starting(const ros::Time &) override;
  void update(const ros::Time &, const ros::Duration &period) override;
  void stopping(const ros::Time &time) override {};

 protected:
  bool addStateHandles(StateInterface*);
  void publishWrench();

  Eigen::VectorXd getJointPositions() const;
  Eigen::VectorXd getJointVelocities() const;
  Eigen::VectorXd getJointTorques() const;

 protected:
  std::atomic_bool started_;
  std::unique_ptr<rc::MomentumObserver> wrench_estimator_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>> wrench_publisher_;
  std::vector<std::string> joint_names_;
  std::vector<StateHandle> state_handles_;
  std::string frame_;
  geometry_msgs::WrenchStamped wrench_ros_;
  Eigen::Matrix<double, 6, 1> wrench_;
};

template <class SI, class SH>
bool WrenchEstimatorBase<SI, SH>::init(SI *hw, ros::NodeHandle &node_handle,
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

  std::vector<double> gain;
  if (!ctrl_handle.getParam("gain", gain)) {
    ROS_ERROR_STREAM("Set the gain parameter.");
    return false;
  }
  Eigen::VectorXd gainv(gain.size());
  for (size_t i=0; i<gain.size(); i++){
    gainv(i) = gain[i];
  }

  double alpha;
  if (!ctrl_handle.getParam("alpha", alpha)) {
    ROS_ERROR_STREAM("Set the alpha parameter.");
    return false;
  }

  wrench_estimator_ = std::make_unique<rc::MomentumObserver>(robot_description, gainv, alpha);
  
  if (!ctrl_handle.getParam("joint_names", joint_names_)) {
    ROS_ERROR_STREAM("Set the joint_names parameter.");
    return false;
  }

  if (!ctrl_handle.getParam("frame", frame_)) {
    ROS_ERROR_STREAM("Set the frame parameter.");
    return false;
  }

  bool state_handle_ok = addStateHandles(hw);
  if (!state_handle_ok) {
    ROS_ERROR("Failed to add the joint state handles.");
    return false;
  }

  wrench_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>>(node_handle, "/external_wrench", 10);
  started_ = false;
  return true;
}

template <class SI, class SH>
bool WrenchEstimatorBase<SI, SH>::addStateHandles(
    SI* hw) {
  for (auto &joint_name : joint_names_) {
    state_handles_.push_back(hw->getHandle(joint_name));
  }
  return true;
}

template <class SI, class SH>
void WrenchEstimatorBase<SI, SH>::starting(const ros::Time &){
  wrench_estimator_->reset(getJointPositions(), getJointVelocities());
}

template <class SI, class SH>
void WrenchEstimatorBase<SI, SH>::update(const ros::Time &, const ros::Duration &period){
 wrench_estimator_->estimate_wrench(getJointPositions(), getJointVelocities(), getJointTorques(), frame_);
 wrench_estimator_->get_wrench(wrench_);
 publishWrench();
}

template <class SI, class SH>
Eigen::VectorXd WrenchEstimatorBase<SI, SH>::getJointPositions() const {
  Eigen::VectorXd joint_positions(wrench_estimator_->get_dof());
  joint_positions.setZero();
  size_t i = 0;
  for (const auto &handle : state_handles_) {
    joint_positions(i) = handle.getPosition();
    i++;
  }
  return joint_positions;
}

template <class SI, class SH>
Eigen::VectorXd WrenchEstimatorBase<SI, SH>::getJointVelocities() const {
  Eigen::VectorXd joint_velocities(wrench_estimator_->get_dof());
  joint_velocities.setZero();
  size_t i = 0;
  for (const auto &handle : state_handles_) {
    joint_velocities(i) = handle.getVelocity();
    i++;
  }
  return joint_velocities;
}

template <class SI, class SH>
Eigen::VectorXd WrenchEstimatorBase<SI, SH>::getJointTorques() const {
  Eigen::VectorXd joint_torques(wrench_estimator_->get_dof());
  joint_torques.setZero();
  size_t i = 0;
  for (const auto &handle : state_handles_) {
    joint_torques(i) = handle.getEffort();
    i++;
  }
  return joint_torques;
}

template <class SI, class SH>
void WrenchEstimatorBase<SI, SH>::publishWrench() {
  if (wrench_publisher_->trylock()) {
    wrench_publisher_->msg_.header.frame_id = frame_;
    wrench_publisher_->msg_.header.stamp = ros::Time::now();
    wrench_publisher_->msg_.wrench.force.x = wrench_(0);
    wrench_publisher_->msg_.wrench.force.y = wrench_(1);
    wrench_publisher_->msg_.wrench.force.z = wrench_(2);
    wrench_publisher_->msg_.wrench.torque.x = wrench_(3);
    wrench_publisher_->msg_.wrench.torque.y = wrench_(4);
    wrench_publisher_->msg_.wrench.torque.z = wrench_(5);
    wrench_publisher_->unlockAndPublish();
  }
}
}