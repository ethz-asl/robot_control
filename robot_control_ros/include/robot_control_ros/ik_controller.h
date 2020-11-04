/*!
 * @file     ik_controller.h
 * @author   Giuseppe Rizzi
 * @date     29.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <robot_control/modeling/robot_wrapper.h>
#include <robot_control/controllers/end_effector_controllers/kdl_ik_ns_controller.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <franka_hw/franka_state_interface.h>

#include <interactive_markers/interactive_marker_server.h>

namespace rc_ros {

template<class StateInterface, class StateHandle, class CommandInterface, class CommandHandle>
class IKControllerBase : public controller_interface::MultiInterfaceController<
    CommandInterface,
    StateInterface> {
 public:

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle, ros::NodeHandle& ctrl_handle) override;
  void starting(const ros::Time&) override;
  virtual void stopping(const ros::Time& /*time*/) {};

  void newTargetCallback(const geometry_msgs::PoseStamped&);
  void updateCommand();

  virtual void update(const ros::Time&, const ros::Duration& period) override;
  void publishRos();

  virtual bool addStateHandles(hardware_interface::RobotHW*);
  virtual bool addCommandHandles(hardware_interface::RobotHW*);

  Eigen::VectorXd getJointVelocities() const;
  Eigen::VectorXd getJointPositions() const;

  bool setTargetCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

 protected:
  int nr_chain_joints_;
  std::shared_ptr<rc::RobotWrapper> robot_wrapper;
  std::shared_ptr<rc::IKNullSpaceController_KDL> controller;

  std::vector<std::string> joint_names_;
  std::vector<CommandHandle> joint_handles_;
  std::vector<StateHandle> state_handles_;

  bool debug_;
  std::mutex target_mutex_;
  pin::SE3 target_pose_;
  pin::SE3 current_pose_;

  std::string frame_id_ = "world";
  std::string controlled_frame_;
  Eigen::VectorXd command_;
  Eigen::VectorXd Kp_, Kd_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd q_desired_;
  Eigen::VectorXd q_nullspace_;

  bool publish_ros_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> pose_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> target_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> q_desired_publisher_;

  ros::Subscriber target_subscriber_;

  Eigen::VectorXd lower_limit_;
  Eigen::VectorXd upper_limit_;
};


template <class StateInterface, class StateHandle>
class IKControllerEffort : public IKControllerBase<StateInterface, StateHandle,
    hardware_interface::EffortJointInterface, hardware_interface::JointHandle> {};

class IKControllerEffortSim : public IKControllerEffort<hardware_interface::JointStateInterface,
                                                        hardware_interface::JointStateHandle> {};

// TODO(giuseppe) robot specific controllers go in their own package
class IKControllerPanda : public IKControllerEffortSim {
  bool addStateHandles(hardware_interface::RobotHW*) override;
};


}

#include <robot_control_ros/impl/ik_controller_impl.h>
