#pragma once

#include <robot_control/modeling/robot_wrapper.h>
#include <robot_control/controllers/end_effector_controllers/task_space_controller.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <realtime_tools/realtime_publisher.h>

namespace rc_ros {
class TaskSpaceControllerSim : public controller_interface::MultiInterfaceController<
    hardware_interface::EffortJointInterface,
    hardware_interface::JointStateInterface> {
  public:
  int END_EFFECTOR_INDEX = 6;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle, ros::NodeHandle& ctrl_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  Eigen::VectorXd getJointVelocities() const;
  Eigen::VectorXd getJointPositions() const;
  void publishRos();

  private:
  rc::RobotWrapper* robot_wrapper;
  rc::TaskSpaceController* controller;

  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<hardware_interface::JointStateHandle> state_handles_sim_;

  std::string joint_names_[7] = {"panda_joint1",
    "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5",
    "panda_joint6", "panda_joint7"};

  std::string controlled_frame_;
  pin::SE3 target_pose_;
  pin::SE3 current_pose_;

  bool publish_ros_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> pose_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> target_publisher_;
};
}
