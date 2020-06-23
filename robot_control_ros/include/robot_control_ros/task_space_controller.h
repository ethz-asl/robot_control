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

#include <franka_hw/franka_state_interface.h>

#include <interactive_markers/interactive_marker_server.h>

namespace rc_ros {

template<class StateInterface, class StateHandle>
class TaskSpaceControllerBase : public controller_interface::MultiInterfaceController<
    hardware_interface::EffortJointInterface,
    StateInterface> {
  public:
  int END_EFFECTOR_INDEX = 6;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle, ros::NodeHandle& ctrl_handle) override;
  void newTargetCallback(const geometry_msgs::PoseStamped&);

  void starting(const ros::Time&);
  void update(const ros::Time&, const ros::Duration& period) override;
  void publishRos();

  virtual bool addStateHandles(hardware_interface::RobotHW*) {};

  Eigen::VectorXd getJointVelocities() const;
  Eigen::VectorXd getJointPositions() const;

  protected:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server;

  void initMarker();
  void markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  std::shared_ptr<rc::RobotWrapper> robot_wrapper;
  std::shared_ptr<rc::TaskSpaceController> controller;

  std::vector<hardware_interface::JointStateHandle> state_handles_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

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

class TaskSpaceController : public TaskSpaceControllerBase<franka_hw::FrankaStateInterface, franka_hw::FrankaStateHandle> {
  bool addStateHandles(hardware_interface::RobotHW*);
};

class TaskSpaceControllerSim : public TaskSpaceControllerBase<hardware_interface::JointStateInterface, hardware_interface::JointStateHandle> {
  bool addStateHandles(hardware_interface::RobotHW*);
};
}

