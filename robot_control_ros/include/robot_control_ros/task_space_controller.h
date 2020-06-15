#include <robot_control/modeling/robot_wrapper.h>
#include <robot_control/controllers/end_effector_controllers/task_space_controller.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace rc_ros {
class TaskSpaceController : public controller_interface::MultiInterfaceController<
    hardware_interface::EffortJointInterface,
    franka_hw::FrankaStateInterface> {
  public:
  int END_EFFECTOR_INDEX = 6;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  Eigen::VectorXd getJointVelocities() const;
  Eigen::VectorXd getJointPositions() const;

  private:
  bool is_real_robot_;
  rc::RobotWrapper* robot_wrapper;
  rc::TaskSpaceController* controller;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  std::vector<hardware_interface::JointStateHandle> state_handles_sim_;

  std::string joint_names_[9] = {"panda_joint1",
    "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5",
    "panda_joint6", "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};

};
}

