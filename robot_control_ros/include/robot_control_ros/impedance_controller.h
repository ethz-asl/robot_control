/*!
 * @file     impedance_controller.h
 * @author   Giuseppe Rizzi
 * @date     04.12.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include "robot_control/controllers/end_effector_controllers/impedance_controller.h"
#include "robot_control_ros/controller_base.h"

namespace rc_ros {

template <class StateInterface, class StateHandle, class CommandInterface, class CommandHandle,
          class... T>
class ImpedanceControllerRos
    : public ControllerBaseRos<StateInterface, StateHandle, CommandInterface, CommandHandle, T...> {
 public:
  ImpedanceControllerRos(){};

  bool init_impl(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle,
                 ros::NodeHandle& ctrl_handle) override;
  virtual void starting(const ros::Time&) override;
  virtual void update(const ros::Time&, const ros::Duration& period) override;
  virtual void stopping(const ros::Time& /*time*/) override{};

  virtual void updateCommand() = 0;

 protected:
  std::shared_ptr<rc::ImpedanceController> controller;
};

template <class SI, class SH, class CI, class CH, class... T>
bool ImpedanceControllerRos<SI, SH, CI, CH, T...>::init_impl(hardware_interface::RobotHW* robot_hw,
                                                             ros::NodeHandle& node_handle,
                                                             ros::NodeHandle& ctrl_handle) {
  controller =
      std::make_shared<rc::ImpedanceController>(this->robot_wrapper, this->controlled_frame_);
  return true;
}

template <class SI, class SH, class CI, class CH, class... T>
void ImpedanceControllerRos<SI, SH, CI, CH, T...>::starting(const ros::Time&) {
  this->updateModel();
  controller->q_nullspace_ = this->q_;
  ROS_INFO_STREAM("Setting nullspace q to current joint position: " << this->q_.transpose());

  this->target_pose_ = this->robot_wrapper->getFramePlacement(this->controlled_frame_);
  this->started_ = true;
}

template <class SI, class SH, class CI, class CH, class... T>
void ImpedanceControllerRos<SI, SH, CI, CH, T...>::update(const ros::Time&,
                                                          const ros::Duration& period) {
  controller->setTaskTarget(this->target_pose_);
  this->updateModel();
  this->eff_command_ = controller->computeCommand();
  updateCommand();
  this->publishCurrentPose();
}
}