/*!
 * @file     impedance_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     04.12.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control_ros/impedance_controller.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>

using namespace hardware_interface;

namespace rc_ros {
class ImpedanceControllerEffortSim
    : public ImpedanceControllerRos<JointStateInterface, JointStateHandle, EffortJointInterface,
                                    JointHandle> {
  void updateCommand() override {
    ROS_INFO_STREAM_THROTTLE(1.0, "Effort command: " << eff_command_.transpose());
    size_t i = 0;
    for (auto& handle : joint_handles_) {
      handle.setCommand(eff_command_[i]);
      i++;
    }
  };
};
}  // namespace rc_ros

PLUGINLIB_EXPORT_CLASS(rc_ros::ImpedanceControllerEffortSim, controller_interface::ControllerBase)
