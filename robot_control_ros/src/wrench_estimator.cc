/*!
 * @file     wrench_estimator.cpp
 * @author   Giuseppe Rizzi
 * @date     12.12.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control_ros/wrench_estimator.h"
#include <pluginlib/class_list_macros.h>
#include <hardware_interface/joint_state_interface.h>

using namespace hardware_interface;

namespace rc_ros {
class WrenchEstimatorSim
    : public WrenchEstimatorBase<JointStateInterface, JointStateHandle> {};
}  // namespace rc_ros

PLUGINLIB_EXPORT_CLASS(rc_ros::WrenchEstimatorSim, controller_interface::ControllerBase)
