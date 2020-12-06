/*!
 * @file     load_test.cpp
 * @author   Giuseppe Rizzi
 * @date     04.12.2020
 * @version  1.0
 * @brief    description
 */

#include <controller_interface/controller_base.h>
#include <pluginlib/class_loader.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "load_test_node");
  pluginlib::ClassLoader<controller_interface::ControllerBase> controller_loader(
      "controller_interface", "controller_interface::ControllerBase");

  try {
    auto controller =
        controller_loader.createInstance("robot_control_ros/ImpedanceControllerEffortSim");
    ROS_INFO("Plugin loaded");
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("Failed to load plugin");
    ROS_ERROR(ex.what());
    return -1;
  }
  return 0;
}
