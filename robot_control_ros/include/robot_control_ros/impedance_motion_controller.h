/*!
 * @file     impedance_motion_controller.h
 * @author   Giuseppe Rizzi
 * @date     28.11.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include "robot_control_ros/ik_controller.h"
#include "hardware_interface/force_torque_sensor_interface.h"
#include "geometry_msgs/Wrench.h"

namespace rc_ros {

template <class StateInterface, class StateHandle, class CommandInterface, class CommandHandle, class ...T>
class ImpedanceMotionControllerBase :
    public IKControllerBase<StateInterface, StateHandle,
                            CommandInterface, CommandHandle,
                            T...> {
public:
  ImpedanceMotionControllerBase(){};
  virtual bool init(hardware_interface::RobotHW* robot_hw,
                    ros::NodeHandle& node_handle,
                    ros::NodeHandle& ctrl_handle);
  virtual void stopping(const ros::Time& /*time*/){};

protected:
  virtual void readMeasurements(){};
  pinocchio::SE3 adaptTarget(const pinocchio::SE3& target) override;
  double threshold_value(const double value, const double positive_threshold);

protected:
  std::string wrench_frame_;
  Eigen::Vector3d force_measured_;
  Eigen::Vector3d torque_measured_;

private:
  Eigen::Matrix<double, 6, 1> Kp_f_;
  Eigen::Matrix<double, 6, 1> Kd_f_;
  Eigen::Vector3d force_threshold_;
  Eigen::Vector3d torque_threshold_;
};

// specialization using FT handle
template<class SI, class SH, class CI, class CH>
class ImpedanceFTController : public ImpedanceMotionControllerBase<SI, SH, CI, CH, hardware_interface::ForceTorqueSensorInterface>{
public:
  using Base = ImpedanceMotionControllerBase<SI, SH, CI, CH, hardware_interface::ForceTorqueSensorInterface>;

private:
  hardware_interface::ForceTorqueSensorHandle ft_handle_;

protected:
  bool addStateHandles(hardware_interface::RobotHW *robot_hw) override {
    bool success = true;
    success &= Base::addStateHandles(robot_hw);

    // add ft handle
    auto ft_interface = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();
    if (ft_interface == nullptr) {
      ROS_ERROR_STREAM("Could not get the force torque sensor interface.");
      success = false;
      return success;
    }
    ft_handle_ = ft_interface->getHandle("ft_sensor");
    return success;
  }

  void readMeasurements() override {
    this->wrench_frame_ = ft_handle_.getFrameId();
    this->force_measured_ = Eigen::Vector3d(ft_handle_.getForce());
    this->torque_measured_ = Eigen::Vector3d(ft_handle_.getTorque());
  }
};

class ImpedanceControllerTest : public ImpedanceMotionControllerBase<hardware_interface::JointStateInterface, hardware_interface::JointStateHandle,
    hardware_interface::EffortJointInterface, hardware_interface::JointHandle>{
private:
  geometry_msgs::Wrench external_wrench_;
  std::mutex external_wrench_mutex_;
  ros::Subscriber external_wrench_sub_;

protected:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle,
            ros::NodeHandle& ctrl_handle) override {
    ImpedanceMotionControllerBase::init(robot_hw, node_handle, ctrl_handle);
    external_wrench_sub_ = ctrl_handle.subscribe<geometry_msgs::Wrench>("external_wrench", 1,
        &ImpedanceControllerTest::measurementCallback, this);

    if (!ctrl_handle.getParam("wrench_frame", this->wrench_frame_)){
      ROS_ERROR("Could not get the wrench frame");
      return false;
    }
    // reset initial external wrench
    external_wrench_.force.x = 0.0;
    external_wrench_.force.y = 0.0;
    external_wrench_.force.z = 0.0;
    external_wrench_.torque.x = 0.0;
    external_wrench_.torque.y = 0.0;
    external_wrench_.torque.z = 0.0;
  }

  void measurementCallback(const geometry_msgs::WrenchConstPtr& msg){
    std::lock_guard<std::mutex> lock(external_wrench_mutex_);
    external_wrench_ = *msg;
  }

  void readMeasurements() override {
    std::lock_guard<std::mutex> lock(external_wrench_mutex_);
    this->force_measured_(0) = external_wrench_.force.x;
    this->force_measured_(1) = external_wrench_.force.y;
    this->force_measured_(2) = external_wrench_.force.z;
    this->torque_measured_(0) = external_wrench_.torque.x;
    this->torque_measured_(1) = external_wrench_.torque.y;
    this->torque_measured_(2) = external_wrench_.torque.z;
    ROS_INFO_STREAM_THROTTLE(1.0, "Updated measurement: f: " 
      << this->force_measured_.transpose()
      << ", t: " << this->torque_measured_.transpose());
  }
};

}

#include <robot_control_ros/impl/impedance_motion_controller_impl.h>
