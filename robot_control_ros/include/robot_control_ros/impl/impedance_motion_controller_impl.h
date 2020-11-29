/*!
 * @file     impedance_motion_controller_impl.h
 * @author   Giuseppe Rizzi
 * @date     28.11.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

namespace rc_ros {

template<class SI, class SH, class CI, class CH, class... T>
bool ImpedanceMotionControllerBase<SI, SH, CI, CH, T...>::init(hardware_interface::RobotHW *robot_hw,
                                                               ros::NodeHandle &node_handle,
                                                               ros::NodeHandle &ctrl_handle) {
  IKControllerBase<SI, SH, CI, CH, T...>::init(robot_hw, node_handle, ctrl_handle);

  std::vector<double> Kp_f{};
  if (!ctrl_handle.getParam("admittance_p_gains", Kp_f)) {
    ROS_ERROR("Failed to parse admittance proportional gains.");
    return false;
  }

  std::vector<double> Kd_f{};
  if (!ctrl_handle.getParam("admittance_d_gains", Kd_f)) {
    ROS_ERROR("Failed to parse admittance derivative gains.");
    return false;
  }

  if (Kp_f.size() != 6){
    ROS_ERROR_STREAM("admittance_p_gains have the wrong size: " << Kp_f.size());
    return false;
  }

  if (Kd_f.size() != 6){
    ROS_ERROR_STREAM("admittance_d_gains have the wrong size: " << Kd_f.size());
    return false;
  }

  for(size_t i=0; i<6; i++){
    Kp_f_(i) = Kp_f[i];
    Kd_f_(i) = Kd_f[i];
  }

  std::vector<double> f_threshold{};
  if (!ctrl_handle.getParam("force_threshold", f_threshold)) {
    ROS_ERROR("Failed to parse force threshold: setting to zero.");
  }
  else{
    if (f_threshold.size() != 3){
      ROS_ERROR_STREAM("Force threshold has the wrong dimension: " << f_threshold.size() << ". Setting to zero.");
      force_threshold_.setZero();
    }
    else{
      force_threshold_ = Eigen::Vector3d(f_threshold.data());
      ROS_INFO_STREAM("Force threshold is: " << force_threshold_.transpose());
    }
  }

  std::vector<double> t_threshold{};
  if (!ctrl_handle.getParam("torque_threshold", t_threshold)) {
    ROS_ERROR("Failed to parse torque threshold: setting to zero.");
  }
  else{
    if (t_threshold.size() != 3){
      ROS_ERROR_STREAM("Torque threshold has the wrong dimension: " << t_threshold.size() << ". Setting to zero.");
      torque_threshold_.setZero();
    }
    else{
      torque_threshold_ = Eigen::Vector3d(t_threshold.data());
      ROS_INFO_STREAM("Force threshold is: " << torque_threshold_.transpose());
    }
  }

  ROS_INFO_STREAM("Admittance p gains: " << Kp_f_.transpose());
  ROS_INFO_STREAM("Admittance d gains: " << Kd_f_.transpose());
  ROS_INFO("Impedance Motion Controller successfully initialized");
  return true;
}

template<class SI, class SH, class CI, class CH, class... T>
void ImpedanceMotionControllerBase<SI, SH, CI, CH, T...>::adaptTarget(pinocchio::SE3& target) {
  readMeasurements();

  // threshold
  for(size_t i=0; i<3; i++){
    force_measured_(i) = threshold_value(force_measured_(i), force_threshold_(i));
    torque_measured_(i) = threshold_value(torque_measured_(i), torque_threshold_(i));
  }

  ROS_INFO_STREAM_THROTTLE(1.0, "Tresholded force: " << force_measured_.transpose()
    << ", " << torque_measured_.transpose());

  // compute error
  Eigen::Vector3d delta_pos = Kp_f_.head(3).cwiseProduct(force_measured_);
  Eigen::Vector3d rot_compliant_desired = Kp_f_.tail(3).cwiseProduct(torque_measured_);
  Eigen::Quaterniond delta_rot(1.0, 0.0, 0.0, 0.0);
  double r = rot_compliant_desired.norm();
  if (r > 0) {
    delta_rot.w() = std::cos(r);
    Eigen::Vector3d quatImg = std::sin(r) / r * rot_compliant_desired;
    delta_rot.x() = quatImg[0];
    delta_rot.y() = quatImg[1];
    delta_rot.z() = quatImg[2];
  }

  // find the transform from the compliant frame to the FT sensor
  // use the relative tf between the sensor and the controlled frame
  // to then change the desired target
  pinocchio::SE3 t_sensor_compliant(delta_rot, delta_pos);
  ROS_INFO_STREAM_THROTTLE(1.0, "Trasform from compliant frame to sensor: " << t_sensor_compliant);
  pinocchio::SE3 t_sensor = this->robot_wrapper->getFramePlacement(wrench_frame_);
  pinocchio::SE3 t_controlled = this->robot_wrapper->getFramePlacement(this->controlled_frame_);
  pinocchio::SE3 t_controlled_sensor = t_controlled.actInv(t_sensor);
  ROS_INFO_STREAM_THROTTLE(1.0, "Transform from sensor to controlled frame: " << t_controlled_sensor);
  target = target.act(t_controlled_sensor.act(t_sensor_compliant));

  ROS_INFO_STREAM_THROTTLE(1.0, "Adapting reference");
}

template<class SI, class SH, class CI, class CH, class... T>
double ImpedanceMotionControllerBase<SI, SH, CI, CH, T...>::threshold_value(const double v, const double positive_threshold){
  if (positive_threshold < 0)
    return v;

  // below threshold value is set to zero
  double delta = std::max(v-positive_threshold, -v-positive_threshold);
  if (delta < 0.0)
    return 0.0;

  // above threshold value is set to the signed difference from threshold
  return (v<=0) ? v + positive_threshold : v - positive_threshold;
}


}