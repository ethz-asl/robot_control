#include "robot_wrapper.h"

namespace rc {
RobotWrapper::RobotWrapper(std::string& urdf_path) {
  pin::urdf::buildModel(urdf_path, model);
  data = pin::Data(model);
  Eigen::VectorXd q0 = getNeutralConfiguration();
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  updateState(q0, v0);
}

// Accessors
const Eigen::VectorXd& RobotWrapper::getQ() const { return q; }
const Eigen::VectorXd& RobotWrapper::getV() const { return v; }

int RobotWrapper::getDof() const {
  return model.nv;
}

VectorXd RobotWrapper::getRandomConfiguration() const {
  return pin::randomConfiguration(model);
}

VectorXd RobotWrapper::getNeutralConfiguration() const {
  return pin::neutral(model);
}

// Changing state
void RobotWrapper::updateState(const VectorXd& new_q, const VectorXd& new_v, bool update_kinematics) {
  q = new_q;
  v = new_v;
  if (update_kinematics) forwardKinematics();
}

void RobotWrapper::forwardKinematics() {
  pin::forwardKinematics(model, data, q, v);
  pin::updateFramePlacements(model, data);
}

Eigen::MatrixXd RobotWrapper::getJointJacobian(std::string& joint_name) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);
  auto joint_id = model.getJointId(joint_name);
  forwardKinematics();
  pin::computeJointJacobians(model, data, q);
  pin::updateFramePlacements(model, data);
  pin::getJointJacobian(model, data, joint_id, pin::ReferenceFrame::LOCAL, J);
  return J;
}

Eigen::MatrixXd RobotWrapper::getFrameJacobian(std::string& frame_name) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);
  auto frame_id = model.getFrameId(frame_name);
  pin::computeJointJacobians(model, data, q);
  pin::updateFramePlacements(model, data);
  pin::getFrameJacobian(model, data, frame_id, pin::ReferenceFrame::LOCAL, J);
  return J;
}
}
