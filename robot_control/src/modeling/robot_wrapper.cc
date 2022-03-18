#include "robot_control/modeling/robot_wrapper.h"

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace rc {

RobotWrapper::RobotWrapper(const std::string& urdf_path) {
  initFromUrdf(urdf_path);
}
RobotWrapper::RobotWrapper() {}

void RobotWrapper::initFromUrdf(const std::string& urdf_path) {
  pin::urdf::buildModel(urdf_path, model);
  data = pin::Data(model);
  Eigen::VectorXd q0 = getNeutralConfiguration();
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  updateState(q0, v0);
}

void RobotWrapper::initFromXml(const std::string& xml_path, bool verbose) {
  pin::urdf::buildModelFromXML(xml_path, model, verbose);
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

Eigen::VectorXd RobotWrapper::getRandomConfiguration() const {
  return pin::randomConfiguration(model);
}

Eigen::VectorXd RobotWrapper::getNeutralConfiguration() const {
  return pin::neutral(model);
}

std::vector<std::string> RobotWrapper::getJointNames() {
  return model.names;
}

int RobotWrapper::getJointId(const std::string &name) {
  return model.getJointId(name);
}

// Changing state
void RobotWrapper::updateState(const Eigen::VectorXd& new_q, const Eigen::VectorXd& new_v, bool update_kinematics) {
  q = new_q;
  v = new_v;
  if (update_kinematics) forwardKinematics();
}

void RobotWrapper::forwardKinematics() {
  pin::forwardKinematics(model, data, q, v);
  pin::updateFramePlacements(model, data);
}

Eigen::MatrixXd RobotWrapper::getJointJacobian(const std::string& joint_name) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);
  auto joint_id = model.getJointId(joint_name);
  forwardKinematics();
  pin::getJointJacobian(model, data, joint_id, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
  return J;
}

Eigen::MatrixXd RobotWrapper::getFrameJacobian(const std::string& frame_name) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);
  auto frame_id = model.getFrameId(frame_name);
  pin::computeJointJacobians(model, data, q);
  pin::updateFramePlacements(model, data);
  pin::getFrameJacobian(model, data, frame_id, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
  return J;
}

void RobotWrapper::getAllFrameJacobians(const std::string& frame_name, Eigen::MatrixXd& J, Eigen::MatrixXd& dJ) {
  auto frame_id = model.getFrameId(frame_name);
  pin::computeJointJacobians(model, data, q);
  pin::computeJointJacobiansTimeVariation(model, data, q, v);
  pin::updateFramePlacements(model, data);
  pin::getFrameJacobian(model, data, frame_id, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
  pin::getFrameJacobianTimeVariation(model, data, frame_id, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
}

pin::SE3& RobotWrapper::getFramePlacement(const std::string& frame_name) {
  auto frame_id = model.getFrameId(frame_name);
  return data.oMf[frame_id];
}

pin::Motion RobotWrapper::getFrameVelocity(const std::string& frame_name) {
  auto frame_id = model.getFrameId(frame_name);
  return pin::getFrameVelocity(model, data, frame_id, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED);
}

Eigen::MatrixXd RobotWrapper::getInertia() {
  data.M.triangularView<Eigen::StrictlyLower>() =
  data.M.transpose().template triangularView<Eigen::StrictlyLower>();
  return data.M;}

Eigen::VectorXd& RobotWrapper::getNonLinearTerms() { return data.nle; }

void RobotWrapper::computeAllTerms() {
  pin::computeAllTerms(model, data, q, v);
}

}
