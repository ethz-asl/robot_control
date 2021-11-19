#pragma once
#include <pinocchio/fwd.hpp>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/spatial/motion.hpp"

namespace pin = pinocchio;

namespace rc {
class RobotWrapper {
  private:
  pin::Model model;
  pin::Data data;

  public:
  Eigen::VectorXd q, v;
  RobotWrapper(std::string& urdf_path);
  RobotWrapper();

  void initFromUrdf(std::string& urdf_path);
  void initFromXml(std::string& xml_file, bool verbose=false);

  // Accessors
  const Eigen::VectorXd& getQ() const;
  const Eigen::VectorXd& getV() const;

  int getDof() const;

  Eigen::VectorXd getRandomConfiguration() const;
  Eigen::VectorXd getNeutralConfiguration() const;
  std::vector<std::string> getJointNames();
  int getJointId(std::string&);

  void forwardKinematics();
  Eigen::MatrixXd getJointJacobian(std::string& joint_name);
  Eigen::MatrixXd getFrameJacobian(std::string& frame_name);
  void getAllFrameJacobians(const std::string& frame_name, Eigen::MatrixXd&, Eigen::MatrixXd&);
  pin::SE3& getFramePlacement(std::string& frame_name);
  pin::Motion getFrameVelocity(std::string& frame_name);
  Eigen::MatrixXd getInertia();
  Eigen::VectorXd& getNonLinearTerms();

  // Changing state
  void updateState(const Eigen::VectorXd& new_q, const Eigen::VectorXd& new_v, bool update_kinematics = true);
  void computeAllTerms();
};
}

