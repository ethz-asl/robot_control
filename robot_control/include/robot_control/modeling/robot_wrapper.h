#pragma once
#include <pinocchio/fwd.hpp>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/spatial/motion.hpp"

namespace pin = pinocchio;
using namespace Eigen;

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

  VectorXd getRandomConfiguration() const;
  VectorXd getNeutralConfiguration() const;
  std::vector<std::string> getJointNames();
  int getJointId(std::string&);

  void forwardKinematics();
  MatrixXd getJointJacobian(std::string& joint_name);
  MatrixXd getFrameJacobian(std::string& frame_name);
  void getAllFrameJacobians(const std::string& frame_name, MatrixXd&, MatrixXd&);
  pin::SE3 getFramePlacement(std::string& frame_name);
  pin::Motion getFrameVelocity(std::string& frame_name);
  MatrixXd getInertia();
  VectorXd getNonLinearTerms();

  // Changing state
  void updateState(const VectorXd& new_q, const VectorXd& new_v, bool update_kinematics = true);
  void computeAllTerms();
};
}

