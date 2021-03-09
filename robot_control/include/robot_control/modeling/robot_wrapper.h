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
   using vector_t = Eigen::VectorXd;
   using matrix_t = Eigen::MatrixXd;
  vector_t q, v;
  RobotWrapper(std::string& urdf_path);
  RobotWrapper();

  void initFromUrdf(std::string& urdf_path);
  void initFromXml(std::string& xml_file, bool verbose=false);

  // Accessors
  const vector_t& getQ() const;
  const vector_t& getV() const;

  int getDof() const;

  vector_t getRandomConfiguration() const;
  vector_t getNeutralConfiguration() const;
  std::vector<std::string> getJointNames();
  int getJointId(std::string&);

  void forwardKinematics();
  matrix_t getJointJacobian(std::string& joint_name);
  matrix_t getFrameJacobian(std::string& frame_name);
  void getAllFrameJacobians(const std::string& frame_name, matrix_t&, matrix_t&);
  pin::SE3& getFramePlacement(std::string& frame_name);
  pin::Motion getFrameVelocity(std::string& frame_name);
  matrix_t getInertia();
  vector_t& getNonLinearTerms();

  // Changing state
  void updateState(const vector_t& new_q, const vector_t& new_v, bool update_kinematics = true);
  void computeAllTerms();
};
}

