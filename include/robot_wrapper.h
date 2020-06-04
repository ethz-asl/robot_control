#include <string>
#include <iostream>
#include <Eigen/Dense>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

namespace pin = pinocchio;
using namespace Eigen;

namespace rc {
class RobotWrapper {
  private:
  pin::Model model;
  pin::Data data;
  Eigen::VectorXd q, v;

  public:
  RobotWrapper(std::string& urdf_path);

  // Accessors
  const Eigen::VectorXd& getQ() const;
  const Eigen::VectorXd& getV() const;

  int getDof() const;

  VectorXd getRandomConfiguration() const;
  VectorXd getNeutralConfiguration() const;

  // Changing state
  void updateState(const VectorXd& new_q, const VectorXd& new_v, bool update_kinematics = true);

  void forwardKinematics();
  Eigen::MatrixXd getJointJacobian(std::string& joint_name);
  Eigen::MatrixXd getFrameJacobian(std::string& frame_name);
};
}

