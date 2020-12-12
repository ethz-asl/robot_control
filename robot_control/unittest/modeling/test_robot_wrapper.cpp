#include <robot_control/modeling/robot_wrapper.h>
#include <Eigen/Dense>
#include <gtest/gtest.h>

using namespace Eigen;

std::string getSourceDir(){
  std::string file_path = __FILE__;
  std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
  return dir_path;
}

TEST(RobotModel, InertiaDot){
  std::string urdfFile = getSourceDir() + "/data/three_planar.urdf";
  rc::RobotWrapper model;
  model.initFromUrdf(urdfFile);
  // numerical differentiation
  Matrix<double, 3, 1> q, qd;
  Matrix<double, 3, 1> q_next, qd_next;
  q << 0, 0, 0;
  qd << 1.0, 1.0, 1.0;
  model.updateState(q, qd, true);
  MatrixXd M = model.getInertia();

  double dt = 0.01;
  q_next = q + dt * qd;
  qd_next = qd;

  model.updateState(q, qd, true);
  MatrixXd M_next = model.getInertia();

  MatrixXd Mdot_numerical = (M_next - M) / dt;



}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
