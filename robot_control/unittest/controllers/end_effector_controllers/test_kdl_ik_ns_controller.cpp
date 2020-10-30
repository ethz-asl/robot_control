/*!
 * @file     test_kdl_ik_ns_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     28.10.2020
 * @version  1.0
 * @brief    description
 */

#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include "robot_control/controllers/end_effector_controllers/kdl_ik_ns_controller.h"

#define DEFAULT_CONFIGURATION 0.0, -0.473, 0.10720, -1.937, -0.1610, 1.48333, 0.75098

using namespace Eigen;

// Fixture
class IKNullSpaceControllerTest : public ::testing::Test
{
 protected:
  std::string robot_description;
  IKNullSpaceControllerTest()
  {
    ros::NodeHandle nh;
    nh.param("robot_description", robot_description, std::string());
  }
};

TEST_F(IKNullSpaceControllerTest, validCommand)
{
  rc::IKNullSpaceController_KDL ctrl(true);
  VectorXd q_nullspace(7), q_nullspace_weights(7);
  q_nullspace << DEFAULT_CONFIGURATION;
  q_nullspace_weights.setConstant(1.0);

  ctrl.initFromXmlString(robot_description, "panda_link0", "panda_hand", q_nullspace, q_nullspace_weights);

  const std::vector<std::pair<Matrix3d, Vector3d>> poses_test{
      {Matrix3d(Quaterniond(-0.2017, 0.6923, -0.3507, 0.5976)), Vector3d(-0.2632, -0.084, 0.0738)},
      {Matrix3d(Quaterniond(0.1898, 0.424, 0.0772, 0.8822)), Vector3d(-0.3649, -0.0012, 0.7972)},
      {Matrix3d(Quaterniond(-0.1169, -0.2111, 0.9704, -0.0125)), Vector3d(0.3814, -0.5876, 0.4489)}
  };

  VectorXd q_out(ctrl.getNumJoints());
  VectorXd q_guess(ctrl.getNumJoints());
  q_guess << DEFAULT_CONFIGURATION;
  std::cout << "Num of joints is: " << ctrl.getNumJoints() << std::endl;
  std::cout << "Configuration guess is: " << q_guess.transpose() << std::endl;

  size_t i = 0;
  double pos_err = 0;
  double rot_err = 0;
  Vector3d pos_out;
  Matrix3d rot_out;

  for (const auto& pose_test : poses_test){
    q_out = ctrl.computeCommand(pose_test.first, pose_test.second, q_guess);
    std::cout << "Computing the forward kinematics." << std::endl;
    ctrl.forwardKinematics(q_out, rot_out, pos_out);

    pos_err = (pos_out - pose_test.second).norm();
    rot_err = AngleAxisd(rot_out.transpose() * pose_test.first).angle();
    std::cout << "Test pose 1 " << std::endl;
    std::cout << "t_des: " << pose_test.second.transpose() << std::endl;
    std::cout << "R_des: " << Quaterniond(pose_test.first).coeffs().transpose() << std::endl;
    std::cout << "q_out: " << q_out.transpose() << std::endl;
    std::cout << "t_err: " << pos_err << std::endl;
    std::cout << "R_err: " << rot_err << std::endl;
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");

  return RUN_ALL_TESTS();
}
