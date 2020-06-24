#include <gtest/gtest.h>
#include <math.h>
#include "robot_control/controllers/end_effector_controllers/kdl_cartesian_velocity_controller.h"

using namespace Eigen;

// Declare a test
TEST(CartesianTestSuite, firstCase)
{
    rc::CartesianVelocityController_KDL ctrl("/home/fjulian/Code/ros_manipulation_ws/src/chimera_app_highlevel_planning/data/models/box_panda_hand.urdf", "panda_link0", "panda_default_EE");
    Matrix<double, 3, 1> velocity_translation;
    velocity_translation << 0.5, 0.0, 0.0;
    Matrix<double, 3, 1> velocity_rotation;
    velocity_rotation << 0.0, 0.0, 0.0;
    Matrix<double, Dynamic, 1> q_in;
    q_in.resize(ctrl.getNumJoints());
    q_in << 0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0, M_PI / 2.0, M_PI / 4.0;
    auto cmd = ctrl.computeCommand(velocity_translation, velocity_rotation, q_in);
}

// Declare another test
// TEST(TestSuite, testCase2)
// {
// <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
